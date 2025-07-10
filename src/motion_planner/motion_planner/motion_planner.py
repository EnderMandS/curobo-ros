import torch
from collections import deque
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as CuroboPose
from curobo.types.robot import JointState as CuroboJointState
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_configs_path, load_yaml
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
)

a = torch.empty(1, device="cuda")
tensor_args = TensorDeviceType()

INTERPOLATION_DT = 0.01


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__("motion_planning_node")

        self.setup_curobo()

        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 1
        )

        # Subscribe to goal pose
        self.goal_pose_sub = self.create_subscription(
            Pose, "goal_pose", self.goal_pose_callback, 1
        )

        # Publisher for joint commands
        self.joint_commands_pub = self.create_publisher(JointState, "joint_commands", 1)

        self.publish_timer = self.publish_timer = self.create_timer(
            INTERPOLATION_DT, self.publish_trajectory_callback
        )
        self.trajectory_queue = deque()

        self.get_logger().info("Motion planning node ready")

    def setup_curobo(self):
        """Initialize cuRobo motion generator"""
        config_file_name = "franka.yml"
        config_file = load_yaml(os.path.join(get_robot_configs_path(), config_file_name))
        urdf_file = config_file["robot_cfg"]["kinematics"]["urdf_path"]
        base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
        ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]
        robot_cfg = RobotConfig.from_basic(urdf_file, base_link, ee_link, tensor_args)

        self.get_logger().debug(f"Config path file: {os.path.join(get_robot_configs_path(), config_file_name)}")
        self.get_logger().debug(f"Robot config: {robot_cfg}")

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg=robot_cfg,
            interpolation_dt=INTERPOLATION_DT,
            tensor_args=tensor_args,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.warmup()

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self.current_joint_state = msg

    def goal_pose_callback(self, msg):
        """Callback for goal pose - triggers motion planning"""
        try:
            # Check if we have current joint state
            if self.current_joint_state is None:
                self.get_logger().warn(
                    "No joint state available. Make sure joint_states topic is publishing."
                )
                return

            self.get_logger().info(
                f"Received goal pose: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]"
            )

            # Convert ROS messages to cuRobo types
            goal_pose = self.ros_pose_to_curobo_pose(msg)
            start_state = self.ros_joint_state_to_curobo_joint_state(
                self.current_joint_state
            )

            # Plan motion
            result = self.motion_gen.plan_single(
                start_state, goal_pose, MotionGenPlanConfig(max_attempts=1)
            )

            if result.success:
                self.trajectory_queue = deque(result.get_interpolated_plan())
                self.get_logger().info(
                    f"Planned traj len: {len(self.trajectory_queue)}"
                )
            else:
                self.get_logger().error("Motion planning failed")

        except Exception as e:
            self.get_logger().error(f"Error during motion planning: {str(e)}")

    def publish_trajectory_callback(self):
        """Timer callback to publish trajectory points"""
        if len(self.trajectory_queue) == 0:
            return

        point = self.trajectory_queue.popleft()
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        joint_state_msg.position = point.position.cpu().numpy().flatten().tolist()

        self.joint_commands_pub.publish(joint_state_msg)

    def ros_pose_to_curobo_pose(self, ros_pose):
        """Convert ROS Pose to cuRobo Pose"""
        return CuroboPose.from_list(
            [
                ros_pose.position.x,
                ros_pose.position.y,
                ros_pose.position.z,
                ros_pose.orientation.w,
                ros_pose.orientation.x,
                ros_pose.orientation.y,
                ros_pose.orientation.z,
            ]
        )

    def ros_joint_state_to_curobo_joint_state(self, ros_joint_state):
        """Convert ROS JointState to cuRobo JointState"""
        if len(ros_joint_state.position) == 0:
            # Default to zeros if no joint state provided
            positions = torch.zeros(1, 6).cuda()
            joint_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
        else:
            positions = torch.tensor(ros_joint_state.position).unsqueeze(0).cuda()
            joint_names = ros_joint_state.name

        return CuroboJointState.from_position(positions, joint_names=joint_names)

    def curobo_trajectory_to_ros_trajectory(self, curobo_traj):
        """Convert cuRobo trajectory to ROS JointTrajectory"""
        ros_traj = JointTrajectory()
        ros_traj.header.stamp = self.get_clock().now().to_msg()
        ros_traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        for i, point in enumerate(curobo_traj):
            traj_point = JointTrajectoryPoint()
            traj_point.positions = point.position.cpu().numpy().flatten().tolist()
            traj_point.time_from_start.sec = int(i * 0.01)
            traj_point.time_from_start.nanosec = int((i * 0.01 - int(i * 0.01)) * 1e9)
            ros_traj.points.append(traj_point)

        return ros_traj


def main(args=None):
    rclpy.init(args=args)

    motion_planning_node = MotionPlanningNode()

    try:
        rclpy.spin(motion_planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        motion_planning_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
