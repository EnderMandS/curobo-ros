import torch
import os
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from planner_interface.srv import Motion, Fk, Ik

from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState as CuroboJointState
from curobo.types.math import Pose as CuroboPose
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.util_file import get_robot_configs_path, load_yaml
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
)
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

a = torch.empty(1, dtype=torch.float32, device="cuda")
tensor_args = TensorDeviceType()


class PlannerServer(Node):

    def __init__(self):
        super().__init__("curobo_planner_node")

        self.interpolation_dt = self.declare_parameter("interpolation_dt", 0.01).value
        self.config_file_name = self.declare_parameter(
            "config_file_name", "franka.yml"
        ).value
        self.max_attempts = self.declare_parameter("max_attempts", 3).value

        service_base_name = self.declare_parameter("service_base_name", "planner").value
        motion_service_name = service_base_name + "/motion"
        ik_service_name = service_base_name + "/ik"
        fk_service_name = service_base_name + "/fk"

        self.get_logger().info(
            f"Initializing PlannerServer with config file: {self.config_file_name}, "
        )
        self.get_logger().info(f"Interpolation dt: {self.interpolation_dt}")
        self.get_logger().info(f"Max attempts for motion planning: {self.max_attempts}")

        self.setup_curobo()

        self.motion_srv = self.create_service(
            Motion, motion_service_name, self.srv_motion_callback
        )
        self.ik_srv = self.create_service(Ik, ik_service_name, self.srv_ik_callback)
        self.fk_srv = self.create_service(Fk, fk_service_name, self.srv_fk_callback)

        self.get_logger().info(
            f"Motion planning service {motion_service_name}, {ik_service_name}, {fk_service_name} ready"
        )

    def setup_curobo(self):
        """Initialize cuRobo motion generator"""
        try:
            config_file = load_yaml(
                os.path.join(get_robot_configs_path(), self.config_file_name)
            )
            urdf_file = config_file["robot_cfg"]["kinematics"]["urdf_path"]
            base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
            ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]
            robot_cfg = RobotConfig.from_basic(
                urdf_file, base_link, ee_link, tensor_args
            )

            self.cspace_joints = config_file["robot_cfg"]["kinematics"]["cspace"][
                "joint_names"
            ]

            self.get_logger().info(
                f"Config path file: {os.path.join(get_robot_configs_path(), self.config_file_name)}"
            )
            self.get_logger().info(
                f"cspace joints: {config_file['robot_cfg']['kinematics']['cspace']['joint_names']}"
            )
            self.get_logger().debug(f"Robot config: {robot_cfg}")

            motion_gen_config = MotionGenConfig.load_from_robot_config(
                robot_cfg=robot_cfg,
                interpolation_dt=self.interpolation_dt,
                self_collision_check=True,
                self_collision_opt=True,
                tensor_args=tensor_args,
            )
            self.motion_gen = MotionGen(motion_gen_config)
            self.motion_gen.warmup()

            ik_config = IKSolverConfig.load_from_robot_config(
                robot_cfg,
                None,
                rotation_threshold=self.declare_parameter(
                    "rotation_threshold", 0.05
                ).value,
                position_threshold=self.declare_parameter(
                    "position_threshold", 0.005
                ).value,
                num_seeds=self.declare_parameter("num_seeds", 20).value,
                self_collision_check=True,
                self_collision_opt=True,
                tensor_args=tensor_args,
                use_cuda_graph=True,
            )
            self.ik_solver = IKSolver(ik_config)

            # Initialize ik CUDA graph
            kin_state = self.ik_solver.fk(self.ik_solver.sample_configs(1))
            result = self.ik_solver.solve_batch(
                CuroboPose(kin_state.ee_position, kin_state.ee_quaternion)
            )
            if not result.success:
                raise RuntimeError("Failed to initialize IK solver with CUDA graph")

            self.fk_model = CudaRobotModel(robot_cfg.kinematics)

        except Exception as e:
            self.get_logger().error(f"Failed to initialize cuRobo: {str(e)}")
            raise RuntimeError("cuRobo initialization failed") from e

    def srv_motion_callback(self, request: Motion.Request, response: Motion.Response):
        """Service callback for motion planning"""
        self.get_logger().info(f"Received request for motion planning")
        try:
            start_state = self.ros_joint_state_to_curobo_joint_state(
                request.start_state
            )
            goal_pose = self.ros_pose_to_curobo_pose(request.goal_pose)

            result = self.motion_gen.plan_single(
                start_state,
                goal_pose,
                MotionGenPlanConfig(max_attempts=self.max_attempts),
            )

            if result.success:
                response.success = True
                response.trajectory = self.curobo_trajectory_to_ros_trajectory(
                    result.get_interpolated_plan()
                )
                self.get_logger().info(
                    f"Planned trajectory length: {len(result.get_interpolated_plan())}"
                )
            else:
                response.success = False
                self.get_logger().error("Motion planning failed")

        except Exception as e:
            self.get_logger().error(f"Error during motion planning: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            response.success = False

        return response

    def srv_ik_callback(self, request: Ik.Request, response: Ik.Response):
        self.get_logger().info(f"Received request for IK computation")
        try:
            pose = self.ros_pose_to_curobo_pose(request.pose)
            result = self.ik_solver.solve_batch(pose)
            if result.success:
                response.success = True
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = self.cspace_joints
                joint_state.position = (
                    result.solution[result.success].cpu().numpy().flatten().tolist()
                )
                response.joint_state = joint_state
            else:
                response.success = False
                self.get_logger().error("IK computation failed")

        except Exception as e:
            self.get_logger().error(f"Error during IK computation: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            response.success = False

        return response

    def srv_fk_callback(self, request: Fk.Request, response: Fk.Response):
        self.get_logger().info(f"Received request for FK computation")
        try:
            joint_state = torch.tensor(
                request.joint_state.position,
                **(tensor_args.as_torch_dict()),
            ).unsqueeze(0)
            result = self.fk_model.get_state(joint_state)
            position = result.ee_position[0].cpu().numpy().tolist()
            quaternion = result.ee_quaternion[0].cpu().numpy().tolist()
            response.success = True
            response.pose.position.x = position[0]
            response.pose.position.y = position[1]
            response.pose.position.z = position[2]
            response.pose.orientation.w = quaternion[0]
            response.pose.orientation.x = quaternion[1]
            response.pose.orientation.y = quaternion[2]
            response.pose.orientation.z = quaternion[3]

        except Exception as e:
            self.get_logger().error(f"Error during FK computation: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            response.success = False

        return response

    def ros_pose_to_curobo_pose(self, ros_pose: Pose) -> CuroboPose:
        """Convert ROS Pose to cuRobo Pose"""
        return CuroboPose.from_list(
            [
                ros_pose.position.x,
                ros_pose.position.y,
                ros_pose.position.z,
                ros_pose.orientation.x,
                ros_pose.orientation.y,
                ros_pose.orientation.z,
                ros_pose.orientation.w,
            ],
            tensor_args=tensor_args,
            q_xyzw=True,
        )

    def ros_joint_state_to_curobo_joint_state(
        self, ros_joint_state: JointState
    ) -> CuroboJointState:
        """Convert ROS JointState to cuRobo JointState"""
        positions = torch.tensor(ros_joint_state.position).unsqueeze(0).cuda()
        joint_names = ros_joint_state.name

        return CuroboJointState.from_position(positions, joint_names=joint_names)

    def curobo_trajectory_to_ros_trajectory(self, curobo_traj) -> JointTrajectory:
        """Convert cuRobo trajectory to ROS JointTrajectory"""
        ros_traj = JointTrajectory()
        ros_traj.header.stamp = self.get_clock().now().to_msg()
        ros_traj.joint_names = self.cspace_joints

        num_waypoints = curobo_traj.position.shape[0]
        for i in range(num_waypoints):
            traj_point = JointTrajectoryPoint()
            traj_point.positions = (
                curobo_traj.position[i].cpu().numpy().flatten().tolist()
            )
            time_sec = i * self.interpolation_dt
            traj_point.time_from_start.sec = int(time_sec)
            traj_point.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)
            ros_traj.points.append(traj_point)

        return ros_traj


def main(args=None):
    rclpy.init(args=args)

    planner_server = PlannerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(planner_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        planner_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
