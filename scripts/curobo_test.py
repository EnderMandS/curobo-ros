import torch
import os
import traceback

# cuRobo
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
)
from curobo.util_file import get_robot_configs_path, load_yaml
from curobo.types.robot import RobotConfig
from curobo.types.base import TensorDeviceType
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

from loguru import logger

tensor_args = TensorDeviceType()

robot_config_file = "franka.yml"
TEST_NUM = 20
if TEST_NUM > 1:
    logger.remove()
    logger.add(lambda msg: print(msg, end=""), level="INFO")

world_config = {
    "mesh": {
        "base_scene": {
            "pose": [10.5, 0.080, 1.6, 0.043, -0.471, 0.284, 0.834],
            "file_path": "scene/nvblox/srl_ur10_bins.obj",
        },
    },
    "cuboid": {
        "table": {
            "dims": [5.0, 5.0, 0.2],  # x, y, z
            "pose": [0.0, 0.0, -0.1, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
        },
    },
}

logger.info(f"Testing robot : {robot_config_file}, with test_num: {TEST_NUM}")

config_file = load_yaml(os.path.join(get_robot_configs_path(), robot_config_file))
urdf_file = config_file["robot_cfg"]["kinematics"]["urdf_path"]
base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]
robot_cfg = RobotConfig.from_basic(urdf_file, base_link, ee_link, tensor_args)

Fk_success = False
Ik_success = False
MotionGen_success = False

# Fk
logger.info("\n\nCurobo test: Forward Kinematics ----------------------------------")
try:
    kin_model = CudaRobotModel(robot_cfg.kinematics)

    q = torch.rand((TEST_NUM, kin_model.get_dof()), **(tensor_args.as_torch_dict()))
    logger.debug(f"Joint fk configuration: {q}")

    out = kin_model.get_state(q)
    logger.debug(f"Forward kinematics output pose: {out.ee_pose}")
    logger.info("Fk test passed.")
    Fk_success = True
except Exception as e:
    logger.error(f"Error in Forward Kinematics: {e}")
    logger.info(traceback.format_exc())

# Ik
logger.info("\n\nCurobo test: Inverse Kinematics ----------------------------------")
try:
    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg,
        world_config,
        rotation_threshold=0.05,
        position_threshold=0.005,
        num_seeds=20,
        self_collision_check=True,
        self_collision_opt=True,
        tensor_args=tensor_args,
        use_cuda_graph=True,
    )
    ik_solver = IKSolver(ik_config)

    q_sample = ik_solver.sample_configs(TEST_NUM)
    kin_state = ik_solver.fk(q_sample)

    goal = Pose(kin_state.ee_position, kin_state.ee_quaternion)
    # goal = out.ee_pose

    result = ik_solver.solve_batch(goal)
    q_solution = result.solution[result.success]

    logger.debug(f"success: {result.success}")
    if all(result.success):
        logger.debug(f"IK solution: {q_solution}")
        logger.info("Inverse Kinematics test passed.")
        Ik_success = True
    else:
        logger.warning(
            "Inverse Kinematics failed to find a solution. Try without self collision check."
        )
        ik_config = IKSolverConfig.load_from_robot_config(
            robot_cfg,
            world_config,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=False,
            self_collision_opt=False,
            tensor_args=tensor_args,
            use_cuda_graph=True,
        )
        ik_solver = IKSolver(ik_config)
        result = ik_solver.solve_batch(out.ee_pose)
        q_solution = result.solution[result.success]

        logger.debug(f"success: {result.success}")
        if all(result.success):
            logger.warning(f"success without self collision check: {result.success}")
            logger.debug(f"IK solution: {q_solution}")
            logger.info("Inverse Kinematics test passed.")
            Ik_success = True
        else:
            logger.error(
                "Inverse Kinematics failed to find a solution even without self collision check."
            )

except Exception as e:
    logger.error(f"Error in Inverse Kinematics: {e}")
    logger.info(traceback.format_exc())

# Motion Generation
logger.info("\n\nCurobo test: Motion Generation ----------------------------------")
try:
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg=robot_cfg,
        interpolation_dt=0.01,
        self_collision_check=True,
        self_collision_opt=True,
        tensor_args=tensor_args,
        use_cuda_graph=False,
    )
    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()

    start_state = JointState.from_position(
        torch.tensor(
            config_file["robot_cfg"]["kinematics"]["cspace"]["retract_config"][
                : -len(config_file["robot_cfg"]["kinematics"]["lock_joints"])
            ]
        )
        .cuda()
        .repeat(TEST_NUM, 1),
        joint_names=config_file["robot_cfg"]["kinematics"]["cspace"]["joint_names"],
    )

    # result = motion_gen.plan_single(start_state, out.ee_pose, MotionGenPlanConfig(max_attempts=3))
    result = motion_gen.plan_batch(
        start_state, goal, MotionGenPlanConfig(max_attempts=1)
    )

    logger.debug(f"success: {result.success}")
    if all(result.success):
        logger.info("Motion Generation test passed.")
        MotionGen_success = True
    else:
        logger.error("Motion Generation failed to find a valid trajectory.")
except Exception as e:
    logger.error(f"Error in Motion Generation: {e}")
    logger.info(traceback.format_exc())

# Summary of results
logger.info("\n\nCurobo test summary:")
logger.info(f"Forward Kinematics test: {Fk_success}")
logger.info(f"Inverse Kinematics test: {Ik_success}")
logger.info(f"Motion  Generation test: {MotionGen_success}")
