"""Isaac Sim 6.0.0 + Isaac Lab 3.0.0-beta — Lite6 ROS2 bridge"""
import argparse
import threading

import torch
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "/home/v/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.kit"
args_cli.visualizer = ["kit"]
simulation_app = AppLauncher(args_cli).app

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.assets import AssetBaseCfg, Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

# ── paths ────────────────────────────────────────────────────────
MAP_USD   = "/home/v/IsaacSim/assets/my_lab_conveyor.usd"
ROBOT_USD = "/home/v/IsaacSim/assets/lite6_complete.usd"

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
INIT_POS    = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]  # rad

# ── environment config ───────────────────────────────────────────
@configclass
class MyEnvironmentSceneCfg:
    conveyor = AssetBaseCfg(
        prim_path="/World/my_lab_conveyor",
        spawn=sim_utils.UsdFileCfg(usd_path=MAP_USD),
    )

# ── robot config ─────────────────────────────────────────────────
# rot: quaternion (x, y, z, w) — -90 deg around Z
LITE6_CFG = ArticulationCfg(
    prim_path="/World/Robot/UF_ROBOT",
    spawn=sim_utils.UsdFileCfg(usd_path=ROBOT_USD),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(-5.1, 1.3, 0.0),
        rot=(0.0, 0.0, -0.7071, 0.7071),
        joint_pos={
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 1.57,
            "joint4": 0.0,
            "joint5": 1.57,
            "joint6": 0.0
        },
    ),
    actuators={
        "j12": ImplicitActuatorCfg(
            joint_names_expr=["joint1", "joint2"],
            stiffness=1600.0, damping=80.0, effort_limit_sim=3400.0,
        ),
        "j345": ImplicitActuatorCfg(
            joint_names_expr=["joint3", "joint4", "joint5"],
            stiffness=1600.0, damping=51.0, effort_limit_sim=3400.0,
        ),
        "j6": ImplicitActuatorCfg(
            joint_names_expr=["joint6"],
            stiffness=1600.0, damping=32.0, effort_limit_sim=3400.0,
        ),
        # "gripper": ImplicitActuatorCfg(
        #     joint_names_expr=["finger_joint1", "finger_joint2"],
        #     stiffness=200.0, damping=10.0, effort_limit_sim=20.0,
        # ),
    },
)

# ── camera config ────────────────────────────────────────────────
camera_cfg = TiledCameraCfg(
    prim_path="/World/Robot/head_camera",
    offset=TiledCameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
    update_period=0.1,
    height=480,
    width=640,
    data_types=["rgb", "depth"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=24.0,
        focus_distance=400.0,
        horizontal_aperture=20.955,
        clipping_range=(0.1, 1.0e5),
    ),
)


# ── ROS2 ─────────────────────────────────────────────────────────
class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge")
        self._targets = list(INIT_POS)
        self._lock = threading.Lock()
        self._idx_map = {n: i for i, n in enumerate(JOINT_NAMES)}
        # self.create_subscription(
        #     JointTrajectory,
        #     "/lite6_traj_controller/joint_trajectory",
        #     self._cb, 10,
        # )

        self.create_subscription(
            JointTrajectory,
            "/lite6_sub",
            self._cb, 10,
        )

    def _cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        with self._lock:
            for name, pos in zip(msg.joint_names, msg.points[0].positions):
                if name in self._idx_map:
                    self._targets[self._idx_map[name]] = float(pos)

    def get_targets(self):
        with self._lock:
            return list(self._targets)


# ── main ─────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = RobotBridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    import omni.usd
    import omni.kit.app

    omni.usd.get_context().open_stage(MAP_USD)
    for _ in range(5):
        simulation_app.update()

    _stage = omni.usd.get_context().get_stage()
    for _prim in list(_stage.Traverse()):
        if _prim.IsValid() and _prim.GetTypeName() == "PhysicsScene":
            _stage.RemovePrim(_prim.GetPath())

    sim = SimulationContext(SimulationCfg(dt=1 / 60, device="cpu"))

    robot = Articulation(LITE6_CFG)
    

    sim.reset()
    robot.reset()


    # sim_joint_names = robot.data.joint_names 
    # joint_ids = [sim_joint_names.index(name) for name in JOINT_NAMES]


    while simulation_app.is_running():
        # pos = torch.tensor([node.get_targets()], dtype=torch.float32, device=sim.device)
        # robot.set_joint_position_target_index(target=pos, joint_ids=joint_ids)
        # robot.write_data_to_sim()
        sim.step()
        # robot.update(sim.get_physics_dt())

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
