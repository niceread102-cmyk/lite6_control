"""Isaac Lab 100% Standard - Lite6 ROS2 Bridge"""
import argparse
import threading
import torch

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
simulation_app = AppLauncher(args_cli).app

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.actuators import ImplicitActuatorCfg

# ── ROS2 Node ──────────────────────────────────────────────────
class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge")
        self.joint_targets = None
        self.create_subscription(JointTrajectory, "/lite6_traj_controller/joint_trajectory", self._cb, 10)

    def _cb(self, msg: JointTrajectory):
        if not msg.points:
            return

        self.joint_targets = msg.points[0].positions

# ── Scene Configuration (Isaac Lab의 핵심) ─────────────────────
class MySceneCfg(InteractiveSceneCfg):

        (usd_path="/home/v/IsaacSim/assets/my_lab_conveyor.usd")


    robot = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/v/xarm_ros2/xarm_description/urdf/lite6_complete.usd",

            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-5.1, 1.3, 0.0),
            rot=(0.707, 0.0, 0.0, -0.707),

            joint_pos={
                "joint.*": 0.0, 
                "joint3": 1.57,
                "joint5": 1.57,
                "finger.*": 0.0
            },
        ),
        actuators={
            # 반복문 없이 모든 관절의 Stiffness/Damping을 한 번에 제어
            "lite6_motors": ImplicitActuatorCfg(
                joint_names_expr=["joint.*", "finger.*"],
                stiffness=1600.0,
                damping=80.0,
            )
        }
    )

# ── Main Loop ──────────────────────────────────────────────────
def main():
    rclpy.init()
    node = RobotBridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    sim_cfg = sim_utils.SimulationCfg(dt=0.01)
    sim = sim_utils.SimulationContext(sim_cfg)

    scene_cfg = MySceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()

    while simulation_app.is_running():

        if node.joint_targets is not None:
            targets_tensor = torch.tensor([node.joint_targets], device=sim.device)
            scene["robot"].set_joint_position_target(targets_tensor)

        scene.write_data_to_sim()
        sim.step()
        scene.update(dt=sim_cfg.dt)

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    main()