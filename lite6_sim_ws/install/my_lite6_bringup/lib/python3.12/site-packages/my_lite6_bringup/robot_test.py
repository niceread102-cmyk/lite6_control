"""Minimal spawn test: ground plane + lite6 only, no map, no ROS2"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "/home/v/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.kit"
args_cli.visualizer = ["kit"]
simulation_app = AppLauncher(args_cli).app

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.assets import Articulation, ArticulationCfg

ROBOT_USD = "/home/v/IsaacSim/assets/lite6_complete.usd"

LITE6_CFG = ArticulationCfg(
    prim_path="/World/Robot/UF_ROBOT",
    spawn=sim_utils.UsdFileCfg(
        usd_path=ROBOT_USD,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            articulation_enabled=True,
            enabled_self_collisions=False,
        ),
    ),
    actuators={},
)


def main():
    sim = SimulationContext(SimulationCfg(dt=1 / 60, device="cpu"))

    # 바닥
    sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())


    # 로봇 스폰
    LITE6_CFG.spawn.func("/World/Robot/UF_ROBOT", LITE6_CFG.spawn)


    robot = Articulation(LITE6_CFG)
    sim.reset()


    robot.reset()


    for _ in range(10):
        sim.step()

    count = 0
    while simulation_app.is_running():
        sim.step()
        count += 1
        if count % 120 == 0:
            print(f"[loop] step {count}", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
