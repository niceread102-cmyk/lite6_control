"""Isaac Sim 6.0.0 — lite6_complete.usda 로드"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "/home/v/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.kit"
args_cli.visualizer = ["kit"]
simulation_app = AppLauncher(args_cli).app

import omni.kit.commands, omni.timeline, omni.usd
from pxr import Gf, Sdf, UsdPhysics

USD_ROBOT = "/home/v/xarm_ros2/xarm_description/urdf/lite6_complete/lite6_complete.usda"


def main():
    # USD를 직접 열기 (reference 아님 — joint body 절대경로 유지)
    omni.usd.get_context().open_stage(USD_ROBOT)
    for _ in range(20):
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()

    omni.kit.commands.execute("AddPhysicsSceneCommand", stage=stage, path="/World/PhysicsScene")
    omni.kit.commands.execute("AddGroundPlaneCommand", stage=stage,
        planePath="/World/GroundPlane", axis="Z", size=1500.0,
        position=Gf.Vec3f(0, 0, 0), color=Gf.Vec3f(0.3, 0.3, 0.3))

    # fixBase
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.CreateAttribute("physxArticulation:fixBase", Sdf.ValueTypeNames.Bool).Set(True)
            print(f"[INFO] fixBase: {prim.GetPath()}", flush=True)
            break

    omni.timeline.get_timeline_interface().play()
    print("[INFO] 시뮬레이션 시작", flush=True)

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
