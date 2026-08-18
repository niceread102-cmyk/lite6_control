import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "/home/v/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.kit"
args_cli.visualizer = ["kit"]
simulation_app = AppLauncher(args_cli).app

import omni.usd, omni.kit.app, omni.kit.commands, omni.timeline
from pxr import Gf, Sdf, UsdGeom

ASSETS = "/home/v/IsaacSim/assets"
stage  = omni.usd.get_context().get_stage()

omni.kit.commands.execute("AddPhysicsSceneCommand", stage=stage, path="/World/PhysicsScene")
omni.kit.commands.execute("AddGroundPlaneCommand",  stage=stage,
    planePath="/World/GroundPlane", axis="Z", size=1500.0,
    position=Gf.Vec3f(0,0,0), color=Gf.Vec3f(0.5,0.5,0.5))
UsdGeom.Xform.Define(stage, "/World/Light")
stage.DefinePrim("/World/Light/DomeLight", "DomeLight") \
    .CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

for name, path, pos in [
    ("Aruco0", f"{ASSETS}/aruco_0.usd", Gf.Vec3d(0.3,  0.1, 0.0)),
    ("Aruco1", f"{ASSETS}/aruco_1.usd", Gf.Vec3d(0.3, -0.1, 0.0)),
]:
    p = stage.DefinePrim(f"/World/{name}", "Xform")
    p.GetReferences().AddReference(path)
    UsdGeom.Xformable(p).AddTranslateOp().Set(pos)

omni.timeline.get_timeline_interface().play()
print("[INFO] ArUco test running", flush=True)

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
