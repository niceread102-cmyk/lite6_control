import argparse
import os
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "/home/v/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.kit"
args_cli.visualizer = ["kit"]
simulation_app = AppLauncher(args_cli).app

import omni.kit.app, omni.kit.commands, omni.timeline, omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Usd

_pkg      = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_LITE6 = os.path.join(_pkg, "assets/lite6_with_gripper_d435i_v2_abs/lite6_with_gripper_d435i_v2_abs.usda")
ROBOT     = "/World/Robot"
ARTIC     = "/World/Robot/Geometry/world/link_base"

stage = omni.usd.get_context().get_stage()

omni.kit.commands.execute("AddPhysicsSceneCommand", stage=stage, path="/World/PhysicsScene")

# Newton 익스텐션 활성화
_mgr = omni.kit.app.get_app().get_extension_manager()
for _ext in ("omni.usd.schema.mujoco", "omni.usd.schema.newton", "isaacsim.physics.newton"):
    _mgr.set_extension_enabled_immediate(_ext, True)
omni.kit.app.get_app().update()

# collide() API 버전 불일치 monkey-patch
import inspect
import newton as _newton
_real_collide = _newton.Model.collide
_valid_collide_params = set(inspect.signature(_real_collide).parameters.keys())
def _patched_collide(self, *args, **kwargs):
    return _real_collide(self, *args, **{k: v for k, v in kwargs.items() if k in _valid_collide_params})
_newton.Model.collide = _patched_collide

from isaacsim.physics.newton import acquire_stage, XPBDSolverConfig
_newton_stage = acquire_stage()
if _newton_stage is not None:
    _newton_stage.cfg.solver_cfg = XPBDSolverConfig()
    print("[INFO] Newton XPBD solver configured", flush=True)
else:
    print("[WARN] acquire_stage() returned None", flush=True)

omni.kit.commands.execute("AddGroundPlaneCommand", stage=stage,
    planePath="/World/GroundPlane", axis="Z", size=1500.0,
    position=Gf.Vec3f(0,0,0), color=Gf.Vec3f(0.2,0.2,0.2))
UsdGeom.Xform.Define(stage, "/World/Light")
stage.DefinePrim("/World/Light/DomeLight","DomeLight") \
    .CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

rp = stage.DefinePrim(ROBOT, "Xform")
rp.GetReferences().AddReference(USD_LITE6)
rp.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physics")

omni.kit.app.get_app().update()

# 중력 테스트용 큐브 (Newton 초기화 전에 USD에 추가해야 인식됨)
from pxr import UsdGeom as _UG, UsdPhysics as _UP2
_cube = _UG.Cube.Define(stage, "/World/TestCube")
_cube.GetSizeAttr().Set(0.1)
_UG.Xformable(_cube).AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 1.0))
_UP2.RigidBodyAPI.Apply(_cube.GetPrim())
_UP2.CollisionAPI.Apply(_cube.GetPrim())
_UP2.MassAPI.Apply(_cube.GetPrim()).CreateMassAttr().Set(1.0)
print("[INFO] Test cube added at (0.5, 0, 1.0) - should fall under gravity", flush=True)

# Joint drives (없으면 관절이 중력에 자유 낙하)
from pxr import UsdPhysics as _UP
for jname, (axis, ke, kd, fmax) in {
    "joint1":       ("angular", 1600., 80.,  3400.),
    "joint2":       ("angular", 1600., 80.,  3400.),
    "joint3":       ("angular", 1600., 51.,  3400.),
    "joint4":       ("angular", 1600., 51.,  3400.),
    "joint5":       ("angular", 1600., 51.,  3400.),
    "joint6":       ("angular", 1600., 32.,  3400.),
    "finger_joint1":("linear",   800., 40.,   200.),
    "finger_joint2":("linear",   800., 40.,   200.),
}.items():
    jp = stage.GetPrimAtPath(f"{ROBOT}/Physics/{jname}")
    if not jp.IsValid(): continue
    drv = _UP.DriveAPI.Get(jp, axis) or _UP.DriveAPI.Apply(jp, axis)
    drv.CreateStiffnessAttr().Set(ke)
    drv.CreateDampingAttr().Set(kd)
    drv.CreateMaxForceAttr().Set(fmax)
    drv.CreateTargetPositionAttr().Set(0.0)

# joint body 중복 제거
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith(ROBOT): continue
    for rel in ("physics:body0", "physics:body1"):
        r = prim.GetRelationship(rel)
        if r and len(r.GetTargets()) > 1:
            r.SetTargets([r.GetTargets()[0]])

# zero mass 링크 최소 질량 설정
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith(ROBOT): continue
    if prim.HasAPI(UsdPhysics.MassAPI):
        mass_api = UsdPhysics.MassAPI(prim)
        mass_attr = mass_api.GetMassAttr()
        if mass_attr and mass_attr.Get() is not None and mass_attr.Get() <= 0:
            mass_attr.Set(0.001)

omni.timeline.get_timeline_interface().play()
print("[INFO] Simulation started (Newton only, no ROS2)", flush=True)

_step = 0
while simulation_app.is_running():
    simulation_app.update()
    _step += 1
    if _step % 100 == 0:
        print(f"[TICK] step={_step}", flush=True)

simulation_app.close()
