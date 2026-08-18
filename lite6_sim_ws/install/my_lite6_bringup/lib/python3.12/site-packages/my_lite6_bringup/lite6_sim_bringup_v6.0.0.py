import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "/home/v/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.kit"
args_cli.visualizer = ["kit"]

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.graph.core as og
import usdrt.Sdf
from pxr import UsdGeom, Sdf, Gf, Usd, PhysxSchema, UsdPhysics

_PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
usd_lite6 = os.path.join(
    _PACKAGE_DIR,
    "assets/lite6_with_gripper_d435i_v2_abs/lite6_with_gripper_d435i_v2_abs.usda"
)

ISAAC_ASSETS_DIR = "/home/v/IsaacSim/assets"

ROBOT_PRIM_PATH  = "/World/Robot"
ARTIC_PRIM_PATH  = "/World/Robot/Geometry/world/link_base"  # ArticulationRootAPI location

EE_PRIM_SUBPATH  = "/Geometry/world/link_base/link1/link2/link3/link4/link5/link6"


def setup_all_ros2_bridges(robot_prim_path: str, camera_render_product_path: str):
    mgr = omni.kit.app.get_app().get_extension_manager()
    for ext in ("isaacsim.ros2.bridge", "isaacsim.core.nodes", "omni.graph.action"):
        mgr.set_extension_enabled_immediate(ext, True)
    omni.kit.app.get_app().update()

    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ROS2_Bridge", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick",                                      "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime",                                 "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Context",                                     "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishClock",                                "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("PublishJointState",                           "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("SubscribeJointState",                         "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController",                      "isaacsim.core.nodes.IsaacArticulationController"),
                    ("CameraHelper",                                "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("PublishTF",                                   "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PubEE",                                       "isaacsim.ros2.bridge.ROS2Publisher"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick",                         "PublishClock.inputs:execIn"),
                    ("OnTick.outputs:tick",                         "PublishJointState.inputs:execIn"),
                    ("OnTick.outputs:tick",                         "SubscribeJointState.inputs:execIn"),
                    ("OnTick.outputs:tick",                         "ArticulationController.inputs:execIn"),
                    ("OnTick.outputs:tick",                         "CameraHelper.inputs:execIn"),
                    ("OnTick.outputs:tick",                         "PublishTF.inputs:execIn"),
                    ("OnTick.outputs:tick",                         "PubEE.inputs:execIn"),
                    ("Context.outputs:context",                     "PublishClock.inputs:context"),
                    ("Context.outputs:context",                     "PublishJointState.inputs:context"),
                    ("Context.outputs:context",                     "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context",                     "CameraHelper.inputs:context"),
                    ("Context.outputs:context",                     "PublishTF.inputs:context"),
                    ("Context.outputs:context",                     "PubEE.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime",          "PublishClock.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime",          "PublishJointState.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime",          "PublishTF.inputs:timeStamp"),
                    ("SubscribeJointState.outputs:jointNames",      "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ],
                keys.SET_VALUES: [
                    ("PublishJointState.inputs:targetPrim",         [usdrt.Sdf.Path(ARTIC_PRIM_PATH)]),
                    ("ArticulationController.inputs:robotPath",     ARTIC_PRIM_PATH),
                    ("PublishJointState.inputs:topicName",          "joint_states"),
                    ("SubscribeJointState.inputs:topicName",        "joint_commands"),
                    ("CameraHelper.inputs:renderProductPath",       camera_render_product_path),
                    ("CameraHelper.inputs:topicName",               "wrist_camera/image_raw"),
                    ("PublishTF.inputs:targetPrims",                [usdrt.Sdf.Path(robot_prim_path)]),
                    ("PubEE.inputs:topicName",                      "ee_pose"),
                    ("PubEE.inputs:messagePackage",                 "geometry_msgs"),
                    ("PubEE.inputs:messageName",                    "Pose"),
                    
                    ("PublishJointState.inputs:topicName",   "isaac_sim/joint_states"),
                    ("SubscribeJointState.inputs:topicName", "joint_states"),
                ],
            },
        )
        print("[INFO] ROS2 Bridge setup complete.", flush=True)
    except Exception as e:
        print(f"[ERROR] Bridge setup failed: {e}", flush=True)


def main():
    stage = omni.usd.get_context().get_stage()

    print("[DEBUG] 1. Setting up scene...", flush=True)
    
    # 1. PhysicsScene 추가 (기본 PhysX 설정)
    omni.kit.commands.execute("AddPhysicsSceneCommand",
        stage=stage, path="/World/PhysicsScene")

    omni.kit.commands.execute("AddGroundPlaneCommand",
        stage=stage, planePath="/World/GroundPlane",
        axis="Z", size=1500.0,
        position=Gf.Vec3f(0, 0, 0), color=Gf.Vec3f(0.2, 0.2, 0.2))

    UsdGeom.Xform.Define(stage, "/World/Light")
    light = stage.DefinePrim("/World/Light/DomeLight", "DomeLight")
    light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

    # (name, usd, position, orient_xyzw, scale)
    _gear_usd = os.path.join(ISAAC_ASSETS_DIR, "steel_gear.usd")
    _gear_scale = Gf.Vec3d(0.06, 0.06, 0.06)
    objects = [
        ("Box",    os.path.join(ISAAC_ASSETS_DIR, "box.usd"),    Gf.Vec3d( 0.40,  0.0, 0.00), Gf.Quatd(1, 0, 0, 0), Gf.Vec3d(1.0, 1.0, 1.0)),
        ("Aruco0", os.path.join(ISAAC_ASSETS_DIR, "test_1.usd"), Gf.Vec3d( 0.40,  0.1, 0.01), Gf.Quatd(0.7071068, 0, 0, -0.7071068), Gf.Vec3d(0.3, 0.3, 0.3)),
        ("Aruco1", os.path.join(ISAAC_ASSETS_DIR, "test_2.usd"), Gf.Vec3d( 0.40, -0.1, 0.01), Gf.Quatd(0.7071068, 0, 0, -0.7071068), Gf.Vec3d(0.3, 0.3, 0.3)),
    ]
    # 기어 20개: 박스 위 같은 XY, Z만 0.15m씩 쌓아서 순차 낙하
    for i in range(20):
        objects.append((
            f"Gear{i}",
            _gear_usd,
            Gf.Vec3d(0.4, 0.0, 0.5 + i * 0.25),
            Gf.Quatd(1, 0, 0, 0),
            _gear_scale,
        ))
    for name, usd_path, pos, orient, scale in objects:
        p = stage.DefinePrim(f"/World/{name}", "Xform")
        p.GetReferences().AddReference(usd_path)
        xf = UsdGeom.Xformable(p)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(pos)
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(orient)
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(scale)

    # 박스·마커 바닥 고정 (kinematic = gravity/충돌에 안 밀림)
    for static_name in ("Box", "Aruco0", "Aruco1"):
        sp = stage.GetPrimAtPath(f"/World/{static_name}")
        if not sp.IsValid():
            continue
        # RigidBodyAPI 없으면 추가 후 kinematic 설정
        rb = UsdPhysics.RigidBodyAPI.Apply(sp)
        rb.CreateKinematicEnabledAttr().Set(True)

    print("[INFO] Spawned objects: Box, Gear, Aruco0, Aruco1", flush=True)

    print("[DEBUG] 2. Loading robot USD (PhysX physics variant)...", flush=True)
    robot_prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    robot_prim.GetReferences().AddReference(usd_lite6)
    robot_prim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physics")

    for _ in range(20):
        simulation_app.update()

    try:
        import omni.kit.viewport.utility as vp_util
        vp = vp_util.get_active_viewport()
        if vp:
            vp.camera_state.set_position_world(Gf.Vec3d(2.0, -1.5, 2.0), True)
            vp.camera_state.set_target_world(Gf.Vec3d(0.3,  0.0, 0.0),  True)
    except Exception as e:
        print(f"[WARN] Viewport camera setup failed: {e}", flush=True)

    print("[DEBUG] 2b. Fixing articulation base and joint paths...", flush=True)

    link_base_prim = stage.GetPrimAtPath(ARTIC_PRIM_PATH)
    if link_base_prim.IsValid():
        # PhysX 방식의 Articulation Root 고정
        link_base_prim.CreateAttribute("physxArticulation:fixBase", Sdf.ValueTypeNames.Bool, False).Set(True)
        print(f"[INFO] PhysX articulation base fixed → {ARTIC_PRIM_PATH}", flush=True)
    else:
        print(f"[WARN] link_base not found at {ARTIC_PRIM_PATH}", flush=True)

    fix_count = 0
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(ROBOT_PRIM_PATH):
            continue
            
        for rel_name in ("physics:body0", "physics:body1"):
            rel = prim.GetRelationship(rel_name)
            if not rel:
                continue
                
            targets = rel.GetTargets()
            if not targets:
                continue
                
            new_targets = []
            changed = False

            if len(targets) > 1:
                targets = [targets[0]]
                changed = True

            for t in targets:
                path_str = str(t)
                if path_str.startswith("/") and not path_str.startswith(ROBOT_PRIM_PATH):
                    parts = path_str.split("/")
                    if len(parts) > 2:
                        new_path = ROBOT_PRIM_PATH + "/" + "/".join(parts[2:])
                        new_targets.append(Sdf.Path(new_path))
                        changed = True
                        fix_count += 1
                    else:
                        new_targets.append(t)
                else:
                    new_targets.append(t)
                    
            if changed:
                rel.SetTargets(new_targets)
            
    print("[DEBUG] 3. Setting joint drives (stiffness/damping/maxForce)...", flush=True)
    joint_drives = {
        "joint1": (1600.0,  80.0, 3400.0),
        "joint2": (1600.0,  80.0, 3400.0),
        "joint3": (1600.0,  51.0, 3400.0),
        "joint4": (1600.0,  51.0, 3400.0),
        "joint5": (1600.0,  51.0, 3400.0),
        "joint6": (1600.0,  32.0, 3400.0),
        "finger_joint1": (800.0, 40.0, 200.0),
        "finger_joint2": (800.0, 40.0, 200.0),
    }
    for jname, (stiff, damp, max_force) in joint_drives.items():
        jp = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/Physics/{jname}")
        if jp.IsValid():
            drive_type = "linear" if "finger" in jname else "angular"
            drv = UsdPhysics.DriveAPI(jp, drive_type)
            if not drv:
                drv = UsdPhysics.DriveAPI.Apply(jp, drive_type)
            drv.CreateStiffnessAttr().Set(stiff)
            drv.CreateDampingAttr().Set(damp)
            drv.CreateMaxForceAttr().Set(max_force)
            drv.CreateTargetPositionAttr().Set(0.0)


    print("[DEBUG] 3b. Fixing negative mass on camera/auxiliary links...", flush=True)
    fixed_mass = 0
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(ROBOT_PRIM_PATH):
            continue
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            mass_attr = mass_api.GetMassAttr()
            if mass_attr and mass_attr.Get() is not None and mass_attr.Get() <= 0:
                mass_attr.Set(0.001)
                fixed_mass += 1
    if fixed_mass:
        print(f"[INFO] Fixed negative mass on {fixed_mass} links", flush=True)

    print("[DEBUG] 3c. Disabling collision on camera link tree...", flush=True)
    camera_link_base_path = (
        ROBOT_PRIM_PATH
        + "/Geometry/world/link_base/link1/link2/link3/link4/link5/link6"
          "/link_eef/camera_link"
    )
    disabled_col = 0
    for prim in stage.Traverse():
        prim_path_str = str(prim.GetPath())
        if prim_path_str.startswith(camera_link_base_path):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                disabled_col += 1
    print(f"[INFO] Disabled collision on {disabled_col} camera link prims", flush=True)

    print("[DEBUG] 4. Activating wrist camera...", flush=True)
    import omni.replicator.core as rep

    # camera_link 자체가 180° 뒤집혀 있으므로
    # optical_frame (orient=-0.5,0.5,-0.5,0.5 = ROS 표준) 아래에 Camera 정의
    camera_prim_path = (
        ROBOT_PRIM_PATH
        + "/Geometry/world/link_base/link1/link2/link3/link4/link5/link6"
          "/link_eef/camera_link/camera_color_frame/camera_color_optical_frame/Camera"
    )

    cam = UsdGeom.Camera.Define(stage, camera_prim_path)
    cam.CreateFocalLengthAttr(1.93)
    cam.CreateHorizontalApertureAttr(3.896)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 5.0))
    # ROS optical(Z=앞,Y=아래) → USD camera(Z=뒤,Y=위): X 180° = Quatd(0,1,0,0)
    # Y 180° 쓰면 X까지 뒤집혀 대각선으로 나옴
    UsdGeom.Xformable(cam.GetPrim()).AddOrientOp(
        UsdGeom.XformOp.PrecisionDouble
    ).Set(Gf.Quatd(0, 1, 0, 0))
    print(f"[INFO] Camera prim ready: {camera_prim_path}", flush=True)

    for _ in range(5):
        simulation_app.update()

    render_product = rep.create.render_product(camera_prim_path, (840, 480))
    rp_path = getattr(render_product, "path", None)
    print(f"[INFO] render_product.path = '{rp_path}'", flush=True)
    if not rp_path:
        print("[ERROR] render_product.path empty → CameraHelper will NOT publish!", flush=True)

    print("[DEBUG] 5. Setting up ROS2 bridges...", flush=True)
    setup_all_ros2_bridges(ROBOT_PRIM_PATH, render_product.path)

    p_x = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:position:x")
    p_y = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:position:y")
    p_z = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:position:z")
    q_w = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:w")
    q_x = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:x")
    q_y = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:y")
    q_z = og.Controller.attribute("/World/ROS2_Bridge/PubEE.inputs:orientation:z")

    print("[DEBUG] 6. PhysX physics setup complete.", flush=True)

    print("[DEBUG] 7. Starting simulation via timeline...", flush=True)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    for _ in range(5):
        simulation_app.update()

    print("\n==================================================================")
    print("[INFO] Isaac Sim Bring-up Running (Omni Core API) - PhysX Backend")
    print("[INFO] Topics: /joint_states  /joint_commands  /wrist_camera/image_raw")
    print("[INFO]         /ee_pose  /tf")
    print("==================================================================\n", flush=True)

    ee_prim_path = ROBOT_PRIM_PATH + EE_PRIM_SUBPATH

    while simulation_app.is_running():
        simulation_app.update()

        try:
            ee_prim = stage.GetPrimAtPath(ee_prim_path)
            if ee_prim.IsValid():
                xf = UsdGeom.Xformable(ee_prim)
                m  = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                t  = m.ExtractTranslation()
                r  = m.ExtractRotationQuat()
                im = r.GetImaginary()
                if p_x: og.Controller.set(p_x, float(t[0]))
                if p_y: og.Controller.set(p_y, float(t[1]))
                if p_z: og.Controller.set(p_z, float(t[2]))
                if q_w: og.Controller.set(q_w, float(r.GetReal()))
                if q_x: og.Controller.set(q_x, float(im[0]))
                if q_y: og.Controller.set(q_y, float(im[1]))
                if q_z: og.Controller.set(q_z, float(im[2]))
                
        except Exception:
            pass

if __name__ == "__main__":
    import traceback as _tb
    try:
        main()
    except KeyboardInterrupt:
        print("Bringup.py Exit.")
    except Exception as _e:
        print("\n" + "="*60)
        print("[FATAL PYTHON ERROR]")
        print("="*60)
        _tb.print_exc()
        print("="*60 + "\n")
        input(">>> Enter -> Exit... <<<")
    finally:
        simulation_app.close()