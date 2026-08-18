"""Isaac Sim 6.0.0 — Lite6 + Wrist Depth Camera"""
import argparse
import threading
import math
import os

from isaaclab.app import AppLauncher

HOME = os.path.expanduser("~")

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = f"{HOME}/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.kit"
args_cli.visualizer = ["kit"]

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.usd
import omni.timeline
import omni.replicator.core as rep
from pxr import UsdGeom, Gf, UsdPhysics, UsdShade

import numpy as np

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

MAP_USD         = f"{HOME}/IsaacSim/assets/f2x_test_lab.usd"
ROBOT_USD       = f"{HOME}/IsaacSim/assets/lite6_complete.usd"
ROBOT_PRIM_PATH = "/World/Robot/UF_ROBOT"

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
INIT_POS    = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]

UR5E_USD            = f"{HOME}/IsaacSim/assets/ur5e.usd"
UR5E_PRIM_PATH      = "/World/UR5E/ur5e"
UR5E_JOINT_NAMES    = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
UR5E_INIT_POS       = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]


class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge")
        self._targets = list(INIT_POS)
        self._lock = threading.Lock()
        self._idx_map = {n: i for i, n in enumerate(JOINT_NAMES)}
        self.create_subscription(
            JointTrajectory, "/lite6_traj_controller/joint_trajectory", self._cb, 10
        )

        # ── UR5e joint subscription ───────────────────────────────
        self._ur5e_targets = list(UR5E_INIT_POS)
        self._ur5e_idx_map = {n: i for i, n in enumerate(UR5E_JOINT_NAMES)}
        self.create_subscription(
            JointTrajectory, "/ur5e_traj_controller/joint_trajectory", self._ur5e_cb, 10
        )
        self.depth_pub = self.create_publisher(Image, "/wrist_depth/depth", 1)
        self.rgb_pub   = self.create_publisher(Image, "/wrist_depth/rgb",   1)
        self._gripper_pos = 0.0
        self.create_subscription(Float32, "/gripper/command", self._gripper_cb, 10)

    def _gripper_cb(self, msg: Float32):
        with self._lock:
            self._gripper_pos = float(max(0.0, min(0.0089, msg.data)))

    def get_gripper_pos(self):
        with self._lock:
            return self._gripper_pos

    # def _cb(self, msg: JointTrajectory):
    #     if not msg.points:
    #         return
    #     with self._lock:
    #         for name, pos in zip(msg.joint_names, msg.points[0].positions):
    #             if name in self._idx_map:
    #                 self._targets[self._idx_map[name]] = float(pos)

    def _cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        with self._lock:
            for name, pos in zip(msg.joint_names, msg.points[-1].positions):
                if name in self._idx_map:
                    self._targets[self._idx_map[name]] = float(pos)

    def get_targets(self):
        with self._lock:
            return list(self._targets)

    def _ur5e_cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        with self._lock:
            for name, pos in zip(msg.joint_names, msg.points[-1].positions):
                if name in self._ur5e_idx_map:
                    self._ur5e_targets[self._ur5e_idx_map[name]] = float(pos)

    def get_ur5e_targets(self):
        with self._lock:
            return list(self._ur5e_targets)

    def publish_depth(self, arr: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "wrist_camera"
        msg.height, msg.width = arr.shape[0], arr.shape[1]
        msg.encoding = "32FC1"
        msg.step = msg.width * 4
        msg.data = arr.astype(np.float32).tobytes()
        self.depth_pub.publish(msg)

    def publish_rgb(self, arr: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "wrist_camera"
        msg.height, msg.width = arr.shape[0], arr.shape[1]
        msg.encoding = "rgba8"
        msg.step = msg.width * 4
        msg.data = arr.astype(np.uint8).tobytes()
        self.rgb_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotBridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    omni.usd.get_context().open_stage(MAP_USD)
    for _ in range(5):
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")

    robot_prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    robot_prim.GetReferences().AddReference(ROBOT_USD)
    robot_xform = UsdGeom.Xformable(robot_prim)
    robot_xform.ClearXformOpOrder()
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(-4.0, 6.0, 0.0))
    robot_xform.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, -90.0))

    # ── UR5e Robot ───────────────────────────────────────────────
    ur5e_xform_prim = stage.DefinePrim("/World/UR5E", "Xform")
    ur5e_prim = stage.DefinePrim(UR5E_PRIM_PATH, "Xform")
    ur5e_prim.GetReferences().AddReference(UR5E_USD)
    ur5e_xform = UsdGeom.Xformable(ur5e_prim)
    ur5e_xform.ClearXformOpOrder()
    ur5e_xform.AddTranslateOp().Set(Gf.Vec3d(-4.0, 2.0, 0.0))
    ur5e_xform.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, -90.0))
    print(f"[INFO] UR5e loaded at {UR5E_PRIM_PATH}", flush=True)

    # ── UR5e Joint Drives ────────────────────────────────────────
    print("[INFO] Setting up UR5e joint drives...", flush=True)
    ur5e_joint_drives = []
    for jname in UR5E_JOINT_NAMES:
        jp = None
        for prim in stage.Traverse():
            if prim.GetName() == jname and prim.IsA(UsdPhysics.Joint):
                jp = prim
                break
        if jp is not None:
            drv = UsdPhysics.DriveAPI.Apply(jp, "angular")
            drv.CreateStiffnessAttr().Set(1600.0)
            drv.CreateDampingAttr().Set(80.0)
            drv.CreateMaxForceAttr().Set(3400.0)
            ur5e_joint_drives.append(drv)
            print(f"[INFO] UR5e drive set: {jname}", flush=True)
        else:
            print(f"[WARN] UR5e joint '{jname}' not found!", flush=True)

    # ── Wrist Depth Camera ────────────────────────────────────────
    # Dynamically find link_eef in stage (path varies across USD versions)
    link_eef_path = None
    for prim in stage.Traverse():
        if prim.GetName() == "link_eef":
            link_eef_path = str(prim.GetPath())
            break
    if link_eef_path is None:
        print("[ERROR] link_eef not found in stage!", flush=True)
        return
    print(f"[INFO] link_eef found at: {link_eef_path}", flush=True)

    WRIST_CAM_PATH = f"{link_eef_path}/WristDepthCamera"

    # Mount position from URDF camera_link_joint: xyz=(0.06746, -0.0175, 0.0237)
    # Rotation (180, 0, 90): corrects both vertical and horizontal flip
    cam = UsdGeom.Camera.Define(stage, WRIST_CAM_PATH)
    cam.CreateFocalLengthAttr(1.93)
    cam.CreateHorizontalApertureAttr(3.8)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0)) 
    cam_xform = UsdGeom.Xformable(cam)
    cam_xform.ClearXformOpOrder()
    cam_xform.AddTranslateOp().Set(Gf.Vec3f(-0.06746, -0.0175, 0.0237))
    cam_xform.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 90.0))

    render_product = rep.create.render_product(WRIST_CAM_PATH, (640, 480))
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_annotator.attach(render_product)
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(render_product)
    print(f"[INFO] Wrist depth camera: {WRIST_CAM_PATH}", flush=True)

    # ── Joint Drives ─────────────────────────────────────────────
    print("[INFO] Setting up joint drives...", flush=True)
    joint_drives = []
    gripper_drives = []
    for jname in JOINT_NAMES:
        jp = stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/{jname}")
        if not jp.IsValid():
            for prim in stage.Traverse():
                if prim.GetName() == jname and prim.IsA(UsdPhysics.Joint):
                    jp = prim
                    break
        if jp.IsValid():
            drv = UsdPhysics.DriveAPI(jp, "angular")
            if not drv:
                drv = UsdPhysics.DriveAPI.Apply(jp, "angular")
            drv.CreateStiffnessAttr().Set(1600.0)
            drv.CreateDampingAttr().Set(80.0)
            drv.CreateMaxForceAttr().Set(3400.0)
            joint_drives.append(drv)
        else:
            print(f"[WARN] Joint '{jname}' not found!", flush=True)

    for gname in ["finger_joint1", "finger_joint2"]:
        gp = None
        for prim in stage.Traverse():
            if prim.GetName() == gname:
                print(f"[INFO] Found '{gname}' at {prim.GetPath()} type={prim.GetTypeName()}", flush=True)
                gp = prim
                break
        if gp is not None:
            drv = UsdPhysics.DriveAPI.Apply(gp, "linear")
            drv.CreateStiffnessAttr().Set(1e6)
            drv.CreateDampingAttr().Set(1e4)
            drv.CreateMaxForceAttr().Set(1e4)
            gripper_drives.append(drv)
        else:
            print(f"[WARN] Gripper joint '{gname}' not found!", flush=True)

    # Friction on all prims inside uflite_finger1 / uflite_finger2 (collision meshes included)
    grip_mat = UsdShade.Material.Define(stage, "/World/GripperFriction")
    grip_phys = UsdPhysics.MaterialAPI.Apply(grip_mat.GetPrim())
    grip_phys.CreateStaticFrictionAttr(100.0)
    grip_phys.CreateDynamicFrictionAttr(100.0)
    grip_phys.CreateRestitutionAttr(0.0)

    finger_paths = set()
    for prim in stage.Traverse():
        if prim.GetName() in ("uflite_finger1", "uflite_finger2"):
            finger_paths.add(str(prim.GetPath()))

    cnt = 0
    for prim in stage.Traverse():
        ppath = str(prim.GetPath())
        if any(ppath.startswith(fp) for fp in finger_paths):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                grip_mat, UsdShade.Tokens.strongerThanDescendants, "physics"
            )
            cnt += 1
    print(f"[INFO] GripperFriction(100) applied to {cnt} finger prim(s)", flush=True)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        simulation_app.update()

    print("\n==================================================================")
    print("[INFO] Robot ready. Listening /lite6_traj_controller/joint_trajectory")
    print("==================================================================\n", flush=True)

    MAX_SPEED = 0.01
    current_pos = list(INIT_POS)
    ur5e_current_pos = list(UR5E_INIT_POS)

    frame_count = 0
    while simulation_app.is_running():
        simulation_app.update()

        targets_rad = node.get_targets()
        for i, (drv, target_rad) in enumerate(zip(joint_drives, targets_rad)):
            diff = target_rad - current_pos[i]
            step = max(-MAX_SPEED, min(MAX_SPEED, diff))
            current_pos[i] += step
            drv.GetTargetPositionAttr().Set(math.degrees(current_pos[i]))

        # ── UR5e joint control ────────────────────────────────────
        ur5e_targets_rad = node.get_ur5e_targets()
        for i, (drv, target_rad) in enumerate(zip(ur5e_joint_drives, ur5e_targets_rad)):
            diff = target_rad - ur5e_current_pos[i]
            step = max(-MAX_SPEED, min(MAX_SPEED, diff))
            ur5e_current_pos[i] += step
            drv.GetTargetPositionAttr().Set(math.degrees(ur5e_current_pos[i]))

        gripper_pos = node.get_gripper_pos()
        for drv in gripper_drives:
            drv.GetTargetPositionAttr().Set(gripper_pos)

        frame_count += 1

        depth = depth_annotator.get_data()
        rgb   = rgb_annotator.get_data()

        if depth is not None and depth.size > 0:
            node.publish_depth(depth)

        if rgb is not None and rgb.size > 0:
            node.publish_rgb(rgb)
            
        if frame_count % 300 == 0 and depth is not None and depth.size > 0:
            valid = depth[(depth > 0.01) & (depth < 9.9)]
            cy, cx = depth.shape[0] // 2, depth.shape[1] // 2
            print(
                f"[Depth] center={depth[cy, cx]:.3f}m  "
                f"min={valid.min() if valid.size > 0 else float('nan'):.3f}m  "
                f"max={valid.max() if valid.size > 0 else float('nan'):.3f}m",
                flush=True,
            )


    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
