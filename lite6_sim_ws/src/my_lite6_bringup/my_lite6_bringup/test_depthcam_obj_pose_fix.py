"""Isaac Sim 6.0.0 — Lite6 + Wrist Depth Camera"""
import argparse
import threading
import time
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
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

MAP_USD         = f"{HOME}/IsaacSim/assets/f2x_lab_v2.usd"
ROBOT_USD       = f"{HOME}/IsaacSim/assets/uf_lite6.usd"
ROBOT_PRIM_PATH = "/World/Robot/UF_ROBOT"

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
INIT_POS    = [1.0, 0.0, 1.57, 0.0, 1.57, 1.0]

GEAR_PATHS = [
    "/f2x_lab/f2x_gear_black_0",
    "/f2x_lab/f2x_gear_black_1",
    "/f2x_lab/f2x_gear_cyan",
    "/f2x_lab/f2x_gear_pink",
    "/f2x_lab/f2x_gear_yellow",
    "/f2x_lab/f2x_gear_blue",
    "/f2x_lab/f2x_gear_green",
]


class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge")
        self._targets = list(INIT_POS)
        self._current_pos = list(INIT_POS)
        self._lock = threading.Lock()
        self._idx_map = {n: i for i, n in enumerate(JOINT_NAMES)}
        self.create_subscription(
            JointTrajectory, "/lite6_traj_controller/joint_trajectory", self._cb, 10
        )
        # MoveIt (fake or real) sends planned trajectories through this action,
        # not the plain topic above — needed so RViz "Plan & Execute" reaches the sim.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/lite6_traj_controller/follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.depth_pub = self.create_publisher(Image, "/wrist_depth/depth", 1)
        self.rgb_pub   = self.create_publisher(Image, "/wrist_depth/rgb",   1)
        self._gripper_pos = 0.0089  # close 0.0 | open 0.0089
        self.create_subscription(Float32, "/gripper/command", self._gripper_cb, 10)

    def _gripper_cb(self, msg: Float32):
        with self._lock:
            self._gripper_pos = float(max(0.0, min(0.0089, msg.data)))

    def get_gripper_pos(self):
        with self._lock:
            return self._gripper_pos

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

    def set_current_pos(self, pos):
        with self._lock:
            self._current_pos = list(pos)

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.INVALID_GOAL)

        with self._lock:
            for name, pos in zip(traj.joint_names, traj.points[-1].positions):
                if name in self._idx_map:
                    self._targets[self._idx_map[name]] = float(pos)
            target = list(self._targets)

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = JOINT_NAMES
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            with self._lock:
                current = list(self._current_pos)
            feedback.actual.positions = current
            feedback.desired.positions = target
            goal_handle.publish_feedback(feedback)
            if all(abs(c - t) < 0.02 for c, t in zip(current, target)):
                break
            time.sleep(0.05)

        goal_handle.succeed()
        return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    depth_annotator = rgb_annotator = render_product = None

    try:
        omni.usd.get_context().open_stage(MAP_USD)
        for _ in range(5):
            simulation_app.update()
        stage = omni.usd.get_context().get_stage()

        UsdGeom.Xform.Define(stage, "/World")

        robot_prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
        robot_prim.GetReferences().AddReference(ROBOT_USD)
        robot_xform = UsdGeom.Xformable(robot_prim)
        robot_xform.ClearXformOpOrder()
        robot_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        robot_xform.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

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

        # phoxi 카메라의 /rgb, /depth, /camera_info는 v4 맵에 원래 있던 네이티브
        # Action Graph가 이미 정상적으로 퍼블리시한다. 여기서 또 만들면 같은
        # 토픽에 인코딩/타이밍이 다른 퍼블리셔가 두 개 겹쳐서 point_cloud_xyzrgb_node의
        # 동기화가 깨지므로, 파이썬에서는 손대지 않는다.

        # ── Joint Drives ─────────────────────────────────────────────
        print("[INFO] Setting up joint drives...", flush=True)
        joint_drives = []
        gripper_drives = []
        for idx, jname in enumerate(JOINT_NAMES):
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
                init_deg = math.degrees(INIT_POS[idx])
                drv.CreateTargetPositionAttr().Set(init_deg)
                state_api = PhysxSchema.JointStateAPI.Apply(jp, "angular")
                state_api.CreatePositionAttr().Set(init_deg)
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
                drv.CreateMaxForceAttr().Set(5.0)
                gripper_drives.append(drv)
            else:
                print(f"[WARN] Gripper joint '{gname}' not found!", flush=True)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(10):
            simulation_app.update()

        print("\n==================================================================")
        print("[INFO] Robot ready. Listening /lite6_traj_controller/joint_trajectory")
        print("==================================================================\n", flush=True)

        MAX_SPEED = 0.02 # manipulator speed
        current_pos = list(INIT_POS)

        last_pose_print = time.monotonic()

        frame_count = 0
        while simulation_app.is_running():
            simulation_app.update()

            targets_rad = node.get_targets()
            for i, (drv, target_rad) in enumerate(zip(joint_drives, targets_rad)):
                diff = target_rad - current_pos[i]
                step = max(-MAX_SPEED, min(MAX_SPEED, diff))
                current_pos[i] += step
                drv.GetTargetPositionAttr().Set(math.degrees(current_pos[i]))
            node.set_current_pos(current_pos)

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

            if time.monotonic() - last_pose_print >= 3.0:
                last_pose_print = time.monotonic()

                def xyzrpy_vals(prim):

                    tx = ty = tz = 0.0
                    rx = ry = rz = 0.0
                    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                        val = op.Get()
                        if val is None:
                            continue
                        op_type = op.GetOpType()
                        if op_type == UsdGeom.XformOp.TypeTranslate:
                            tx, ty, tz = val[0], val[1], val[2]
                        elif op_type == UsdGeom.XformOp.TypeRotateXYZ:
                            rx, ry, rz = val[0], val[1], val[2]
                        elif op_type == UsdGeom.XformOp.TypeOrient:
                            rx, ry, rz = Gf.Rotation(val).Decompose(
                                Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()
                            )
                    return (tx, ty, tz, rx, ry, rz)

                def fmt(vals):
                    x, y, z, r, p, yaw = vals
                    return f"{x:.4f} {y:.4f} {z:.4f} {r:.2f} {p:.2f} {yaw:.2f}"

                print("[Gear Pose]")
                for path in GEAR_PATHS:
                    outer = stage.GetPrimAtPath(path)
                    if not outer.IsValid():
                        print(f"  {path}: 프림 없음")
                        continue
                    print(f"  {path} xyzrpy {fmt(xyzrpy_vals(outer))}")

    finally:
        # Runs on normal exit, the early "link_eef not found" return, an
        # exception, or Ctrl+C (KeyboardInterrupt unwinds through this
        # finally too) — guarantees the GPU/render context and ROS node are
        # always released, so the next launch doesn't race a still-shutting-
        # down Kit process for the GPU.
        print("[INFO] Shutting down — releasing ROS node and Kit app...", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
