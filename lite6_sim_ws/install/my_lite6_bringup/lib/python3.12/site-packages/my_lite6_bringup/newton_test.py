"""Isaac Sim 6.0.0 — Newton + Lite6 + ROS2 joint control"""

import argparse
import os
import signal
import threading

import numpy as np

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.experience = "/home/v/IsaacSim/_build/linux-aarch64/release/apps/isaacsim.exp.full.newton.kit"
args_cli.visualizer = ["kit"]
simulation_app = AppLauncher(args_cli).app

_stop = False
def _on_sigint(sig, frame):
    global _stop
    _stop = True
signal.signal(signal.SIGINT, _on_sigint)

import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from pxr import Gf, UsdPhysics

USD_LITE6 = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "assets/lite6_with_gripper_d435i_v2_abs/lite6_with_gripper_d435i_v2_abs.usda",
)

# ── ROS2 ──────────────────────────────────────────────────────────────────────
import rclpy
from sensor_msgs.msg import JointState

if not rclpy.ok():
    rclpy.init()

_ros_node = rclpy.create_node("lite6_newton_bridge")

# 관절 이름 (MjcActuator 순서와 동일)
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
               "drive_joint", "drive_joint2"]

_lock = threading.Lock()
_joint_targets = np.zeros(len(JOINT_NAMES), dtype=np.float32)

def _on_joint_cmd(msg: JointState):
    with _lock:
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                _joint_targets[JOINT_NAMES.index(name)] = float(pos)

_sub_cmd = _ros_node.create_subscription(
    JointState, "/joint_command", _on_joint_cmd, 10
)
_pub_state = _ros_node.create_publisher(JointState, "/joint_states_newton", 10)

def _ros_spin():
    try:
        rclpy.spin(_ros_node)
    except Exception:
        pass

threading.Thread(target=_ros_spin, daemon=True).start()

# ── Newton 설정 ───────────────────────────────────────────────────────────────
mgr = omni.kit.app.get_app().get_extension_manager()
for ext in ("omni.usd.schema.mujoco", "omni.usd.schema.newton",
            "isaacsim.physics.newton", "isaacsim.physics.newton.tensors"):
    mgr.set_extension_enabled_immediate(ext, True)

from isaacsim.physics.newton import acquire_stage
ns = acquire_stage()
if ns:
    ns.cfg.collapse_fixed_joints = True

# ── 스테이지 구성 ─────────────────────────────────────────────────────────────
omni.usd.get_context().new_stage()
simulation_app.update()
stage = omni.usd.get_context().get_stage()

omni.kit.commands.execute("AddPhysicsSceneCommand", stage=stage, path="/World/PhysicsScene")
omni.kit.commands.execute("AddGroundPlaneCommand", stage=stage,
    planePath="/World/GroundPlane", axis="Z", size=1500.0,
    position=Gf.Vec3f(0, 0, 0), color=Gf.Vec3f(0.5, 0.5, 0.5))

# Lite6 로봇 로드 (mujoco variant)
robot = stage.DefinePrim("/World/Robot", "Xform")
robot.GetReferences().AddReference(USD_LITE6)
robot.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("mujoco")
for _ in range(10):
    simulation_app.update()

# physics:body0 다중 타겟 수정
from pxr import Sdf
for prim in stage.Traverse():
    if "/World/Robot/Physics/" not in str(prim.GetPath()):
        continue
    rel = prim.GetRelationship("physics:body0")
    if rel.IsValid():
        targets = rel.GetTargets()
        if len(targets) > 1:
            rel.SetTargets([targets[0]])

# zero-mass body 최소 질량
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith("/World/Robot"):
        continue
    if prim.HasAPI(UsdPhysics.MassAPI):
        attr = UsdPhysics.MassAPI(prim).GetMassAttr()
        if attr and attr.Get() is not None and attr.Get() <= 0:
            attr.Set(0.001)

# MjcActuator gain 설정 (중력 버팀)
for i in range(1, 7):
    ap = stage.GetPrimAtPath(f"/World/Robot/Physics/joint{i}_actuator")
    if ap.IsValid():
        ap.GetAttribute("mjc:gainPrm").Set([1600.0] + [0.0]*9)
        ap.GetAttribute("mjc:biasPrm").Set([0.0, -80.0, -1600.0] + [0.0]*7)

simulation_app.update()

# ── 시뮬레이션 시작 ───────────────────────────────────────────────────────────
omni.timeline.get_timeline_interface().play()
print("[INFO] Newton + Lite6 + ROS2 시작", flush=True)
print("[INFO]   subscribe: /joint_command  (sensor_msgs/JointState)", flush=True)
print("[INFO]   publish:   /joint_states_newton  (sensor_msgs/JointState)", flush=True)

from isaacsim.physics.newton.tensors import create_simulation_view as newton_create_sim_view

_step = 0
_pub_interval = 10  # ~6Hz publish

# ArticulationView 관련 (Newton 초기화 후 첫 프레임에 생성)
_art_view = None
_sim_view = None
_dof_names = []
_joint_to_dof = []   # JOINT_NAMES[i] → dof index (-1: 없음)
_num_dofs = 0
_indices_np = np.array([0], dtype=np.int32)

while simulation_app.is_running() and not _stop:
    ns_cur = acquire_stage()

    # Newton 초기화 완료 후 ArticulationView 한 번만 생성
    if ns_cur and ns_cur.initialized and _art_view is None:
        try:
            _sim_view = newton_create_sim_view(frontend_name="warp", newton_stage=ns_cur)
            _art_view = _sim_view.create_articulation_view("/World/Robot/Geometry/world/link_base")
            mt = _art_view.get_metatype(0)
            _dof_names = list(mt.dof_names) if mt and hasattr(mt, "dof_names") else []
            _num_dofs = _art_view.max_dofs
            print(f"[INFO] DOF count: {_num_dofs}", flush=True)
            print(f"[INFO] DOF names: {_dof_names}", flush=True)
            # JOINT_NAMES → dof index 매핑
            _dof_name_to_idx = {n: i for i, n in enumerate(_dof_names)}
            _joint_to_dof = [_dof_name_to_idx.get(jn, -1) for jn in JOINT_NAMES]
            print(f"[INFO] JOINT_NAMES→DOF mapping: {list(zip(JOINT_NAMES, _joint_to_dof))}", flush=True)
        except Exception as e:
            print(f"[WARN] ArticulationView 생성 실패: {e}", flush=True)

    # 관절 명령 적용 (ArticulationView → CTRL_DIRECT sync 자동 처리)
    if _art_view is not None and _num_dofs > 0:
        targets = np.zeros((1, _num_dofs), dtype=np.float32)
        with _lock:
            tgt_copy = _joint_targets.copy()
        if _joint_to_dof:
            for jn_i, dof_i in enumerate(_joint_to_dof):
                if dof_i >= 0:
                    targets[0, dof_i] = tgt_copy[jn_i]
        else:
            # 매핑 없으면 순서대로
            n = min(len(tgt_copy), _num_dofs)
            targets[0, :n] = tgt_copy[:n]
        try:
            _art_view.set_dof_position_targets(targets, _indices_np)
        except Exception as e:
            print(f"[WARN] set_dof_position_targets: {e}", flush=True)

    simulation_app.update()
    _step += 1

    # 관절 상태 퍼블리시
    if _step % _pub_interval == 0 and _art_view is not None:
        try:
            q_raw = _art_view.get_dof_positions(_indices_np)
            if q_raw is not None:
                q_np = q_raw.numpy() if hasattr(q_raw, "numpy") else np.array(q_raw)
                q = q_np[0] if q_np.ndim == 2 else q_np
                msg = JointState()
                msg.header.stamp = _ros_node.get_clock().now().to_msg()
                n = min(len(q), len(_dof_names) if _dof_names else len(JOINT_NAMES))
                msg.name = (_dof_names if _dof_names else JOINT_NAMES)[:n]
                msg.position = [float(v) for v in q[:n]]
                _pub_state.publish(msg)
        except Exception:
            pass

print("[INFO] 종료 중...", flush=True)
rclpy.shutdown()
simulation_app.close()
