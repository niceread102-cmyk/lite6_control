import glob
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

RECEIVED_DIR = '/home/v/received'
POLL_PERIOD_SEC = 1.0

# Camera -> robot_base extrinsic: p_base = T_cam_base @ p_cam (m). Hand-eye calibration
# ~/Projects/xarm_lite6/cam2robot_pos_calibration/result/T_base_cam.yaml (touch/board_model, 2026-10-27 16:45, RMS 1.1 mm)
T_cam_base = np.array([
    [-0.00169203, -0.99945118,  0.03308287,  0.25562417],
    [-0.97936149,  0.00834258,  0.20194425, -0.25696641],
    [-0.20210942, -0.03205840, -0.97883811,  0.71477754],
    [ 0.0,         0.0,         0.0,         1.0],
])
R_cam_base = T_cam_base[:3, :3]
t_cam_base = T_cam_base[:3, 3]

# --- safety limits -----------------------------------------------------
SCORE_MIN     = 0.5     # reject low-confidence detections
WS_X          = (0.05, 0.45)   # robot-base-frame workspace bounds (m)
WS_Y          = (-0.45, 0.45)
WS_Z          = (-0.05, 0.60)
MAX_JUMP_M    = 0.30    # reject if target moves further than this from last accepted target
MIN_PUBLISH_INTERVAL_SEC = 3.0  # rate limit: don't command a new move more often than this
# -------------------------------------------------------------------------


def cam_to_base(p_cam_xyz):
    p_cam_h = np.append(np.asarray(p_cam_xyz, dtype=float), 1.0)
    p_base_h = T_cam_base @ p_cam_h
    return p_base_h[:3]


class Obj6DPublisher(Node):
    def __init__(self):
        super().__init__('obj_6d_publisher')
        self.pub = self.create_publisher(Pose, '/target_pose', 10)
        self._seen_files = set()
        self._last_pos = None
        self._last_publish_t = 0.0
        self.timer = self.create_timer(POLL_PERIOD_SEC, self._poll)
        self.get_logger().info(f'obj_6d_publisher watching {RECEIVED_DIR}')

    def _poll(self):
        for path in sorted(glob.glob(os.path.join(RECEIVED_DIR, 'obj_pose_*.json'))):
            if path in self._seen_files:
                continue
            self._seen_files.add(path)
            self._handle_file(path)

    def _handle_file(self, path):
        try:
            with open(path) as f:
                detections = json.load(f)
        except Exception as e:
            self.get_logger().error(f'[{path}] failed to read/parse: {e}')
            return

        if not detections:
            return

        best = max(detections, key=lambda d: d.get('score', 0.0))
        self._handle_detection(path, best)

    def _handle_detection(self, path, det):
        model_id = det.get('model_id', '?')
        score    = det.get('score', 0.0)
        xyz_mm   = det.get('xyz_mm')
        rpy_deg  = det.get('rpy_deg')

        if xyz_mm is None or rpy_deg is None:
            self.get_logger().error(f'[{path}] missing xyz_mm/rpy_deg, REJECTED')
            return

        xyz_mm = np.asarray(xyz_mm, dtype=float)
        rpy_deg = np.asarray(rpy_deg, dtype=float)

        if not (np.all(np.isfinite(xyz_mm)) and np.all(np.isfinite(rpy_deg))):
            self.get_logger().error(f'[{path}] [{model_id}] NaN/Inf in pose, REJECTED')
            return

        if score < SCORE_MIN:
            self.get_logger().warn(f'[{path}] [{model_id}] score {score:.3f} < {SCORE_MIN}, REJECTED')
            return

        now = time.monotonic()
        if now - self._last_publish_t < MIN_PUBLISH_INTERVAL_SEC:
            self.get_logger().warn(
                f'[{path}] [{model_id}] rate-limited '
                f'({now - self._last_publish_t:.1f}s < {MIN_PUBLISH_INTERVAL_SEC}s), REJECTED')
            return

        p_cam  = xyz_mm / 1000.0  # mm -> m
        p_base = cam_to_base(p_cam)

        if not (WS_X[0] <= p_base[0] <= WS_X[1] and
                WS_Y[0] <= p_base[1] <= WS_Y[1] and
                WS_Z[0] <= p_base[2] <= WS_Z[1]):
            self.get_logger().error(
                f'[{path}] [{model_id}] p_base={np.round(p_base, 3)} outside workspace '
                f'bounds x{WS_X} y{WS_Y} z{WS_Z}, REJECTED')
            return

        if self._last_pos is not None:
            jump = float(np.linalg.norm(p_base - self._last_pos))
            if jump > MAX_JUMP_M:
                self.get_logger().error(
                    f'[{path}] [{model_id}] jump {jump:.3f}m > {MAX_JUMP_M}m from last '
                    f'accepted target, REJECTED (possible bad detection)')
                return

        R_obj  = Rotation.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
        R_base = R_cam_base @ R_obj
        q      = Rotation.from_matrix(R_base).as_quat()  # [x, y, z, w]

        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = p_base.tolist()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q.tolist()

        self.pub.publish(msg)
        self._last_pos = p_base
        self._last_publish_t = now
        self.get_logger().info(
            f'[{path}] [{model_id}] score={score:.3f} published p_base={np.round(p_base, 3)}')


def main(args=None):
    rclpy.init(args=args)
    node = Obj6DPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
