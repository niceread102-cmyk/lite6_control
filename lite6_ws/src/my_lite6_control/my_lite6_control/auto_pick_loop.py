"""photo_neo_capture_test_hyeon.ipynb -> matrix_calculator_auto -> ee_6d_auto -> sequence_real 이어붙이기.

이 스크립트 하나만 실행하면 됨 — sequence_real.py를 별도로 띄울 필요 없음
(SequenceRealNode를 같은 프로세스 안에서 같이 실행함).

~/received/ 에 새 obj_pose_base_*.{json,txt}가 오면(노트북이 scp로 보냄),
xyzrpy를 그대로 matrix_calculator_auto.compute_grasp()에 넣어 그립 포즈를 계산하고,
ee_6d_auto.TargetPublisher로 /target_pose에 퍼블리시한다. sequence_real이 픽업을
끝내고 상태가 MOVE로 돌아올 때까지 기다렸다가 다음 detection을 보낸다.
"""
import glob
import os

import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_lite6_control.ee_6d_auto import TargetPublisher
from my_lite6_control.matrix_calculator_auto import compute_grasp
from my_lite6_control.sequence_real import SequenceRealNode

RECEIVED_DIR = os.path.expanduser('~/received')
POLL_SEC     = 3.0


def list_detection_files():
    files = glob.glob(os.path.join(RECEIVED_DIR, 'obj_pose_base_*.json')) + \
            glob.glob(os.path.join(RECEIVED_DIR, 'obj_pose_base_*.txt'))
    return sorted(files)


def parse_detections(path):
    """반환: [(x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, label), ...]"""
    out = []
    if path.endswith('.json'):
        import json
        with open(path) as f:
            data = json.load(f)
        for i, det in enumerate(data):
            x, y, z = det['xyz_mm']
            roll, pitch, yaw = det['rpy_deg']
            out.append((x / 1000.0, y / 1000.0, z / 1000.0, roll, pitch, yaw,
                        det.get('model_id', f'det{i}')))
    else:  # .txt: 한 줄에 "x y z roll pitch yaw" (m/deg)
        with open(path) as f:
            for i, line in enumerate(f):
                vals = line.split()
                if len(vals) != 6:
                    continue
                x, y, z, roll, pitch, yaw = map(float, vals)
                out.append((x, y, z, roll, pitch, yaw, f'{os.path.basename(path)}#{i}'))
    return out


class VisionWatcher(TargetPublisher):
    def __init__(self, seq_node: SequenceRealNode):
        super().__init__()  # ee_6d_auto.TargetPublisher.__init__ -> /target_pose 퍼블리셔
        self._seq_node = seq_node
        self._seen = set(list_detection_files())  # 시작 시점 기존 파일은 무시
        self._pending = []
        self.get_logger().info(
            f'[INIT] watching {RECEIVED_DIR} — {len(self._seen)} existing file(s) skipped, '
            f'polling every {POLL_SEC}s')
        self.create_timer(POLL_SEC, self._tick)

    def _tick(self):
        for path in list_detection_files():
            if path in self._seen:
                continue
            self._seen.add(path)
            dets = parse_detections(path)
            self.get_logger().info(f'[NEW] {os.path.basename(path)} -> {len(dets)} detection(s) queued')
            self._pending.extend(dets)

        if not self._pending:
            return

        if self._seq_node._busy:
            return  # 아직 이전 픽업(approach~home) 처리 중 -> 대기

        x, y, z, roll, pitch, yaw, label = self._pending.pop(0)
        grasp_pos, grasp_quat, grasp_rpy = compute_grasp(x, y, z, roll, pitch, yaw, verbose=False)

        self.get_logger().info(
            f'[SEND] {label}  grasp_pos=({grasp_pos[0]:.4f},{grasp_pos[1]:.4f},{grasp_pos[2]:.4f})  '
            f'grasp_rpy_deg=({grasp_rpy[0]:.1f},{grasp_rpy[1]:.1f},{grasp_rpy[2]:.1f})  '
            f'({len(self._pending)} remaining)')

        self.publish_target_quat(
            float(grasp_pos[0]), float(grasp_pos[1]), float(grasp_pos[2]),
            float(grasp_quat[0]), float(grasp_quat[1]), float(grasp_quat[2]), float(grasp_quat[3]))


def main(args=None):
    rclpy.init(args=args)
    seq_node = SequenceRealNode()
    watcher  = VisionWatcher(seq_node)

    executor = MultiThreadedExecutor()
    executor.add_node(seq_node)
    executor.add_node(watcher)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        watcher.destroy_node()
        seq_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
