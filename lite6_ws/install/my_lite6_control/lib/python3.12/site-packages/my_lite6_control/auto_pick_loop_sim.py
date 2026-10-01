"""photo_neo_capture_sim.ipynb -> matrix_calculator_auto -> ee_6d -> sequence_sim_action_fix, wired together.

Isaac Sim test rig — just run this one script. No need to launch
sequence_sim_action_fix.py separately (SequenceNode runs in the same process).

Picks one object at a time: whenever a new obj_pose_base_*.{json,txt} shows up
in /tmp, only its first detection is taken and sent through compute_grasp() ->
/target_pose; any other detections in that same file are ignored — once one
object is picked, the remaining objects' positions may no longer match the
capture (moved out of the way, or a neighbor now exposed), so every pick needs
a fresh capture.

Fully automated: `ros2 run my_lite6_control auto_pick_loop_sim` is all it
takes — main() spawns photo_neo_capture_sim_auto.py (the notebook's headless
counterpart) as a subprocess via the conda binpick interpreter
(set AUTO_PICK_SPAWN_CAPTURE=0 to skip that and just poll /tmp instead). The
READY_SIGNAL_PATH file only exists while the robot is idle (busy=False), and
gets deleted the moment a new detection is picked up (busy=True) — the
capture process waits for this file before capturing, so managing the signal
alone is enough to drive the next capture automatically. Capture count is set
via the AUTO_PICK_N_RUNS env var (empty = infinite, an integer = that many).
"""
import glob
import os
import subprocess

import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_lite6_control.config import load_cli_config
from my_lite6_control.ee_6d import TargetPublisher
from my_lite6_control.matrix_calculator_auto import compute_grasp
from my_lite6_control.sequence_sim_action_fix import SequenceNode

# photo_neo_capture_sim_auto.py is the headless counterpart of the notebook
# (photo_neo_capture_sim.ipynb) — SAM6D/FoundationPose are only installed in
# the conda binpick environment, so this file's (ROS2-side) Python can't just
# import them. Spawned as a separate process instead.
CAPTURE_SCRIPT_DIR    = '/home/v/baseline-sam6D-foundation-pose'
CAPTURE_SCRIPT        = 'photo_neo_capture_sim_auto.py'
CAPTURE_PYTHON        = '/home/v/.virtualenvs/binpick/bin/python'
# NOTE: this is a *venv*, not the conda env of the same name — the conda
# "binpick" env's open3d package is an empty leftover directory (an implicit
# namespace package with __file__=None and no submodules), which is what
# caused "AttributeError: module 'open3d' has no attribute 'geometry'" the
# first time this was pointed at the conda interpreter. This venv has the
# project's actual lightweight open3d shim plus CUDA torch, cv2, and rclpy —
# verified working.
# Set AUTO_PICK_SPAWN_CAPTURE=0 to skip spawning the capture process and just
# poll /tmp instead (e.g. if the notebook/script is already running
# separately). AUTO_PICK_N_RUNS is passed through to the capture script as-is.
SPAWN_CAPTURE = os.environ.get('AUTO_PICK_SPAWN_CAPTURE', '1') != '0'

# photo_neo_capture_sim.ipynb saves obj_pose_base_*.json to /tmp (same machine,
# no scp needed — see cell 15's json_path). Real-robot pipeline uses ~/received
# instead (auto_pick_loop.py) because that one *is* scp'd over from elsewhere.
RECEIVED_DIR = '/tmp'
POLL_SEC     = 3.0

# Signal file the notebook's run_auto_loop() polls for. Only exists while
# idle — the notebook captures once this file appears, and this script
# deletes it the moment it picks up that capture's detection file, putting
# the notebook back to sleep until the next cycle finishes.
READY_SIGNAL_PATH = '/tmp/auto_pick_ready.signal'


def _touch_ready():
    with open(READY_SIGNAL_PATH, 'w') as f:
        f.write('ready')


def _clear_ready():
    try:
        os.remove(READY_SIGNAL_PATH)
    except FileNotFoundError:
        pass


def list_detection_files():
    files = glob.glob(os.path.join(RECEIVED_DIR, 'obj_pose_base_*.json')) + \
            glob.glob(os.path.join(RECEIVED_DIR, 'obj_pose_base_*.txt'))
    return sorted(files)


def parse_detections(path):
    """Returns: [(x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, label), ...]"""
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
    else:  # .txt: one line per detection, "x y z roll pitch yaw" (m/deg)
        with open(path) as f:
            for i, line in enumerate(f):
                vals = line.split()
                if len(vals) != 6:
                    continue
                x, y, z, roll, pitch, yaw = map(float, vals)
                out.append((x, y, z, roll, pitch, yaw, f'{os.path.basename(path)}#{i}'))
    return out


class VisionWatcher(TargetPublisher):
    def __init__(self, seq_node: SequenceNode, capture_proc=None):
        super().__init__()  # ee_6d.TargetPublisher.__init__ -> /target_pose publisher
        self._seq_node = seq_node
        self._capture_proc = capture_proc  # subprocess.Popen handle, or None if not spawned
        self._seen = set(list_detection_files())  # ignore files that already existed at startup
        self._was_busy = False
        self.get_logger().info(
            f'[INIT] watching {RECEIVED_DIR} — {len(self._seen)} existing file(s) skipped, '
            f'polling every {POLL_SEC}s')
        self.create_timer(POLL_SEC, self._tick)

    def _tick(self):
        # If the capture process has exited (n_runs completed, gave up after
        # too many empty frames, or crashed), there's nothing left to feed
        # this loop — shut the ROS2 side down too instead of spinning forever.
        if self._capture_proc is not None and self._capture_proc.poll() is not None:
            code = self._capture_proc.returncode
            self.get_logger().info(f'[EXIT] capture process ended (exit code {code}) -> shutting down')
            rclpy.shutdown()
            return

        busy = self._seq_node._busy

        # Sync the ready-signal file's existence to the busy state every tick,
        # rather than toggling it on a single event — this way, if a
        # published target gets rejected by pose_is_safe and never actually
        # flips busy, the signal simply stays put and the system self-heals.
        if busy:
            _clear_ready()
        elif not os.path.exists(READY_SIGNAL_PATH):
            _touch_ready()

        if self._was_busy and not busy:
            self.get_logger().info('[READY] cycle finished -> notebook can capture again')
        self._was_busy = busy

        if busy:
            return  # still mid-pickup (approach~home) -> haven't looked at new files yet

        for path in list_detection_files():
            if path in self._seen:
                continue
            self._seen.add(path)
            dets = parse_detections(path)
            if not dets:
                self.get_logger().warn(f'[NEW] {os.path.basename(path)} -> 0 detections, skipping')
                continue

            if len(dets) > 1:
                self.get_logger().info(
                    f'[NEW] {os.path.basename(path)} -> {len(dets)} detection(s), using only the '
                    f'first — one pick per capture, the rest come from the next recapture')

            x, y, z, roll, pitch, yaw, label = dets[0]
            grasp_pos, grasp_quat, grasp_rpy = compute_grasp(x, y, z, roll, pitch, yaw, verbose=False)

            self.get_logger().info(
                f'[SEND] {label}  grasp_pos=({grasp_pos[0]:.4f},{grasp_pos[1]:.4f},{grasp_pos[2]:.4f})  '
                f'grasp_rpy_deg=({grasp_rpy[0]:.1f},{grasp_rpy[1]:.1f},{grasp_rpy[2]:.1f})')

            self.publish_target_quat(
                float(grasp_pos[0]), float(grasp_pos[1]), float(grasp_pos[2]),
                float(grasp_quat[0]), float(grasp_quat[1]), float(grasp_quat[2]), float(grasp_quat[3]))
            return  # send just the one this tick — any other new files wait for the next tick


def _spawn_capture_process():
    script_path = os.path.join(CAPTURE_SCRIPT_DIR, CAPTURE_SCRIPT)
    if not (SPAWN_CAPTURE and os.path.exists(CAPTURE_PYTHON) and os.path.exists(script_path)):
        return None
    return subprocess.Popen(
        [CAPTURE_PYTHON, CAPTURE_SCRIPT],
        cwd=CAPTURE_SCRIPT_DIR,
        env=os.environ,  # carries AUTO_PICK_N_RUNS through as-is, if set
    )


def main(args=None):
    cfg = load_cli_config()  # SequenceNode's config: --config <yaml> / dotted overrides after the executable name
    capture_proc = _spawn_capture_process()
    if capture_proc is not None:
        print(f'[auto_pick_loop_sim] spawned capture process pid={capture_proc.pid} '
              f'({CAPTURE_SCRIPT} via binpick env)')
    elif SPAWN_CAPTURE:
        print('[auto_pick_loop_sim] capture script/interpreter not found — '
              'run photo_neo_capture_sim.ipynb (or the .py) separately')

    rclpy.init(args=args)
    seq_node = SequenceNode(cfg)
    watcher  = VisionWatcher(seq_node, capture_proc)

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
        if rclpy.ok():  # _tick() may have already called shutdown() itself
            rclpy.shutdown()
        if capture_proc is not None and capture_proc.poll() is None:
            capture_proc.terminate()


if __name__ == '__main__':
    main()
