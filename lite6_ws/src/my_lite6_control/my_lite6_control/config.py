"""Config for the Lite6 robot logic and the motion-test sim: YAML with `defaults:` includes, dotted
command-line overrides and validation (same format as sim-dataset-repo's binsim/config.py).

    cfg = load_config("config/default.yaml", ["planning.planner_id=RRTstar"])
    cfg.planning.planner_id  -> 'RRTstar'

Merge order (later wins): each file listed in `defaults:` -> the file itself -> command-line overrides.
Relative file paths (keys in PATH_KEYS) resolve against the YAML file they appear in; `~` is expanded.
Joint poses are radians and may be written with `pi` (`pi/2`, `-0.5*pi`); they are evaluated to floats on load.
Joint limits are written in degrees; joint_limits() returns them in radians.

Nothing here imports ROS, so the Isaac Sim script (sim/lite6_motion_sim.py) uses it too.
"""
from __future__ import annotations

import argparse
import ast
import copy
import math
import operator
import os
import sys
from dataclasses import dataclass
from typing import Any, Iterable

import yaml

PACKAGE = "my_lite6_control"
DEFAULT_CONFIG = "default.yaml"

# Keys whose string values are file paths (resolved relative to the YAML that defines them).
PATH_KEYS = {"robot_usd", "map_usd", "experience", "service_setup"}

# Allowed keys of one robot.joint_limits entry (catches typos such as `velocity_deg_s`).
JOINT_LIMIT_KEYS = {"position_deg", "max_velocity_deg_s", "max_acceleration_deg_s2"}


class Cfg(dict):
    """dict with attribute access (recursive). Plain dict underneath so it dumps to YAML/JSON."""

    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError as e:
            raise AttributeError(f"config has no key '{key}' (have: {list(self.keys())})") from e

    def __setattr__(self, key, value):
        self[key] = value

    def get_path(self, dotted: str, default=None):
        node: Any = self
        for k in dotted.split("."):
            if isinstance(node, dict) and k in node:
                node = node[k]
            elif isinstance(node, list) and k.isdigit() and int(k) < len(node):
                node = node[int(k)]
            else:
                return default
        return node


@dataclass(frozen=True)
class JointLimit:
    """One joint's limits in rad, rad/s, rad/s^2 (None = no limit beyond MoveIt's own)."""
    name: str
    lower: float
    upper: float
    max_velocity: float | None
    max_acceleration: float | None


def _wrap(obj):
    if isinstance(obj, dict):
        return Cfg({k: _wrap(v) for k, v in obj.items()})
    if isinstance(obj, list):
        return [_wrap(v) for v in obj]
    return obj


def to_plain(obj):
    if isinstance(obj, dict):
        return {k: to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_plain(v) for v in obj]
    return obj


def deep_merge(base: dict, over: dict) -> dict:
    """Recursive merge; dicts merge, everything else (incl. lists) is replaced."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def resolve_path(p: str, base_dir: str) -> str:
    # omniverse:// and http(s):// URLs resolve at runtime.
    if "://" in p:
        return p
    p = os.path.expandvars(os.path.expanduser(p))
    if os.path.isabs(p):
        return p
    return os.path.normpath(os.path.join(base_dir, p))


def _resolve_paths(node, base_dir: str):
    if isinstance(node, dict):
        for k, v in node.items():
            if k in PATH_KEYS and isinstance(v, str) and v:
                node[k] = resolve_path(v, base_dir)
            else:
                _resolve_paths(v, base_dir)
    elif isinstance(node, list):
        for v in node:
            _resolve_paths(v, base_dir)
    return node


def _load_yaml_tree(path: str, _stack=()) -> dict:
    path = os.path.abspath(path)
    if path in _stack:
        raise ValueError(f"circular `defaults:` include: {' -> '.join(_stack + (path,))}")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    base_dir = os.path.dirname(path)
    defaults = data.pop("defaults", []) or []
    merged: dict = {}
    for inc in defaults:
        inc_path = resolve_path(inc if inc.endswith((".yaml", ".yml")) else inc + ".yaml", base_dir)
        merged = deep_merge(merged, _load_yaml_tree(inc_path, _stack + (path,)))
    _resolve_paths(data, base_dir)
    return deep_merge(merged, data)


def apply_override(tree: dict, dotted: str, value: Any):
    keys = dotted.split(".")
    node: Any = tree
    for k in keys[:-1]:
        if isinstance(node, list):
            node = node[int(k)]
        else:
            node = node.setdefault(k, {})
    last = keys[-1]
    if isinstance(node, list):
        node[int(last)] = value
    else:
        node[last] = value


def parse_overrides(items: Iterable[str]) -> list[tuple[str, Any]]:
    out = []
    for it in items or []:
        if "=" not in it:
            raise ValueError(f"override '{it}' must look like key.sub=value")
        k, v = it.split("=", 1)
        out.append((k.strip(), yaml.safe_load(v)))
    return out


_BINOPS = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul, ast.Div: operator.truediv}


def _eval(node) -> float:
    if isinstance(node, ast.Constant) and type(node.value) in (int, float):
        return float(node.value)
    if isinstance(node, ast.Name) and node.id == "pi":
        return math.pi
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        v = _eval(node.operand)
        return -v if isinstance(node.op, ast.USub) else v
    if isinstance(node, ast.BinOp) and type(node.op) in _BINOPS:
        return _BINOPS[type(node.op)](_eval(node.left), _eval(node.right))
    raise ValueError("only numbers, pi, + - * / and parentheses are allowed")


def eval_number(v, name: str = "value") -> float:
    """A number, or a string expression of numbers and pi such as 'pi/2' or '-0.5*pi'."""
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v)
    if isinstance(v, str):
        try:
            return _eval(ast.parse(v.strip(), mode="eval").body)
        except (SyntaxError, ValueError, ZeroDivisionError) as e:
            raise ValueError(f"{name}: cannot evaluate {v!r} ({e})") from None
    raise ValueError(f"{name} must be a number or an expression such as pi/2, got {v!r}")


def _evaluate_poses(tree: dict, errs: list):
    """robot.poses (and a list-valued sim.initial_pose) -> floats in rad."""
    poses = (tree.get("robot") or {}).get("poses") or {}
    for name, values in list(poses.items()):
        if not isinstance(values, (list, tuple)):
            errs.append(f"robot.poses.{name} must be a list of joint values (rad), got {values!r}")
            continue
        try:
            poses[name] = [eval_number(v, f"robot.poses.{name}[{i}]") for i, v in enumerate(values)]
        except ValueError as e:
            errs.append(str(e))
    sim = tree.get("sim") or {}
    if isinstance(sim.get("initial_pose"), (list, tuple)):
        try:
            sim["initial_pose"] = [eval_number(v, f"sim.initial_pose[{i}]") for i, v in enumerate(sim["initial_pose"])]
        except ValueError as e:
            errs.append(str(e))


def _finish_paths(node):
    """Paths set on the command line: expand `~`, make relative ones absolute (relative to the cwd)."""
    if isinstance(node, dict):
        for k, v in node.items():
            if k in PATH_KEYS and isinstance(v, str) and v and "://" not in v:
                node[k] = os.path.abspath(os.path.expandvars(os.path.expanduser(v)))
            else:
                _finish_paths(v)
    elif isinstance(node, list):
        for v in node:
            _finish_paths(v)


def load_config(path: str, overrides: Iterable[str] = ()) -> Cfg:
    path = os.path.abspath(os.path.expanduser(path))
    overrides = list(overrides or [])
    tree = _load_yaml_tree(path)
    for k, v in parse_overrides(overrides):
        apply_override(tree, k, v)
    _finish_paths(tree)
    errs: list = []
    _evaluate_poses(tree, errs)
    tree["_meta"] = {"path": path, "overrides": overrides}
    cfg = _wrap(tree)
    validate(cfg, errs)
    return cfg


def default_config_path(name: str = DEFAULT_CONFIG) -> str:
    """config/<name> of this package: the source tree's copy when it can be found (edits apply without
    `colcon build`), else the installed share/ copy."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(os.path.dirname(here), "config", name)]  # running from the source tree
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(PACKAGE)
    except Exception:  # no ROS environment, or the package is not installed
        share = None
    if share:
        # <ws>/install/<pkg>/share/<pkg> -> look for <ws>/src/<pkg>/config/<name> in every parent
        d = share
        while os.path.dirname(d) != d:
            d = os.path.dirname(d)
            candidates.append(os.path.join(d, "src", PACKAGE, "config", name))
        candidates.append(os.path.join(share, "config", name))
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(f"{PACKAGE} config '{name}' not found (looked in: {', '.join(candidates)})")


def describe(cfg) -> str:
    """'config <path> + <overrides>' for startup logs."""
    meta = cfg.get("_meta") or {}
    text = f"config {meta.get('path', '?')}"
    if meta.get("overrides"):
        text += " + " + " ".join(meta["overrides"])
    return text


def dump_config(cfg) -> str:
    return yaml.safe_dump(to_plain({k: v for k, v in cfg.items() if k != "_meta"}), sort_keys=False)


def load_cli_config(default_name: str = DEFAULT_CONFIG, argv: list | None = None) -> Cfg:
    """Config for a `ros2 run` entry point: `--config <yaml>` and dotted overrides after the executable name
    (ROS arguments after --ros-args are left to rclpy). `--print-config` prints the merged result and exits."""
    argv = list(sys.argv if argv is None else argv)
    try:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(args=argv)
    except ImportError:
        pass
    parser = argparse.ArgumentParser(prog=os.path.basename(argv[0]) if argv else None,
                                     description=f"Config: {PACKAGE}/config/{default_name} unless --config is given.")
    parser.add_argument("--config", default=None, help=f"YAML config (default: {PACKAGE}/config/{default_name})")
    parser.add_argument("--print-config", action="store_true", help="print the merged config and exit")
    parser.add_argument("overrides", nargs="*", help="dotted overrides, e.g. planning.planner_id=RRTstar")
    args = parser.parse_args(argv[1:])
    cfg = load_config(args.config or default_config_path(default_name), args.overrides)
    if args.print_config:
        print(f"# {describe(cfg)}")
        print(dump_config(cfg), end="")
        sys.exit(0)
    return cfg


def joint_limits(cfg) -> list[JointLimit]:
    """robot.joint_limits in robot.joint_names order, in rad / rad/s / rad/s^2."""
    out = []
    for name in cfg.robot.joint_names:
        e = cfg.robot.joint_limits[name]
        lo, hi = e["position_deg"]
        vel = e.get("max_velocity_deg_s")
        acc = e.get("max_acceleration_deg_s2")
        out.append(JointLimit(name, math.radians(lo), math.radians(hi),
                              math.radians(vel) if vel else None, math.radians(acc) if acc else None))
    return out


def named_pose(cfg, pose) -> list[float]:
    """Joint values (rad) of a pose name in robot.poses, or of an explicit list."""
    if isinstance(pose, str):
        poses = cfg.robot.poses
        if pose not in poses:
            raise KeyError(f"robot.poses has no pose '{pose}' (have: {list(poses)})")
        pose = poses[pose]
    return [float(v) for v in pose]


def limit_violations(cfg, positions, names=None, tol: float = 1e-6) -> list[str]:
    """Joints outside robot.joint_limits, as readable strings (empty = all inside)."""
    limits = {lim.name: lim for lim in joint_limits(cfg)}
    out = []
    for name, q in zip(names if names is not None else cfg.robot.joint_names, positions):
        lim = limits.get(name)
        if lim is not None and not (lim.lower - tol <= q <= lim.upper + tol):
            out.append(f"{name}={q:+.4f} rad ({math.degrees(q):+.1f} deg) outside "
                       f"[{math.degrees(lim.lower):+.1f}, {math.degrees(lim.upper):+.1f}] deg")
    return out


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _check_range(errs: list, name: str, v):
    if not (isinstance(v, (list, tuple)) and len(v) == 2 and all(_is_num(x) for x in v) and v[0] < v[1]):
        errs.append(f"{name} must be [min, max] with min < max, got {v!r}")


def _check_positive(errs: list, name: str, v, allow_none: bool = False, integer: bool = False):
    if v is None and allow_none:
        return
    ok = (isinstance(v, int) and not isinstance(v, bool)) if integer else _is_num(v)
    if not (ok and v > 0):
        errs.append(f"{name} must be {'an integer' if integer else 'a number'} > 0"
                    f"{' or null' if allow_none else ''}, got {v!r}")


def validate(cfg: Cfg, errs: list | None = None):
    errs = list(errs or [])

    for k in ("robot.group", "robot.base_frame", "robot.eef_link", "robot.tcp_offset_m", "robot.joint_names",
              "robot.joint_limits", "robot.poses", "planning.planner_id", "planning.planning_time_s",
              "planning.planning_attempts", "planning.velocity_scaling", "planning.acceleration_scaling",
              "planning.goal_joint_tolerance_rad", "planning.enforce_joint_limits", "planning.ik.timeout_s",
              "safety.workspace_x_m", "safety.workspace_y_m", "safety.workspace_z_m", "safety.max_reach_m", "ros"):
        if cfg.get_path(k) is None:
            errs.append(f"missing required key: {k}")
    if errs:
        raise ValueError("invalid config:\n  - " + "\n  - ".join(errs))

    tcp = cfg.robot.tcp_offset_m
    if not (_is_num(tcp) or (isinstance(tcp, (list, tuple)) and len(tcp) == 3 and all(_is_num(x) for x in tcp))):
        errs.append(f"robot.tcp_offset_m must be a number (z) or [x, y, z], got {tcp!r}")

    names = cfg.robot.joint_names
    if not (isinstance(names, list) and names and all(isinstance(n, str) for n in names)):
        errs.append(f"robot.joint_names must be a non-empty list of joint names, got {names!r}")
        names = []
    limits = cfg.robot.joint_limits if isinstance(cfg.robot.joint_limits, dict) else {}
    for n in names:
        e = limits.get(n)
        if not isinstance(e, dict):
            errs.append(f"robot.joint_limits.{n} is missing (needs at least position_deg: [min, max])")
            continue
        unknown = set(e) - JOINT_LIMIT_KEYS
        if unknown:
            errs.append(f"robot.joint_limits.{n}: unknown key(s) {sorted(unknown)} (allowed: {sorted(JOINT_LIMIT_KEYS)})")
        _check_range(errs, f"robot.joint_limits.{n}.position_deg", e.get("position_deg"))
        _check_positive(errs, f"robot.joint_limits.{n}.max_velocity_deg_s", e.get("max_velocity_deg_s"), allow_none=True)
        _check_positive(errs, f"robot.joint_limits.{n}.max_acceleration_deg_s2", e.get("max_acceleration_deg_s2"),
                        allow_none=True)
    extra = set(limits) - set(names)
    if extra:
        errs.append(f"robot.joint_limits has joints that are not in robot.joint_names: {sorted(extra)}")

    limits_ok = not errs  # poses can only be checked against valid limits
    for pname, values in (cfg.robot.poses or {}).items():
        if not isinstance(values, list):
            continue  # already reported by _evaluate_poses
        if len(values) != len(names):
            errs.append(f"robot.poses.{pname} has {len(values)} values, robot.joint_names has {len(names)}")
        elif limits_ok:
            bad = limit_violations(cfg, values)
            if bad:
                errs.append(f"robot.poses.{pname} is outside robot.joint_limits: " + "; ".join(bad))

    p = cfg.planning
    for section in ("planning", "sequence"):
        for key in ("velocity_scaling", "acceleration_scaling"):
            v = cfg.get_path(f"{section}.{key}")
            if v is not None and not (_is_num(v) and 0 < v <= 1):
                errs.append(f"{section}.{key} must be in (0, 1], got {v!r}")
    _check_positive(errs, "planning.planning_time_s", p.planning_time_s)
    _check_positive(errs, "planning.planning_attempts", p.planning_attempts, integer=True)
    _check_positive(errs, "planning.goal_joint_tolerance_rad", p.goal_joint_tolerance_rad)
    _check_positive(errs, "planning.ik.timeout_s", p.ik.timeout_s)
    if not isinstance(p.enforce_joint_limits, bool):
        errs.append(f"planning.enforce_joint_limits must be true or false, got {p.enforce_joint_limits!r}")

    s = cfg.safety
    for key in ("workspace_x_m", "workspace_y_m", "workspace_z_m"):
        _check_range(errs, f"safety.{key}", s[key])
    _check_positive(errs, "safety.max_reach_m", s.max_reach_m)

    init = cfg.get_path("sim.initial_pose")
    if isinstance(init, str) and init not in (cfg.robot.poses or {}):
        errs.append(f"sim.initial_pose '{init}' is not in robot.poses ({list(cfg.robot.poses or {})})")
    elif isinstance(init, list):
        if len(init) != len(names):
            errs.append(f"sim.initial_pose has {len(init)} values, robot.joint_names has {len(names)}")
        elif limits_ok and limit_violations(cfg, init):
            errs.append("sim.initial_pose is outside robot.joint_limits: " + "; ".join(limit_violations(cfg, init)))

    if errs:
        raise ValueError("invalid config:\n  - " + "\n  - ".join(errs))
