import numpy as np
from scipy.spatial.transform import Rotation

def euler_rotation_matrix(x, y, z):
    Rx = np.array([[1,         0,          0],
                   [0, np.cos(x), -np.sin(x)],
                   [0, np.sin(x),  np.cos(x)]])

    Ry = np.array([[ np.cos(y), 0, np.sin(y)],
                   [ 0,         1,         0],
                   [-np.sin(y), 0, np.cos(y)]])

    Rz = np.array([[np.cos(z), -np.sin(z), 0],
                   [np.sin(z),  np.cos(z), 0],
                   [0,         0,          1]])
        
    # return Rz @ Ry @ Rx # extrinsic
    return Rx @ Ry @ Rz   # intrinsic

list_pose = []
BASE_Z = np.array([0.0, 0.0, 1.0]) # Robot base Z

# DEFAULT_LOCAL_OFFSET = np.array([0.02, 0.0, 0.0])
# FLIP_LOCAL_OFFSET_TOP = np.array([0.03, -0.042, 0.04])
# FLIP_LOCAL_OFFSET_DOWN = np.array([0.03, -0.042, -0.06])
# SIDE_LOCAL_OFFSET_TOP = np.array([0.03, -0.042, 0.03])
# SIDE_LOCAL_OFFSET_DOWN = np.array([0.03, -0.042, -0.05])

# DEFAULT_LOCAL_OFFSET = np.array([0.00, 0.0, 0.01])
# FLIP_LOCAL_OFFSET_TOP = np.array([0.01, -0.035, 0.045])
# FLIP_LOCAL_OFFSET_DOWN = np.array([0.01, -0.035, -0.045])
# SIDE_LOCAL_OFFSET_TOP = np.array([0.02, -0.03, 0.029])
# SIDE_LOCAL_OFFSET_DOWN = np.array([0.02, -0.03, -0.029]) 

DEFAULT_LOCAL_OFFSET = np.array([-0.01, -0.002, 0.0135])
FLIP_LOCAL_OFFSET_TOP = np.array([0.015, -0.035, 0.037])
FLIP_LOCAL_OFFSET_DOWN = np.array([0.015, -0.035, -0.043])
SIDE_LOCAL_OFFSET_TOP = np.array([0.03, -0.035, 0.037])
SIDE_LOCAL_OFFSET_DOWN = np.array([0.03, -0.035, -0.043]) 
SIDE_THRESHOLD_DEG = 50.0

def classify_pose(num):
    obj_x = num[:, 0]
    obj_x = obj_x / np.linalg.norm(obj_x)
    obj_z = num[:, 2]
    obj_z = obj_z / np.linalg.norm(obj_z)

    z_cos = np.dot(obj_z, BASE_Z)
    z_angle = np.degrees(np.arccos(np.clip(z_cos, -1.0, 1.0)))

    if z_angle < SIDE_THRESHOLD_DEG or z_angle > (180.0 - SIDE_THRESHOLD_DEG):
        print(f"obj_z·base_z = {z_cos:.3f}  angle={z_angle:.1f}deg  -> 측면")
        return "측면"

    cos_sim = np.dot(obj_x, BASE_Z)
    angle = np.degrees(np.arccos(np.clip(cos_sim, -1.0, 1.0)))

    pose = "기본자세" if cos_sim < 0 else "뒤집어짐"
    print(f"obj_x·base_z = {cos_sim:.3f}  angle={angle:.1f}deg  -> {pose}")
    return pose

def _build_frame(ee_z, ee_y):
    ee_z = ee_z / np.linalg.norm(ee_z)
    ee_y = ee_y / np.linalg.norm(ee_y)
    ee_x = np.cross(ee_y, ee_z)
    return np.column_stack([ee_x, ee_y, ee_z])


def cross_product(num, obj_pos=None):
    pose = classify_pose(num)

    if pose == "기본자세":
        R_ee = _build_frame(num[:, 0], -num[:, 2])
        grasp_pos = obj_pos + num @ DEFAULT_LOCAL_OFFSET if obj_pos is not None else None
        note = None

    elif pose == "뒤집어짐":
        R_ee = _build_frame(-num[:, 0], -num[:, 1])
        R_top = R_ee
        Rz180 = np.array([[-1.0, 0.0, 0.0],
                           [ 0.0,-1.0, 0.0],
                           [ 0.0, 0.0, 1.0]])
        R_down = R_top @ Rz180

        rpy_top  = Rotation.from_matrix(R_top).as_euler('XYZ', degrees=True)
        rpy_down = Rotation.from_matrix(R_down).as_euler('XYZ', degrees=True)
        print(f"  candidate[top]  rpy={np.round(rpy_top, 1)}")
        print(f"  candidate[down] rpy={np.round(rpy_down, 1)}")

        if obj_pos is not None:
            pos_top  = obj_pos + num @ FLIP_LOCAL_OFFSET_TOP
            pos_down = obj_pos + num @ FLIP_LOCAL_OFFSET_DOWN
            print(f"  candidate[top]  pos={np.round(pos_top, 4)}")
            print(f"  candidate[down] pos={np.round(pos_down, 4)}")

            full_top  = np.append(pos_top, rpy_top)
            full_down = np.append(pos_down, rpy_down)
            print(f"  candidate[top]  xyzrpy={np.array2string(full_top, precision=3, suppress_small=True, floatmode='fixed')}")
            print(f"  candidate[down] xyzrpy={np.array2string(full_down, precision=3, suppress_small=True, floatmode='fixed')}")

            grasp_pos = pos_top
        else:
            grasp_pos = None
        note = "뒤집어짐 후보 두 개(top/down) 계산됨 -> 기본값 'top' 반환 (선택 기준 미정)"

    else:
        z_cos = np.dot(num[:, 2], BASE_Z)

        if z_cos >= 0:
            label = "top"
            R_ee = _build_frame(-num[:, 2], -num[:, 1])
            offset = SIDE_LOCAL_OFFSET_TOP
        else:
            label = "bottom"
            R_ee = _build_frame(num[:, 2], num[:, 1])
            offset = SIDE_LOCAL_OFFSET_DOWN

        grasp_pos = obj_pos + num @ offset if obj_pos is not None else None
        note = f"측면 '{label}' 규칙 사용 (obj_z·base_z = {z_cos:.3f})"

    print(f"R_ee:\n{R_ee}")

    ee_rpy = Rotation.from_matrix(R_ee).as_euler('XYZ', degrees=True)  # intrinsic, euler_rotation_matrix와 동일 컨벤션
    list_pose.append(ee_rpy)
    print(f"ee rpy (deg) = {ee_rpy[0]:.1f} {ee_rpy[1]:.1f} {ee_rpy[2]:.1f}")

    if grasp_pos is not None:
        ee_rpy = np.append(grasp_pos, ee_rpy)
        print(np.array2string(ee_rpy, precision=3, suppress_small=True, floatmode='fixed'))
        print(f"grasp position = {grasp_pos[0]:.4f} {grasp_pos[1]:.4f} {grasp_pos[2]:.4f}")
    if note:
        print(note)

    return R_ee, grasp_pos


def compute_grasp(x, y, z, roll, pitch, yaw, verbose=True):
    """비대화형 진입점 — obj pose(미터/도, intrinsic XYZ) -> grasp pose.

    파이프라인(카메라 인식 -> 이 함수 -> ee_6d.py -> sequence_real.py)에서
    input() 없이 바로 호출하기 위한 함수. matrix_calculator_fix.py의 대화형
    main() 루프와 동일한 계산을 그대로 재사용함.

    Returns:
        grasp_pos: np.ndarray shape (3,) — grasp 위치 (m)
        grasp_quat_xyzw: np.ndarray shape (4,) — [x,y,z,w]
        grasp_rpy_deg: np.ndarray shape (3,) — intrinsic XYZ RPY (deg)
    """
    obj_pos = np.array([x, y, z])
    num = euler_rotation_matrix(np.radians(roll), np.radians(pitch), np.radians(yaw))

    if not verbose:
        import contextlib, io
        with contextlib.redirect_stdout(io.StringIO()):
            R_ee, grasp_pos = cross_product(num, obj_pos)
    else:
        R_ee, grasp_pos = cross_product(num, obj_pos)

    grasp_quat = Rotation.from_matrix(R_ee).as_quat()  # [x,y,z,w]
    grasp_rpy = Rotation.from_matrix(R_ee).as_euler('XYZ', degrees=True)
    return grasp_pos, grasp_quat, grasp_rpy


if __name__ == '__main__':
    # 빠른 동작 확인용 — 실제 파이프라인에선 compute_grasp()를 직접 import해서 씀
    pos, quat, rpy = compute_grasp(0.2675, 0.1278, 0.0243, -45.36, -8.56, -15.20)
    print(f"\ngrasp_pos={pos}  grasp_quat(xyzw)={quat}  grasp_rpy(deg)={rpy}")