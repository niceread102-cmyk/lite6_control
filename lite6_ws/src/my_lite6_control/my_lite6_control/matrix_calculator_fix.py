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

DEFAULT_LOCAL_OFFSET = np.array([-0.005243, 0.007267, 0.011494])
SIDE_LOCAL_OFFSET_TOP = np.array([0.011240, -0.03650, 0.03325])
SIDE_LOCAL_OFFSET_DOWN = np.array([0.011240, -0.03650, -0.042]) 
FLIP_LOCAL_OFFSET_TOP = np.array([0.013, -0.03650, 0.0422])
FLIP_LOCAL_OFFSET_DOWN = np.array([0.013, -0.03650, -0.04978])

SIDE_THRESHOLD_DEG = 50.0

def classify_pose(num):
    # obj_x vs base_z (world up) is the sole classification axis:
    #   obj_x ~ -base_z (angle -> 180)  => (upright)
    #   obj_x ⊥ base_z  (angle ~ 90)    => (side)
    #   obj_x ~ +base_z (angle -> 0)    => (flipped)
    obj_x = num[:, 0]
    obj_x = obj_x / np.linalg.norm(obj_x)

    x_cos = np.dot(obj_x, BASE_Z)
    x_angle = np.degrees(np.arccos(np.clip(x_cos, -1.0, 1.0)))

    if x_angle > (180.0 - SIDE_THRESHOLD_DEG):
        pose = "upright"
    elif x_angle < SIDE_THRESHOLD_DEG:
        pose = "flipped"
    else:
        pose = "side"

    print(f"obj_x·base_z = {x_cos:.3f}  angle={x_angle:.1f}deg  -> {pose}")
    return pose

def _build_frame(ee_z, ee_y):
    ee_z = ee_z / np.linalg.norm(ee_z)
    ee_y = ee_y / np.linalg.norm(ee_y)
    ee_x = np.cross(ee_y, ee_z)
    return np.column_stack([ee_x, ee_y, ee_z])


def cross_product(num, obj_pos=None):
    pose = classify_pose(num)

    if pose == "upright":
        R_ee = _build_frame(num[:, 0], -num[:, 2])
        grasp_pos = obj_pos + num @ DEFAULT_LOCAL_OFFSET if obj_pos is not None else None
        note = None

    elif pose == "flipped":
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
            print(f"  candidate[top]  xyzrpy={full_top[0]:.4f} {full_top[1]:.4f} {full_top[2]:.4f} "
                  f"{full_top[3]:.2f} {full_top[4]:.2f} {full_top[5]:.2f}")
            print(f"  candidate[down] xyzrpy={full_down[0]:.4f} {full_down[1]:.4f} {full_down[2]:.4f} "
                  f"{full_down[3]:.2f} {full_down[4]:.2f} {full_down[5]:.2f}")

            grasp_pos = pos_top
        else:
            grasp_pos = None
        note = "flipped: two candidates (top/down) computed -> defaulting to 'top' (no selection rule yet)"

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
        note = f"side: using '{label}' rule (obj_z·base_z = {z_cos:.3f})"

    print(f"R_ee:\n{R_ee}")

    ee_rpy = Rotation.from_matrix(R_ee).as_euler('XYZ', degrees=True)  # intrinsic, same convention as euler_rotation_matrix
    list_pose.append(ee_rpy)
    print(f"ee rpy (deg) = {ee_rpy[0]:.1f} {ee_rpy[1]:.1f} {ee_rpy[2]:.1f}")

    if grasp_pos is not None:
        ee_rpy = np.append(grasp_pos, ee_rpy)
        print(f"{ee_rpy[0]:.4f} {ee_rpy[1]:.4f} {ee_rpy[2]:.4f} "
              f"{ee_rpy[3]:.2f} {ee_rpy[4]:.2f} {ee_rpy[5]:.2f}")
        print(f"grasp position = {grasp_pos[0]:.4f} {grasp_pos[1]:.4f} {grasp_pos[2]:.4f}")
    if note:
        print(note)

    return R_ee


def main():
    while True:
        user_input = input("input obj x y z roll pitch yaw (or type 'q' to quit): ")

        if user_input.lower() == 'q':
            break

        try:
            x, y, z, target_roll, target_pitch, target_yaw = map(float, user_input.split())
        except ValueError:
            print("Invalid input. Enter exactly 6 numbers (x y z roll pitch yaw), or 'q' to quit.")
            continue

        obj_pos = np.array([x, y, z])
        num = euler_rotation_matrix(np.radians(target_roll), np.radians(target_pitch), np.radians(target_yaw))

        print(f"rotation {num}")

        cross_product(num, obj_pos)

if __name__ == '__main__':
    main()