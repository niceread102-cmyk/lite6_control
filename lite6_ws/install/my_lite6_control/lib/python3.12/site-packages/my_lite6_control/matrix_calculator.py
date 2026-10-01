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

def cross_product(num):
    ee_z = num[:, 0]                     # obj X축 -> EE 접근축
    ee_z = ee_z / np.linalg.norm(ee_z)

    ee_y = -num[:, 2]                    # obj -Z축 -> EE 닫힘폭 축 (ee_z와 항상 직교)
    ee_y = ee_y / np.linalg.norm(ee_y)

    ee_x = np.cross(ee_y, ee_z)          # 오른손 좌표계(X=Y×Z) 완성

    R_ee = np.column_stack([ee_x, ee_y, ee_z])
    print(f"R_ee:\n{R_ee}")

    ee_rpy = Rotation.from_matrix(R_ee).as_euler('XYZ', degrees=True)  # intrinsic, euler_rotation_matrix와 동일 컨벤션
    print(f"ee rpy (deg) = {ee_rpy[0]:.1f} {ee_rpy[1]:.1f} {ee_rpy[2]:.1f}")

    return R_ee


def main():
    while True:
        user_input = input("input obj roll pitch yaw (or type 'q' to quit): ")

        if user_input.lower() == 'q':
            break

        try:
            target_roll, target_pitch, target_yaw = map(float, user_input.split())
        except ValueError:
            print("Invalid input. Enter exactly 3 numbers (roll pitch yaw), or 'q' to quit.")
            continue

        num = euler_rotation_matrix(np.radians(target_roll), np.radians(target_pitch), np.radians(target_yaw))

        print(f"rotation {num}")

        cross_product(num)

if __name__ == '__main__':
    main()