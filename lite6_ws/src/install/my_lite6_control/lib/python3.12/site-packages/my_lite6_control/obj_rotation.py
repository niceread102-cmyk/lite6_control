import numpy as np
from scipy.spatial.transform import Rotation


def main():
    print("Pivot-based Grasp Pose Calculator")

    raw = input("pivot  pos        x y z             : ").split()
    pivot_pos = np.array(list(map(float, raw[0:3])))

    raw = input("grasp  offset+rpy x y z roll pitch yaw : ").split()
    offset    = np.array(list(map(float, raw[0:3])))
    grasp_rpy = np.array(list(map(float, raw[3:6])))

    # R = [roll] @ [pitch] @ [yaw]
    R = Rotation.from_euler('XYZ', grasp_rpy, degrees=True).as_matrix()
    rotated_offset = R @ offset

    final_pos = pivot_pos + rotated_offset

    print(f"\nrotated offset  : {rotated_offset[0]:.4f} {rotated_offset[1]:.4f} {rotated_offset[2]:.4f}")
    print(f"final grasp pos : {final_pos[0]:.4f} {final_pos[1]:.4f} {final_pos[2]:.4f}")
    print(f"final grasp rpy : {grasp_rpy[0]:.3f} {grasp_rpy[1]:.3f} {grasp_rpy[2]:.3f}")


if __name__ == '__main__':
    main()
