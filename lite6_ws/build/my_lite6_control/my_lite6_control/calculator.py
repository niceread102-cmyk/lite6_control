def main():
    print("XYZ Calculator  (A - B)")
    print("Enter values separated by spaces\n")

    raw = input("A  x y z : ").split()
    ax, ay, az = float(raw[0]), float(raw[1]), float(raw[2])

    raw = input("B  x y z : ").split()
    bx, by, bz = float(raw[0]), float(raw[1]), float(raw[2])

    print(f"\nResult : {ax+bx:.6f}  {ay+by:.6f}  {az+bz:.6f}")


if __name__ == '__main__':
    main()
