#!/usr/bin/env python3
import argparse


def main():
    parser = argparse.ArgumentParser(description="Apply a rigid transform to a PCD map.")
    parser.add_argument("input", help="Input PCD path")
    parser.add_argument("output", help="Output PCD path")
    parser.add_argument("--xyz", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    args = parser.parse_args()
    print(
        "transform_pcd.py is a placeholder. "
        f"input={args.input} output={args.output} xyz={args.xyz} rpy={args.rpy}"
    )


if __name__ == "__main__":
    main()
