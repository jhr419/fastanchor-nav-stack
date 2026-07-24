#!/usr/bin/env python3
import argparse


def main():
    parser = argparse.ArgumentParser(description="Downsample a PCD map.")
    parser.add_argument("input", help="Input PCD path")
    parser.add_argument("output", help="Output PCD path")
    parser.add_argument("--leaf-size", type=float, default=0.2)
    args = parser.parse_args()
    print(
        "downsample_pcd.py is a placeholder. "
        f"input={args.input} output={args.output} leaf_size={args.leaf_size}"
    )


if __name__ == "__main__":
    main()
