#!/usr/bin/env python3
import argparse


def main():
    parser = argparse.ArgumentParser(description="Crop a PCD map by xyz bounds.")
    parser.add_argument("input", help="Input PCD path")
    parser.add_argument("output", help="Output PCD path")
    parser.add_argument("--min", nargs=3, type=float, default=[-50.0, -50.0, -5.0])
    parser.add_argument("--max", nargs=3, type=float, default=[50.0, 50.0, 5.0])
    args = parser.parse_args()
    print(
        "crop_pcd.py is a placeholder. "
        f"input={args.input} output={args.output} min={args.min} max={args.max}"
    )


if __name__ == "__main__":
    main()
