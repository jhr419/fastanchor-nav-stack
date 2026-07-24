#!/usr/bin/env python3
import argparse


def main():
    parser = argparse.ArgumentParser(description="Evaluate a FastAnchor trajectory.")
    parser.add_argument("trajectory", help="Estimated trajectory file")
    parser.add_argument("--reference", default="", help="Optional reference trajectory")
    args = parser.parse_args()
    print(
        "evaluate_trajectory.py is a placeholder. "
        f"trajectory={args.trajectory} reference={args.reference}"
    )


if __name__ == "__main__":
    main()
