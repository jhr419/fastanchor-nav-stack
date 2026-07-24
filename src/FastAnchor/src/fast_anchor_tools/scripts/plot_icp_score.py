#!/usr/bin/env python3
import argparse


def main():
    parser = argparse.ArgumentParser(description="Plot ICP fitness scores from a log file.")
    parser.add_argument("log", help="Input log or CSV path")
    parser.add_argument("--output", default="", help="Optional output image path")
    args = parser.parse_args()
    print(
        "plot_icp_score.py is a placeholder. "
        f"log={args.log} output={args.output}"
    )


if __name__ == "__main__":
    main()
