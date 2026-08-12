#!/usr/bin/env python3
"""Convert origin/destination CSVs from 0-indexed to 1-indexed vertex IDs
and write them in the new semicolon-separated format with a commented header.
"""

import argparse
import pandas as pd


def convert(input_path: str, output_path: str) -> None:
    df = pd.read_csv(input_path)
    df["origin"] += 1
    df["destination"] += 1
    df["customers"] = 1

    rows = df.to_csv(sep=";", header=False, index=False, lineterminator="\n")
    rows = rows.replace(";", "; ")  # exactly one space after each separator

    with open(output_path, "w") as f:
        f.write("# left-node-id; right-node-id; customers\n")
        f.write(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="Path to the existing origin/destination CSV")
    parser.add_argument("output", help="Path to write the converted CSV")
    args = parser.parse_args()
    convert(args.input, args.output)


if __name__ == "__main__":
    main()
