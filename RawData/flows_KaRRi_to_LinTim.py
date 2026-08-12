"""Transform a flow CSV into link-id/customers format.

Reads a CSV file containing a single 'flow' column and writes a new CSV
with columns 'link-id' and 'customers', separated by '; ', with a
'# link-id; customers' header.
"""

import argparse

import pandas as pd


def transform(input_path: str, output_path: str) -> None:
    df = pd.read_csv(input_path)

    with open(output_path, "w", newline="\n") as f:
        f.write("# link-id; customers\n")
        for link_id, customers in enumerate(df["flow"], start=1):
            f.write(f"{link_id}; {customers}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_path", help="Path to the input CSV file (must have a 'flow' column)")
    parser.add_argument("output_path", help="Path to write the transformed CSV file")
    args = parser.parse_args()

    transform(args.input_path, args.output_path)
    print(f"Wrote {args.output_path}")
