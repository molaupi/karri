# ******************************************************************************
# MIT License
#
# Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ******************************************************************************

import argparse
import pandas as pd

# Reads columns from two different CSV files and concatenates them to one output CSV file.
# Both CSV files need to have same number of rows.
# Must specify which columns to use from either CSV file.


argParser = argparse.ArgumentParser()
argParser.add_argument("-i1", "--infile1", type=str, help="path to first input file", required=True, dest="infile1")
argParser.add_argument("-c1", "--columns1", nargs='+', type=str, help="columns to read from first input file",
                       required=True, dest="columns1")
argParser.add_argument("-co1", "--out-column-names1", nargs='+',
                       help="optionally renames input columns from first file to given names in output",
                       dest="outnames1")
argParser.add_argument("-i2", "--infile2", type=str, help="path to second input file", required=True, dest="infile2")
argParser.add_argument("-c2", "--columns2", nargs='+', type=str, help="columns to read from second input file",
                       required=True, dest="columns2")
argParser.add_argument("-co2", "--out-column-names2", nargs='+',
                       help="optionally renames input columns from second file to given names in output",
                       dest="outnames2")
argParser.add_argument("-o", "--output", type=str, help="path to output file", required=True, dest="output")
argParser.add_argument("-oo", "--output-order", nargs='+', type=str,
                       help="order of columns in output. Must specify using output names.", dest="outputOrder")

args = argParser.parse_args()

if args.outnames1 is not None and len(args.outnames1) != len(args.columns1):
    print(f"Output names of columns in first input (given: {' '.join(args.outnames1)}) must contain same number of "
          f"names as specified input columns (given: {' '.join(args.columns1)} )")
    exit(1)
if args.outnames2 is not None and len(args.outnames2) != len(args.columns2):
    print(f"Output names of columns in second input (given: {' '.join(args.outnames2)}) must contain same number of "
          f"names as specified input columns (given: {' '.join(args.colums2)} )")
    exit(1)

outnames1 = args.outnames1
if outnames1 is None:
    outnames1 = args.columns1
outnames2 = args.outnames2
if outnames2 is None:
    outnames2 = args.columns2

outputOrder = args.outputOrder
if outputOrder is not None:
    if len(outputOrder) != len(args.columns1) + len(args.columns2):
        print(f"Output order specifies fewer columns than necessary.")
        exit(1)
    for outColName in outputOrder:
        if not (outColName in outnames1 or outColName in outnames2):
            print(f"Output name {outColName} specified in output order is not an output column.")
            exit(1)
else:
    outputOrder = outnames1 + outnames2

df1 = pd.read_csv(args.infile1)
df2 = pd.read_csv(args.infile2)

df1 = df1[args.columns1]
df2 = df2[args.columns2]
df1 = df1.rename(columns=dict(zip(args.columns1, outnames1)))
df2 = df2.rename(columns=dict(zip(args.columns2, outnames2)))

resdf = df1.join(df2)
resdf = resdf[outputOrder]

resdf.to_csv(args.output, index=False)
