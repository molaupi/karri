# ******************************************************************************
# MIT License
#
# Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

# Reads rider paths file output by extract_rider_paths.py and computes total flow on each edge, i.e. how often each
# edge is traversed by a rider.

argParser = argparse.ArgumentParser()
argParser.add_argument("-p", "--inpaths", type=str, help="path to rider paths CSV file output by extract_rider_paths.py", required=True,
                       dest="pathsfile")
argParser.add_argument("-n", "--num-edges", type=int, help="total number of edges in input graph", required=True,
                       dest="num_edges")
argParser.add_argument("-o", "--output", type=str, help="path to output file", required=True, dest="output")

args = argParser.parse_args()

df = pd.read_csv(args.pathsfile, skipinitialspace=True, keep_default_na=False)
num_edges = args.num_edges


def path_string_to_edges(path_str):
    if len(path_str) == 0:
        return []
    strings = path_str.split(" : ")
    strings = [s.strip() for s in strings]
    edges = [int(e) for e in strings]
    return edges


paths = df["path_as_graph_edge_ids"]
paths = paths.apply(path_string_to_edges)

flow = [0] * num_edges
for p in paths:
    for e in p:
        flow[e] += 1

res = {"flow": flow}
res = pd.DataFrame(res)
output = args.output
if not output.endswith(".csv"):
    output += ".csv"
res.to_csv(output, index=False)
