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
import math

import pandas as pd

# Reads vehicle paths file output by KaRRi and extracts paths taken by each rider.


argParser = argparse.ArgumentParser()
argParser.add_argument("-i", "--infile", type=str, help="path to vehpaths.csv file output by KaRRi", required=True,
                       dest="infile")
argParser.add_argument("-o", "--output", type=str, help="path to output file", required=True, dest="output")

args = argParser.parse_args()

df = pd.read_csv(args.infile, skipinitialspace=True, keep_default_na=False)


def read_events(events_str):
    strings = events_str.split(" : ")
    strings = [s.strip() for s in strings]
    strings = map(lambda x: x[1:len(x) - 1], strings)
    events = [(int(s.split(" - ")[0]), s.split(" - ")[1]) for s in strings]
    return events


maxReqId = max(df["events"].apply(lambda x: max([e[0] for e in read_events(x)])))
numRequests = maxReqId + 1

df = df[["vehicle_id", "stop_number", "events", "graph_edge_ids_to_stop"]]
df = df.sort_values("vehicle_id", kind="stable")

asgns = [(-1, math.inf, math.inf)] * numRequests
events_with_stop = df.apply(lambda x: [(e[0], x["vehicle_id"], x["stop_number"]) for e in read_events(x["events"])],
                            axis='columns')
events_with_stop = [e for es in events_with_stop for e in es]
for e in events_with_stop:
    reqid = e[0]
    vehid = e[1]
    stopnum = e[2]
    asgns[reqid] = (vehid, min(stopnum, asgns[reqid][1]), max(stopnum, asgns[reqid][1]))

vehpaths = df[["vehicle_id", "graph_edge_ids_to_stop"]]
vehpaths = vehpaths.groupby("vehicle_id").apply(lambda x: x["graph_edge_ids_to_stop"].to_list())


def getRiderPath(asgn, vehpaths):
    if asgn[1] == math.inf or asgn[2] == math.inf:
        return ""
    vehid = asgn[0]
    path_as_list = vehpaths[vehid][asgn[1] + 1:asgn[2] + 1]
    path_as_list = [s for s in path_as_list if s != ""]
    path_as_string = " : ".join(path_as_list)
    return path_as_string

riderpaths = [''] * numRequests
for i in range(numRequests):
    riderpaths[i] = getRiderPath(asgns[i], vehpaths)

res = {"request_id" : range(numRequests), "path_as_graph_edge_ids" : riderpaths}
res = pd.DataFrame(res)
res.to_csv(args.output, index=False)
