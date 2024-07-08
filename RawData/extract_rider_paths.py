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
argParser.add_argument("-p", "--inpaths", type=str, help="path to vehpaths.csv file output by KaRRi", required=True,
                       dest="pathsfile")
argParser.add_argument("-v", "--vehicles", type=str, help="path to .csv file containing initial vehicle locations",
                       required=True,
                       dest="vehiclesfile")
argParser.add_argument("--pickup-locs", action="store_true", dest="include_pickup_locs",
                       help="include pickup location in path")
argParser.add_argument("--no-pickup-locs", action="store_false", dest="include_pickup_locs",
                       help="do not include pickup location in path")
argParser.set_defaults(include_pickup_locs=True)
argParser.add_argument("-o", "--output", type=str, help="path to output file", required=True, dest="output")

args = argParser.parse_args()

df = pd.read_csv(args.pathsfile, skipinitialspace=True, keep_default_na=False)
vehicles_df = pd.read_csv(args.vehiclesfile, skipinitialspace=True, keep_default_na=False)
initial_veh_locs = vehicles_df["initial_location"]


def read_events(events_str):
    strings = events_str.split(" : ")
    strings = [s.strip() for s in strings]
    strings = map(lambda x: x[1:len(x) - 1], strings)
    events = [(int(s.split(" - ")[0]), s.split(" - ")[1]) for s in strings]
    return events


df = df[["vehicle_id", "stop_number", "events", "graph_edge_ids_to_stop"]]
df = df.sort_values("vehicle_id", kind="stable")
vehpaths = df[["vehicle_id", "graph_edge_ids_to_stop"]]
vehpaths = vehpaths.groupby("vehicle_id", as_index=False)["graph_edge_ids_to_stop"].apply(list)


df = df[df["events"] != '']
events_with_stop = df.apply(lambda x: [(e[0], x["vehicle_id"], x["stop_number"]) for e in read_events(x["events"])],
                            axis='columns')
events_with_stop = [e for es in events_with_stop for e in es]

maxReqId = max([e[0] for e in events_with_stop])
numRequests = maxReqId + 1
asgns = [(-1, math.inf, math.inf)] * numRequests

for e in events_with_stop:
    reqid = e[0]
    vehid = e[1]
    stopnum = e[2]
    if asgns[reqid][0] != -1 and asgns[reqid][0] != vehid:
        print(
            "Pickup of request %s is with vehicle %s but dropoff is with vehicle %s" % (reqid, asgns[reqid][0], vehid))
    asgns[reqid] = (vehid, min(stopnum, asgns[reqid][1]), max(stopnum, asgns[reqid][1]))

for (v, p, d) in asgns:
    if p == math.inf or d == math.inf:
        continue
    path = vehpaths.loc[vehpaths["vehicle_id"] == v]["graph_edge_ids_to_stop"].values[0]
    numstops = len(path)
    if p >= numstops:
        print("Event at stop %s of vehicle %s even though vehicle has only %s stops." % (p, v, numstops))
    if d >= numstops:
        print("Event at stop %s of vehicle %s even though vehicle has only %s stops." % (d, v, numstops))


def getPickupLoc(asgn, vehpaths):
    if asgn[1] == math.inf:
        return ""
    vehid = asgn[0]
    vehpath = vehpaths.loc[vehpaths["vehicle_id"] == vehid]["graph_edge_ids_to_stop"].values[0]
    i = asgn[1]
    while i >= 0:
        leg_to_pickup = vehpath[i]
        if leg_to_pickup != "":
            locs_to_pickup = leg_to_pickup.split(" : ")
            return locs_to_pickup[-1]
        i -= 1
    return str(initial_veh_locs[vehid])


def getRiderPath(asgn, vehpaths):
    if asgn[1] == math.inf or asgn[2] == math.inf:
        return ""
    vehid = asgn[0]
    path_as_list = vehpaths.loc[vehpaths["vehicle_id"] == vehid]["graph_edge_ids_to_stop"].values[0]
    path_as_list = path_as_list[asgn[1] + 1:asgn[2] + 1]
    path_as_list = [s for s in path_as_list if s != ""]
    if args.include_pickup_locs:
        path_as_list = [getPickupLoc(asgn, vehpaths)] + path_as_list
    path_as_string = " : ".join(path_as_list)
    return path_as_string


riderpaths = [''] * numRequests
for i in range(numRequests):
    riderpaths[i] = getRiderPath(asgns[i], vehpaths)

res = {"request_id": range(numRequests), "path_as_graph_edge_ids": riderpaths}
res = pd.DataFrame(res)
res.to_csv(args.output, index=False)
