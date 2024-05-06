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
from mat4py import loadmat

import pandas as pd

# Reads .mat file for network and requests in a format used by Andres Fielbaum and Javier Alonso-Mora and converts
# it to csv file for requests. Note that we discard information about network in given data and instead map
# requests to our own network later.


argParser = argparse.ArgumentParser()
argParser.add_argument("-i", "--infile", type=str, help="path to .mat input file", required=True, dest="infile")
argParser.add_argument("-o", "--output", type=str, help="base path for output files", required=True, dest="output")

# default r-name: Utrecht: Requests, Sunshine: RequestsSunshine
# default r-time-scale: Utrecht: 3600, Sunshine: 60


args = argParser.parse_args()

if "utrecht" in args.infile.lower():
    req_name = "Requests"


    def req_time_scale(x):
        return (x - 2) * 3600
else:
    req_name = "RequestsSunshine"


    def req_time_scale(x):
        return x * 60

data = loadmat(args.infile)
nodes = data["Nodes"]
requests = data[req_name]

# Get .poly file that contains all nodes
latitudes = [ll[0] for ll in nodes]
minLat = min(latitudes)
maxLat = max(latitudes)
latPadding = 0.05 * (maxLat - minLat)
minLat -= latPadding
maxLat += latPadding
longitudes = [ll[1] for ll in nodes]
minLng = min(longitudes)
maxLng = max(longitudes)
lngPadding = 0.05 * (maxLng - minLng)
minLng -= lngPadding
maxLng += lngPadding

with open(args.output + "_Inner.poly", "w") as b:
    b.write("polygon\n1\n")
    b.write("\t%s\t%s\n" % (minLng, maxLat))
    b.write("\t%s\t%s\n" % (maxLng, maxLat))
    b.write("\t%s\t%s\n" % (maxLng, minLat))
    b.write("\t%s\t%s\n" % (minLng, minLat))
    b.write("\t%s\t%s\n" % (minLng, maxLat))
    b.write("END\nEND")

if not "utrecht" in args.infile.lower():
    latPadding = 0.05 * (maxLat - minLat)
    minLat -= latPadding
    maxLat += latPadding
    lngPadding = 0.05 * (maxLng - minLng)
    minLng -= lngPadding
    maxLng += lngPadding
    with open(args.output + "_Outer.poly", "w") as b:
        b.write("polygon\n1\n")
        b.write("\t%s\t%s\n" % (minLng, maxLat))
        b.write("\t%s\t%s\n" % (maxLng, maxLat))
        b.write("\t%s\t%s\n" % (maxLng, minLat))
        b.write("\t%s\t%s\n" % (minLng, minLat))
        b.write("\t%s\t%s\n" % (minLng, maxLat))
        b.write("END\nEND")

# Process nodes
nodes_res = {"latitude": [n[0] for n in nodes], "longitude": [n[1] for n in nodes]}
nodes_res = pd.DataFrame(nodes_res)
nodes_res.to_csv(args.output + ".nodes.csv", index=False)

# Process edges
edges = data["Edges"]
edges.sort(key=lambda x: x[0])
edges_res = {"tail": [int(e[0]) - 1 for e in edges], "head": [int(e[1]) - 1 for e in edges],
             "travel_time": [e[2] for e in edges]}
edges_res = pd.DataFrame(edges_res)
edges_res.to_csv(args.output + ".edges.csv", index=False)

# Process requests
nodes_latlng_str = ["(%s|%s)" % (lat, lng) for (lat, lng) in zip(latitudes, longitudes)]

requests_res = {"origin": [nodes_latlng_str[int(r[0])] for r in requests],
                "destination": [nodes_latlng_str[int(r[1])] for r in requests],
                "req_time": [int(round(req_time_scale(r[2]))) for r in requests]}
requests_res = pd.DataFrame(requests_res)

print("Max req time: %s" % max(requests_res["req_time"]))
print("Min req time: %s" % min(requests_res["req_time"]))
requests_res.to_csv(args.output + ".requests.csv", index=False)
