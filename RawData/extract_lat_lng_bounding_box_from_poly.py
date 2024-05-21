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

# Extracts bounding box for polygon in given .poly file


argParser = argparse.ArgumentParser()
argParser.add_argument("-i", "--infile", type=str, help="path to .poly file", required=True,
                       dest="infile")
argParser.add_argument("-p", "--padding", type=float, help="padding on each side of bounding box, relative to width/height", required=False,
                       dest="padding", default=0.0)
argParser.add_argument("-o", "--output", type=str, help="path to output .poly file", required=True, dest="output")

args = argParser.parse_args()

minLng = math.inf
maxLng = 0
minLat = math.inf
maxLat = 0
with open(args.infile, "r") as file:
    for line in file:
        if not line.startswith("  "):
            continue
        # data lines have format '  <lng>  <lat>'
        line = line.rstrip().lstrip()
        line = line.split("  ")
        lng = float(line[0])
        lat = float(line[1])
        minLng = min(minLng, lng)
        maxLng = max(maxLng, lng)
        minLat = min(minLat, lat)
        maxLat = max(maxLat, lat)

latPadding = args.padding * (maxLat - minLat)
minLat -= latPadding
maxLat += latPadding
lngPadding = args.padding * (maxLng - minLng)
minLng -= lngPadding
maxLng += lngPadding

with open(args.output, "w") as b:
    b.write("polygon\n1\n")
    b.write("  %s  %s\n" % (minLng, maxLat))
    b.write("  %s  %s\n" % (maxLng, maxLat))
    b.write("  %s  %s\n" % (maxLng, minLat))
    b.write("  %s  %s\n" % (minLng, minLat))
    b.write("  %s  %s\n" % (minLng, maxLat))
    b.write("END\nEND")