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

# Reads vehicle paths file output by KaRRi and extracts paths taken by each rider.


argParser = argparse.ArgumentParser()
argParser.add_argument("-i", "--infile", type=str, help="path to CSV file containing lat/lng column", required=True,
                       dest="infile")
argParser.add_argument("-c", "--column-name", type=str, help="name of lat/lng column", required=True,
                       dest="column_name")
argParser.add_argument("-o", "--output", type=str, help="path to output .poly file", required=True, dest="output")

args = argParser.parse_args()

df = pd.read_csv(args.infile, skipinitialspace=True, keep_default_na=False)
col_name = args.column_name

# Expect one column containing one pair per row in format '(<latitude>|<longitude>)'
def read_latlng(s):
    s = s[1:len(s)-1]
    s = s.split('|')
    return (float(s[0]), float(s[1]))

latlng = df[col_name]
latlng = [read_latlng(s) for s in latlng]

# Get .poly file that contains all nodes
latitudes = [ll[0] for ll in latlng]
minLat = min(latitudes)
maxLat = max(latitudes)
latPadding = 0.05 * (maxLat - minLat)
minLat -= latPadding
maxLat += latPadding
longitudes = [ll[1] for ll in latlng]
minLng = min(longitudes)
maxLng = max(longitudes)
lngPadding = 0.05 * (maxLng - minLng)
minLng -= lngPadding
maxLng += lngPadding

with open(args.output, "w") as b:
    b.write("polygon\n1\n")
    b.write("\t%s\t%s\n" % (minLng, maxLat))
    b.write("\t%s\t%s\n" % (maxLng, maxLat))
    b.write("\t%s\t%s\n" % (maxLng, minLat))
    b.write("\t%s\t%s\n" % (minLng, minLat))
    b.write("\t%s\t%s\n" % (minLng, maxLat))
    b.write("END\nEND")