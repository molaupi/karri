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
argParser.add_argument("-r", "--requests", type=str, help="path to original requests file", required=True,
                       dest="requestsfile")
argParser.add_argument("-a", "--answered", type=str, help="name of KaRRi result",
                       required=True, dest="resultsname")
argParser.add_argument("-o", "--output", type=str, help="path to output file", required=True, dest="output")

args = argParser.parse_args()

origreq = pd.read_csv(args.requestsfile, skipinitialspace=True, keep_default_na=False)
print("Number of original requests: ", len(origreq))
bestasgn = pd.read_csv(args.resultsname + "_run1.bestassignments.csv", skipinitialspace=True, keep_default_na=False)

# Requests that were not answered have a -1 in the not_using_vehicle column
answered_bestasgn = bestasgn[bestasgn["not_using_vehicle"] != "-1"]
answered = answered_bestasgn["request_id"]
res = origreq.iloc[answered,]
print("Number of answered requests: ", len(res))
res.to_csv(args.output, index=False)
