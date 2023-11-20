/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************

#include <cstdlib>
#include <random>
#include <fstream>
#include "Tools/CommandLine/CommandLineParser.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "DataStructures/Utilities/OriginDestination.h"

inline void printUsage() {
    std::cout <<
              "Usage: DrawRandomDepartureTimes -p <file> -t <file> -o <file>\n"
              "Turns a set of origin-destination pairs into a set of ride requests by adding departure times.\n"
              "Departure times are drawn randomly with a probability distribution specified in a reference request file.\n"
              "Input origin and destination are expected to be edges.\n"
              "  -s <seed>         start random bit generator with <seed> (dflt: 0)\n"
              "  -p <file>         take origin-destination pairs from <file>\n"
              "  -t <file>         distribute departure times as in request file <file>\n"
              "  -t-col-name       column name of request time in reference file (dflt: req_time).\n"
              "  -scale <factor>   divide all departure times by a factor of <factor> (dflt: 1)\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}


// Reference distribution of request times is represented by sorting times into bins of a certain distribution.
static constexpr int BIN_DURATION = 300;

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto seed = clp.getValue<int>("s", 0);
        const auto scaleFactor = clp.getValue<int>("scale", 1);
        const auto odPairsFileName = clp.getValue<std::string>("p");
        const auto timeDistFileName = clp.getValue<std::string>("t");
        const auto timeColName = clp.getValue<std::string>("t-col-name", "req_time");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";

        // Read OD-pairs from file:
        std::cout << "Read OD-pairs from file... ";
        auto odPairs = importODPairsFrom(odPairsFileName);
        if (odPairs.empty())
            throw std::invalid_argument("OD-pairs cannot be empty!");
        std::cout << "done.\n";

        // Read the time distribution as bins and build requests:
        int reqTime;
        io::CSVReader<1, io::trim_chars<>> timeDistFileReader(timeDistFileName);
        timeDistFileReader.read_header(io::ignore_extra_column, timeColName);
        std::vector<karri::Request> requests(odPairs.size(), karri::Request());
        std::cout << "Read reference time distribution from file, determine request times and construct requests... ";
        std::vector<int> numReqTimesInBins;
        while (timeDistFileReader.read_row(reqTime)) {
            const auto bin = reqTime / BIN_DURATION;
            if (numReqTimesInBins.size() <= bin) {
                numReqTimesInBins.resize(bin + 1, 0);
            }
            ++numReqTimesInBins[bin];
        }

        std::minstd_rand binrand(seed + 1);
        std::discrete_distribution<> binDist(numReqTimesInBins.begin(), numReqTimesInBins.end());
        std::uniform_int_distribution<> withinBinDist(0, BIN_DURATION - 1);

        for (int i = 0; i < odPairs.size(); ++i) {
            reqTime = (binDist(binrand) * BIN_DURATION + withinBinDist(binrand)) / scaleFactor;
            requests[i] = {i, odPairs[i].origin, odPairs[i].destination, 1,reqTime, reqTime};
        }


        std::stable_sort(requests.begin(), requests.end(), [](const auto &lhs, const auto &rhs) {
            return lhs.issuingTime < rhs.issuingTime;
        });
        std::cout << "done.\n";


        // Write requests to output
        std::cout << "Writing requests to output...";
        std::ofstream out(outputFileName);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        out << "origin, destination, req_time\n";
        for (const auto &req: requests) {
            out << req.origin << ", " << req.destination << ", " << req.issuingTime << "\n";
        }
        std::cout << "done.\n";
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}