/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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
#include "DataStructures/Utilities/OriginDestination.h"
#include <routingkit/vector_io.h>

inline void printUsage() {
    std::cout <<
              "Usage: ODPairsCsvToRoutingKit -p <file> -o <file>\n"
              "Reads a CSV file containing ODPairs (columns 'origin' and 'destination') and transforms it into two\n"
              "files containing origin and destination, respectively, in RoutingKit binary vector format.\n"
              "  -p <file>         take origin-destination pairs from <file>\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}


int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto odPairsFileName = clp.getValue<std::string>("p");
        auto outputFileName = clp.getValue<std::string>("o");

        // Read OD-pairs from file:
        std::cout << "Read OD-pairs from file... ";
        auto odPairs = importODPairsFrom(odPairsFileName);
        if (odPairs.empty())
            throw std::invalid_argument("OD-pairs cannot be empty!");
        std::vector<unsigned> origin(odPairs.size());
        std::vector<unsigned> destination(odPairs.size());
        for (int i = 0; i < odPairs.size(); ++i) {
            origin[i] = static_cast<unsigned>(odPairs[i].origin);
            destination[i] = static_cast<unsigned>(odPairs[i].destination);
        }
        std::cout << "done.\n";

        // Write binary output
        std::cout << "Writing binary to output...";
        RoutingKit::save_vector(outputFileName + ".origin", origin);
        RoutingKit::save_vector(outputFileName + ".destination", destination);
        std::cout << "done.\n";
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}