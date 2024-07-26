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
              "Usage: ODPairsRoutingKitToCsv -p <file> -o <file>\n"
              "Reads two RoutingKit vector files for containing origins/destinations and writes them into a single CSV file.\n"
              "  -p <file>         take origins from <file>.origin and destinations from <file>.destination\n"
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
        const auto fileBaseName = clp.getValue<std::string>("p");
        const auto originFileName = fileBaseName + ".origin";
        const auto destinationFileName = fileBaseName + ".destination";
        auto outputFileName = clp.getValue<std::string>("o");

        // Read origins from file:
        std::cout << "Read origins from file... ";
        const auto origins = RoutingKit::load_vector<uint32_t>(originFileName);
        if (origins.empty())
            throw std::invalid_argument("origins cannot be empty!");
        std::cout << "done.\n";

        // Read destinations from file:
        std::cout << "Read destinations from file... ";
        const auto destinations = RoutingKit::load_vector<uint32_t>(destinationFileName);
        if (destinations.empty())
            throw std::invalid_argument("destinations cannot be empty!");
        std::cout << "done.\n";

        if (origins.size() != destinations.size())
            throw std::invalid_argument(
                    "Not the same number of origins (" + std::to_string(origins.size()) + ") as destinations(" +
                    std::to_string(destinations.size()) + ")!");

        // Write CSV output
        std::cout << "Writing OD-pairs to CSV output...";
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";
        std::ofstream out(outputFileName);
        out << "origin,destination\n";
        for (int i = 0; i < origins.size(); ++i)
            out << origins[i] << "," << destinations[i] << "\n";
        std::cout << "done.\n";
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}