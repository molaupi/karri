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

#pragma once


#include "Tools/EnumParser.h"

enum TransformMode {
    ODPairs,
    InitialVehicleLocations
};

enum LocType {
    VERTEX_ID,
    EDGE_ID,
    LATLNG,
    EPSG_31467
};

// Make EnumParser usable with OsmRoadCategory.
template<>
void EnumParser<LocType>::initNameToEnumMap() {
    nameToEnum = {
            {"vertex-id",  LocType::VERTEX_ID},
            {"edge-id",    LocType::EDGE_ID},
            {"lat-lng",    LocType::LATLNG},
            {"epsg-31467", LocType::EPSG_31467}
    };
}


struct AlwaysEligible {
    bool operator()(const int &) const { return true; }

    bool operator()(int &) const { return true; }
};

using Epsg31467Coords = std::pair<double, double>;

namespace transform_locations_input {

    using TrimPolicy = io::trim_chars<>;
    using QuotePolicy = io::no_quote_escape<','>;
    using OverflowPolicy = io::throw_on_overflow;
    using CommentPolicy = io::single_line_comment<'#'>;


    LatLng parseLatLngString(std::string s) {
        // Pairs are expected in format "( <lat as float> | <lng as float> )"
        s.erase(remove(s.begin(), s.end(), ' '), s.end());
        if (s.front() != '(' || s.back() != ')')
            throw std::invalid_argument(
                    "Parentheses in lat/lng pair string are wrong or string is ill formatted -- '" + s + "'");
        s.pop_back();
        s.erase(s.begin());
        size_t delimIdx;
        double lat = std::stod(s, &delimIdx);
        if (s[delimIdx] != '|' || delimIdx + 1 == s.size())
            throw std::invalid_argument(
                    "Delimiter '|' in lat/lng pair string is missing or string is ill formatted -- '" + s +
                    "'");
        double lng = std::stod(s.substr(delimIdx + 1));
        return {lat, lng};
    }

    Epsg31467Coords parseEastingNorthingString(std::string s) {
        // Pairs are expected in format "( <easting as float> | <northing as float> )"
        s.erase(remove(s.begin(), s.end(), ' '), s.end());
        if (s.front() != '(' || s.back() != ')')
            throw std::invalid_argument(
                    "Parentheses in lat/lng pair string are wrong or string is ill formatted -- '" + s + "'");
        s.pop_back();
        s.erase(s.begin());
        size_t delimIdx;
        double easting = std::stod(s, &delimIdx);
        if (s[delimIdx] != '|' || delimIdx + 1 == s.size())
            throw std::invalid_argument(
                    "Delimiter '|' in lat/lng pair string is missing or string is ill formatted -- '" + s +
                    "'");
        double northing = std::stod(s.substr(delimIdx + 1));
        return {easting, northing};
    }

    template<typename GraphT>
    class ODPairReader {
    public:
        ODPairReader(const std::unique_ptr<GraphT> &sourceGraphPtr,
                     const std::unique_ptr<std::vector<int>> &sourceGraphOrigToSeqEdgeIdsPtr,
                     const std::string &inputFileName,
                     const std::string &oColName,
                     const std::string &dColName) :
                sourceGraphPtr(sourceGraphPtr),
                sourceGraphOrigToSeqEdgeIdsPtr(sourceGraphOrigToSeqEdgeIdsPtr),
                inReader(inputFileName) {
            inReader.read_header(io::ignore_extra_column, oColName, dColName);
        }


        template<LocType inLocType>
        std::enable_if_t<inLocType == VERTEX_ID, std::vector<std::pair<int, int>>>
        readInput() {
            std::cout << "Reading source OD-pairs from file... " << std::flush;
            std::vector<std::pair<int, int>> pairs;
            int origin, destination;
            while (inReader.read_row(origin, destination)) {
                assert(origin >= 0);
                assert(destination >= 0);
                pairs.emplace_back(origin, destination);
            }
            std::cout << "done.\n";
            return pairs;
        }

        template<LocType inLocType>
        std::enable_if_t<inLocType == EDGE_ID, std::vector<std::pair<int, int>>>
        readInput() {
            assert(sourceGraphPtr && sourceGraphOrigToSeqEdgeIdsPtr);
            std::cout << "Reading source OD-pairs from file... " << std::flush;
            std::vector<std::pair<int, int>> pairs;
            int origin, destination;
            while (inReader.read_row(origin, destination)) {
                assert(origin >= 0);
                assert(destination >= 0);
                origin = sourceGraphOrigToSeqEdgeIdsPtr->at(origin);
                destination = sourceGraphOrigToSeqEdgeIdsPtr->at(destination);
                if (origin >= sourceGraphPtr->numEdges())
                    throw std::invalid_argument("Origin " + std::to_string(origin) + " is not a valid edge id.");
                if (destination >= sourceGraphPtr->numEdges())
                    throw std::invalid_argument(
                            "Origin " + std::to_string(destination) + " is not a valid edge id.");

                pairs.emplace_back(origin, destination);
            }
            std::cout << "done.\n";
            return pairs;
        }

        template<LocType inLocType>
        std::enable_if_t<inLocType == LATLNG, std::vector<std::pair<LatLng, LatLng>>>
        readInput() {

            std::cout << "Reading source OD-pairs from file... " << std::flush;
            std::vector<std::pair<LatLng, LatLng>> pairs;
            std::string origin, destination;
            while (inReader.read_row(origin, destination)) {
                pairs.emplace_back(parseLatLngString(origin), parseLatLngString(destination));
            }
            std::cout << "done.\n";
            return pairs;
        }

        template<LocType inLocType>
        std::enable_if_t<inLocType == EPSG_31467, std::vector<std::pair<Epsg31467Coords, Epsg31467Coords>>>
        readInput() {

            std::cout << "Reading source OD-pairs from file... " << std::flush;
            std::vector<std::pair<Epsg31467Coords, Epsg31467Coords>> pairs;
            std::string origin, destination;
            while (inReader.read_row(origin, destination)) {
                pairs.emplace_back(parseEastingNorthingString(origin), parseEastingNorthingString(destination));
            }
            std::cout << "done.\n";
            return pairs;
        }

    private:
        const std::unique_ptr<GraphT> &sourceGraphPtr;
        const std::unique_ptr<std::vector<int>> &sourceGraphOrigToSeqEdgeIdsPtr;
        io::CSVReader<2, TrimPolicy, QuotePolicy, OverflowPolicy, CommentPolicy> inReader;
    };

    template<typename GraphT>
    class InitialVehicleLocationReader {
    public:

        InitialVehicleLocationReader(const std::unique_ptr<GraphT> &sourceGraphPtr,
                                     const std::unique_ptr<std::vector<int>> &sourceGraphOrigToSeqEdgeIdsPtr,
                                     const std::string &inputFileName,
                                     const std::string &lColName) :
                sourceGraphPtr(sourceGraphPtr),
                sourceGraphOrigToSeqEdgeIdsPtr(sourceGraphOrigToSeqEdgeIdsPtr),
                inReader(inputFileName) {
            inReader.read_header(io::ignore_extra_column, lColName);
        }


        template<LocType inLocType>
        std::enable_if_t<inLocType == VERTEX_ID, std::vector<int>>
        readInput() {
            std::cout << "Reading source initial locations from file... " << std::flush;
            std::vector<int> locs;
            int loc;
            while (inReader.read_row(loc)) {
                assert(loc >= 0);
                locs.emplace_back(loc);
            }
            std::cout << "done.\n";
            return locs;
        }

        template<LocType inLocType>
        std::enable_if_t<inLocType == EDGE_ID, std::vector<int>>
        readInput() {
            assert(sourceGraphPtr && sourceGraphOrigToSeqEdgeIdsPtr);
            std::cout << "Reading source initial locations from file... " << std::flush;
            std::vector<int> locs;
            int loc;
            while (inReader.read_row(loc)) {
                assert(loc >= 0);
                loc = sourceGraphOrigToSeqEdgeIdsPtr->at(loc);
                if (loc >= sourceGraphPtr->numEdges())
                    throw std::invalid_argument("Location " + std::to_string(loc) + " is not a valid edge id.");

                locs.emplace_back(loc);
            }
            std::cout << "done.\n";
            return locs;
        }

        template<LocType inLocType>
        std::enable_if_t<inLocType == LATLNG, std::vector<LatLng>>
        readInput() {
            std::cout << "Reading source initial locations from file... " << std::flush;
            std::vector<LatLng> locs;
            std::string loc;
            while (inReader.read_row(loc)) {
                locs.emplace_back(parseLatLngString(loc));
            }
            std::cout << "done.\n";
            return locs;
        }

        template<LocType inLocType>
        std::enable_if_t<inLocType == EPSG_31467, std::vector<Epsg31467Coords>>
        readInput() {
            std::cout << "Reading source initial locations from file... " << std::flush;
            std::vector<Epsg31467Coords> locs;
            std::string loc;
            while (inReader.read_row(loc)) {
                locs.emplace_back(parseEastingNorthingString(loc));
            }
            std::cout << "done.\n";
            return locs;
        }

    private:
        const std::unique_ptr<GraphT> &sourceGraphPtr;
        const std::unique_ptr<std::vector<int>> &sourceGraphOrigToSeqEdgeIdsPtr;
        io::CSVReader<1, TrimPolicy, QuotePolicy, OverflowPolicy, CommentPolicy> inReader;

    };
}