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

#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"

namespace karri::stats {

    // Statistics about the OSM road categories of edges considered for PDLocs.
    struct OsmRoadCategoryStats {

        OsmRoadCategoryStats() : counts() {
            counts.fill(0);
        }

        void incCountForCat(const OsmRoadCategory& cat) {
            ++counts[static_cast<unsigned long>(cat)];
        }

        // Write a textual representation to the specified output stream.
        friend inline std::ostream &operator<<(std::ostream &os, const OsmRoadCategoryStats &stats) {
            uint64_t totalCount = 0;
            for (const auto& c : stats.counts) totalCount += c;

            os << "\n" << "Road categories of PDLocs:\nTotal count: " << totalCount << "\n";
            for (int i = 0; i < NUM_OSM_ROAD_CATEGORIES; ++i) {
                os << nameOfOsmRoadCategory(static_cast<OsmRoadCategory>(i)) << ": " << stats.counts[i] << "\n";
            }

            return os;
        }

        std::string getLoggerRow() const {
            std::stringstream ss;
            for (int i = 0; i < NUM_OSM_ROAD_CATEGORIES; ++i)
                ss << counts[i] << ", ";

            uint64_t totalCount = 0;
            for (const auto& c : counts) totalCount += c;
            ss << totalCount;
            return ss.str();
        }

    private:
        std::array<uint32_t, NUM_OSM_ROAD_CATEGORIES> counts;

    public:

        static constexpr auto LOGGER_NAME = "road_cats_of_pdlocs.csv";
        static std::string getLoggerCols() {
            std::stringstream header;
            for (int i = 0; i < NUM_OSM_ROAD_CATEGORIES; ++i)
                header << osmRoadCategoryNames[i] << ",";
            header << "total\n";
            return header.str();
        }
    };

}