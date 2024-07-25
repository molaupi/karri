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

#pragma once

namespace mixfix::geojson {

        static void addGeoJsonFeaturesForLine(const std::vector<LatLng> &latLngPath, const int lineId,
                                              const int vehicleTravelTime,
                                              nlohmann::json &featureCol) {
            static char lastEdgeColor[] = "red";


            // LineString feature for the line's edges
            nlohmann::json lineFeature;
            lineFeature["type"] = "Feature";
            lineFeature["geometry"] = {{"type",        "LineString"},
                                       {"coordinates", nlohmann::json::array()}};

            for (const auto &latLng: latLngPath) {
                const auto coord = nlohmann::json::array({latLng.lngInDeg(), latLng.latInDeg()});
                lineFeature["geometry"]["coordinates"].push_back(coord);
            }
            lineFeature["properties"] = {{"line_id", lineId},
                                         {"vehicle_travel_time", vehicleTravelTime}};
            featureCol["features"].push_back(lineFeature);

            // MultiPoint feature for start and end of line
            nlohmann::json startFeature;
            startFeature["type"] = "Feature";
            startFeature["geometry"] = {{"type",        "MultiPoint"},
                                        {"coordinates", nlohmann::json::array()}};
            const auto firstEdgeTail = latLngPath.front();
            const auto lastEdgeHead = latLngPath.back();
            startFeature["geometry"]["coordinates"].push_back({firstEdgeTail.lngInDeg(), firstEdgeTail.latInDeg()});
            startFeature["geometry"]["coordinates"].push_back({lastEdgeHead.lngInDeg(), lastEdgeHead.latInDeg()});
            startFeature["properties"] = {{"stroke",  lastEdgeColor},
                                          {"line_id", lineId}};
            featureCol["features"].push_back(startFeature);
        }

} // end namespace mixfix::geojson