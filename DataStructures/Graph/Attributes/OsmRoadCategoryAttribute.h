/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

#include <cassert>
#include <ostream>

#include "DataStructures/Graph/Attributes/AbstractAttribute.h"
#include "Tools/EnumParser.h"

// Road categories defined by OpenStreetMap.
enum class OsmRoadCategory {
    MOTORWAY,
    MOTORWAY_LINK,
    TRUNK,
    TRUNK_LINK,
    PRIMARY,
    PRIMARY_LINK,
    SECONDARY,
    SECONDARY_LINK,
    TERTIARY,
    TERTIARY_LINK,
    UNCLASSIFIED,
    RESIDENTIAL,
    LIVING_STREET,
    SERVICE,
    PEDESTRIAN,
    TRACK,
    FOOTWAY,
    BRIDLEWAY,
    STEPS,
    PATH,
    CYCLEWAY,
    ROAD,
};

static const unsigned long NUM_OSM_ROAD_CATEGORIES = static_cast<unsigned long>(OsmRoadCategory::ROAD) + 1;

using IsRoadAccessibleByCategory = std::function<bool(const OsmRoadCategory&)>;

static IsRoadAccessibleByCategory defaultIsVehicleAccessible = [](const OsmRoadCategory& cat) {
    return cat != OsmRoadCategory::ROAD && cat <= OsmRoadCategory::SERVICE;
};

static IsRoadAccessibleByCategory defaultIsPedestrianAccessible = [](const OsmRoadCategory& cat) {
    // We allow pedestrians to walk on cycle paths as shared paths are sometimes misdeclared.
    return cat != OsmRoadCategory::ROAD && cat >= OsmRoadCategory::TERTIARY; // && cat != OsmRoadCategory::CYCLEWAY;
};

static IsRoadAccessibleByCategory defaultIsCyclistAccessible = [](const OsmRoadCategory& cat) {
    return cat != OsmRoadCategory::ROAD && cat >= OsmRoadCategory::SECONDARY && cat != OsmRoadCategory::STEPS;
};

// Make EnumParser usable with OsmRoadCategory.
template<>
void EnumParser<OsmRoadCategory>::initNameToEnumMap() {
    nameToEnum = {
            {"motorway",       OsmRoadCategory::MOTORWAY},
            {"motorway_link",  OsmRoadCategory::MOTORWAY_LINK},
            {"trunk",          OsmRoadCategory::TRUNK},
            {"trunk_link",     OsmRoadCategory::TRUNK_LINK},
            {"primary",        OsmRoadCategory::PRIMARY},
            {"primary_link",   OsmRoadCategory::PRIMARY_LINK},
            {"secondary",      OsmRoadCategory::SECONDARY},
            {"secondary_link", OsmRoadCategory::SECONDARY_LINK},
            {"tertiary",       OsmRoadCategory::TERTIARY},
            {"tertiary_link",  OsmRoadCategory::TERTIARY_LINK},
            {"unclassified",   OsmRoadCategory::UNCLASSIFIED},
            {"residential",    OsmRoadCategory::RESIDENTIAL},
            {"living_street",  OsmRoadCategory::LIVING_STREET},
            {"service",        OsmRoadCategory::SERVICE},
            {"pedestrian",     OsmRoadCategory::PEDESTRIAN},
            {"track",          OsmRoadCategory::TRACK},
            {"footway",        OsmRoadCategory::FOOTWAY},
            {"bridleway",      OsmRoadCategory::BRIDLEWAY},
            {"steps",          OsmRoadCategory::STEPS},
            {"path",           OsmRoadCategory::PATH},
            {"cycleway",       OsmRoadCategory::CYCLEWAY},
            {"road",           OsmRoadCategory::ROAD}
    };
}

static std::array<std::string, NUM_OSM_ROAD_CATEGORIES> osmRoadCategoryNames = {
        "motorway",
        "motorway_link",
        "trunk",
        "trunk_link",
        "primary",
        "primary_link",
        "secondary",
        "secondary_link",
        "tertiary",
        "tertiary_link",
        "unclassified",
        "residential",
        "living_street",
        "service",
        "pedestrian",
        "track",
        "footway",
        "bridleway",
        "steps",
        "path",
        "cycleway",
        "road"
};

static std::string nameOfOsmRoadCategory(const OsmRoadCategory& cat) {
    return osmRoadCategoryNames[static_cast<unsigned long>(cat)];
}

// Writes the character representation of the specified OSM road category to the given stream.
inline std::ostream &operator<<(std::ostream &os, const OsmRoadCategory cat) {
    os << nameOfOsmRoadCategory(cat);
    return os;
}

// An attribute associating an OSM road category with each edge of a graph.
class OsmRoadCategoryAttribute : public AbstractAttribute<OsmRoadCategory> {
public:
    // Returns the attribute's default value.
    static Type defaultValue() {
        return OsmRoadCategory::ROAD;
    }

    // Returns the OSM road category of edge e.
    const Type &osmRoadCategory(const int e) const {
        assert(e >= 0);
        assert(e < values.size());
        return values[e];
    }

    // Returns a reference to the OSM road category of edge e.
    Type &osmRoadCategory(const int e) {
        assert(e >= 0);
        assert(e < values.size());
        return values[e];
    }

protected:
    static constexpr const char *NAME = "osm_road_category"; // The attribute's unique name.
};

