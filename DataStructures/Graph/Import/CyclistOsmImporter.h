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

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <routingkit/osm_graph_builder.h>
#include <routingkit/tag_map.h>
#include <routingkit/id_mapper.h>

#include "DataStructures/Containers/BitVector.h"
#include "DataStructures/Graph/Attributes/CapacityAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/LengthAttribute.h"
#include "DataStructures/Graph/Attributes/NumLanesAttribute.h"
#include "DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "DataStructures/Graph/Attributes/RoadGeometryAttribute.h"
#include "DataStructures/Graph/Attributes/SpeedLimitAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "Tools/Constants.h"
#include "Tools/EnumParser.h"
#include "Tools/LexicalCast.h"
#include "Tools/StringHelpers.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"

// An importer for reading graphs from OpenStreetMap data. The OSM data must be given as a file in
// PBF format. Edge attributes depending on the mode of transportation (travel time) refer to cyclists.
// First, the class Graph repeatedly calls nextVertex to read the next vertex and fetches various vertex attributes.
// Then it repeatedly calls nextEdge to read the next edge and fetches various edge attributes.
class CyclistOsmImporter {
public:

    CyclistOsmImporter(
            const IsRoadAccessibleByCategory &isCyclistAccessible = defaultIsCyclistAccessible,
            std::function<bool(uint64_t osm_node_id, const RoutingKit::TagMap &node_tags)> make_routing_node = nullptr)
            : isCyclistAccessible(isCyclistAccessible),
              make_routing_node(std::move(make_routing_node)) {}

    // Opens the input file(s) and reads the header line(s).
    void init(const std::string &filename) {
        // Returns the direction in which the road segment is open.
        auto getRoadDirection = [&](const OsmRoadCategory cat, const RoutingKit::TagMap &tags) {
            assert(cat != OsmRoadCategory::ROAD);

            const auto oneway = tags["oneway"];
            if (oneway) {
                if (stringEq(oneway, "no") || stringEq(oneway, "false") || stringEq(oneway, "0"))
                    return RoadDirection::OPEN_IN_BOTH;
                else if (stringEq(oneway, "yes") || stringEq(oneway, "true") || stringEq(oneway, "1"))
                    return RoadDirection::FORWARD;
                else if (stringEq(oneway, "-1") || stringEq(oneway, "reverse"))
                    return RoadDirection::REVERSE;
            }

            return roadDefaults.at(cat).direction;
        };

        EnumParser<OsmRoadCategory> parseOsmRoadCategory;

        // Returns true if the specified way is open for vehicles.
        auto isWayOpenForCyclists = [&](uint64_t, const RoutingKit::TagMap &tags) {
            // todo: be more specific than only OSM category (e.g. for path, check whether it is bicycle accessible using tags)
            const auto highway = tags["highway"];
            if (highway)
                try {
                    const auto cat = parseOsmRoadCategory(highway);
                    return isCyclistAccessible(cat);
                } catch (std::invalid_argument & /*e*/) {}
            return false;
        };

        // Invoked when a way is discovered that is open for vehicles.
        auto wayCallback = [&](uint64_t, const unsigned int seqId, const RoutingKit::TagMap &tags) {
            assert(seqId < wayCategory.size());
            assert(tags["highway"]);
            const auto cat = parseOsmRoadCategory(tags["highway"]);
            const auto dir = getRoadDirection(cat, tags);
            assert(dir != RoadDirection::CLOSED);

            wayCategory[seqId] = cat;

            switch (dir) {
                case RoadDirection::OPEN_IN_BOTH:
                    return RoutingKit::OSMWayDirectionCategory::open_in_both;
                case RoadDirection::FORWARD:
                    return RoutingKit::OSMWayDirectionCategory::only_open_forwards;
                case RoadDirection::REVERSE:
                    return RoutingKit::OSMWayDirectionCategory::only_open_backwards;
                default:
                    assert(false);
                    return RoutingKit::OSMWayDirectionCategory::closed;
            }
        };

        // Actually perform the work. Extract a graph from OSM data.
        const std::string fileNameWithExtension = filename + (endsWith(filename, ".osm.pbf") ? "" : ".osm.pbf");
        try {
            idMapping = RoutingKit::load_osm_id_mapping_from_pbf(
                    fileNameWithExtension, nullptr, isWayOpenForCyclists);
        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
            throw e;
        }
        edgeIDMapper = RoutingKit::IDMapper(idMapping.is_routing_way);
        nodeIDMapper = RoutingKit::IDMapper(idMapping.is_routing_node);
        const auto numWaysOpenForCyclists = idMapping.is_routing_way.population_count();
        wayCategory.resize(numWaysOpenForCyclists);
        osmGraph = load_osm_routing_graph_from_pbf(
                fileNameWithExtension, idMapping, wayCallback, nullptr,
                nullptr, false, RoutingKit::OSMRoadGeometry::uncompressed);
    }

    // Returns the number of vertices in the graph, or 0 if the number is not yet known.
    int numVertices() const {
        return osmGraph.node_count();
    }

    // Returns the number of edges in the graph, or 0 if the number is not yet known.
    int numEdges() const {
        return osmGraph.arc_count();
    }

    // Reads the next vertex from disk. Returns false if there are no more vertices.
    bool nextVertex() {
        return ++currentVertex < osmGraph.node_count();
    }

    // Returns the ID of the current vertex. Vertices must have sequential IDs from 0 to n − 1.
    int vertexId() const {
        return currentVertex;
    }

    // Reads the next edge from disk. Returns false if there are no more edges.
    bool nextEdge() {
        ++currentEdge;
        while (currentTail < static_cast<int>(osmGraph.node_count()) &&
               osmGraph.first_out[currentTail + 1] == currentEdge)
            ++currentTail;
        return currentEdge < osmGraph.arc_count();
    }

    // Returns the tail vertex of the current edge.
    int edgeTail() const {
        return currentTail;
    }

    // Returns the head vertex of the current edge.
    int edgeHead() const {
        assert(currentEdge >= 0);
        assert(currentEdge < osmGraph.arc_count());
        return osmGraph.head[currentEdge];
    }

    // Returns the value of the specified attribute for the current vertex/edge, or the attribute's
    // default value if the attribute is not part of the file format.
    template<typename Attr>
    typename Attr::Type getValue() const {
        return Attr::defaultValue();
    }

    // Closes the input file(s).
    void close() {}

    // Returns the OSM way ID of the given edge in the graph and the OSM node ID of the head of that edge.
    std::pair<uint64_t, uint64_t> getOsmIdsForEdgeAndHead(const int e) const {
        assert(e >= 0 && e < osmGraph.way.size() && e < osmGraph.head.size());
        const auto routing_way_id = osmGraph.way[e];
        const auto osm_way_id = edgeIDMapper.to_global(routing_way_id);
        const auto head_vertex = osmGraph.head[e];
        const auto osm_node_id = nodeIDMapper.to_global(head_vertex);
        return {osm_way_id, osm_node_id};
    }

private:
    // A struct that carries standard values for a set of static road properties.
    struct RoadDefaults {
        double cyclingSpeed;           // The cycling speed for a road of this category in km/h.
        RoadDirection direction;  // The direction in which the road segment is open.
    };

    // A map that contains default road properties for a set of OSM road categories.
    inline static const std::unordered_map<OsmRoadCategory, RoadDefaults> roadDefaults = {
            {OsmRoadCategory::MOTORWAY,       {18, RoadDirection::FORWARD}},
            {OsmRoadCategory::MOTORWAY_LINK,  {18, RoadDirection::FORWARD}},
            {OsmRoadCategory::TRUNK,          {18, RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::TRUNK_LINK,     {18, RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::PRIMARY,        {18, RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::PRIMARY_LINK,   {18, RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::SECONDARY,      {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::SECONDARY_LINK, {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::TERTIARY,       {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::TERTIARY_LINK,  {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::UNCLASSIFIED,   {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::RESIDENTIAL,    {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::LIVING_STREET,  {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::SERVICE,        {18,  RoadDirection::OPEN_IN_BOTH}},
            {OsmRoadCategory::PEDESTRIAN,     {4.5, RoadDirection::OPEN_IN_BOTH}}, // speed of walking and pushing a bike
            {OsmRoadCategory::TRACK,          {4.5, RoadDirection::OPEN_IN_BOTH}}, // speed of walking and pushing a bike
            {OsmRoadCategory::FOOTWAY,        {4.5, RoadDirection::OPEN_IN_BOTH}}, // speed of walking and pushing a bike
            {OsmRoadCategory::BRIDLEWAY,      {4.5, RoadDirection::OPEN_IN_BOTH}}, // speed of walking and pushing a bike
            {OsmRoadCategory::STEPS,          {0, RoadDirection::CLOSED}}, // cannot use steps with bike
            {OsmRoadCategory::PATH,           {4.5, RoadDirection::OPEN_IN_BOTH}}, // speed of walking and pushing a bike
            {OsmRoadCategory::CYCLEWAY,       {18,  RoadDirection::OPEN_IN_BOTH}},
    };


    const IsRoadAccessibleByCategory &isCyclistAccessible;
    std::function<bool(uint64_t osm_node_id, const RoutingKit::TagMap &node_tags)> make_routing_node;

    RoutingKit::OSMRoutingGraph osmGraph;     // The graph extracted from OSM data.
    RoutingKit::OSMRoutingIDMapping idMapping; // Mapping used in OSM Graph creation and used by edgeIDMapper.
    RoutingKit::IDMapper edgeIDMapper; // Mapping from osm way ids (global ids) to graph way ids (local ids) and vice versa.
    RoutingKit::IDMapper nodeIDMapper; // Mapping from osm node ids (global ids) to graph vertex ids (local ids) and vice versa.
    std::vector<OsmRoadCategory> wayCategory; // The OSM road category for each way.

    int currentVertex = -1; // The index of the current vertex in the OSM graph.
    int currentEdge = -1;   // The index of the current edge in the OSM graph.
    int currentTail = -1;   // The tail of the current edge.
};

// Returns the value of the LatLng attribute for the current vertex.
template<>
inline LatLngAttribute::Type CyclistOsmImporter::getValue<LatLngAttribute>() const {
    assert(currentVertex >= 0);
    assert(currentVertex < osmGraph.node_count());
    return {osmGraph.latitude[currentVertex], osmGraph.longitude[currentVertex]};
}

// Returns the value of the OsmNodeId attribute for the current vertex.
template<>
inline OsmNodeIdAttribute::Type CyclistOsmImporter::getValue<OsmNodeIdAttribute>() const {
    assert(currentVertex >= 0);
    assert(currentVertex < osmGraph.node_count());
    return nodeIDMapper.to_global(currentVertex);
}

// Returns the value of the length attribute for the current edge.
template<>
inline LengthAttribute::Type CyclistOsmImporter::getValue<LengthAttribute>() const {
    assert(currentEdge >= 0);
    assert(currentEdge < osmGraph.arc_count());
    return osmGraph.geo_distance[currentEdge];
}

// Returns the value of the OSM road category attribute for the current edge.
template<>
inline OsmRoadCategoryAttribute::Type CyclistOsmImporter::getValue<OsmRoadCategoryAttribute>() const {
    assert(currentEdge >= 0);
    assert(currentEdge < osmGraph.arc_count());
    return wayCategory[osmGraph.way[currentEdge]];
}

// Returns the value of the road geometry attribute for the current edge.
template<>
inline RoadGeometryAttribute::Type CyclistOsmImporter::getValue<RoadGeometryAttribute>() const {
    assert(currentEdge >= 0);
    assert(currentEdge < osmGraph.arc_count());
    const auto first = osmGraph.first_modelling_node[currentEdge];
    const auto last = osmGraph.first_modelling_node[currentEdge + 1];
    std::vector<LatLng> path(last - first);
    for (int v = first, i = 0; v < last; ++v, ++i)
        path[i] = {osmGraph.modelling_node_latitude[v], osmGraph.modelling_node_longitude[v]};
    return path;
}

// Returns the value of the free-flow speed attribute for the current edge.
template<>
inline FreeFlowSpeedAttribute::Type CyclistOsmImporter::getValue<FreeFlowSpeedAttribute>() const {
    assert(currentEdge >= 0);
    assert(currentEdge < osmGraph.arc_count());
    return std::round(roadDefaults.at(getValue<OsmRoadCategoryAttribute>()).cyclingSpeed);
}

// Returns the value of the travel time attribute for the current edge.
template<>
inline TravelTimeAttribute::Type CyclistOsmImporter::getValue<TravelTimeAttribute>() const {
    if (getValue<FreeFlowSpeedAttribute>() == 0)
        return INFTY;
    return std::round(36.0 * getValue<LengthAttribute>() / getValue<FreeFlowSpeedAttribute>());
}
