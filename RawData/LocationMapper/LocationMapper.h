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


// Facilitates the mappings of input locations to output locations.
// The central part maps a given latitude/longitude coordinate to the closest vertex in the target graph.
// Uses an input mapper to transform the input location from the input representation to a LatLng and an
// output mapper to transform the vertex in the target graph to the desired output representation (vertex or edge).
// The output mapper may additionally make use of the original location for a better mapping.
// The mapping can be given a location (vertex or edge) in the output graph that is avoided if desired.
template<
        typename InputLocationToLatLngMapperT,
        typename LatLngToTargetVertexMapperT,
        typename TarVertexToOutputLocT>
class LocationMapper {


public:

    LocationMapper(InputLocationToLatLngMapperT &inputLocationToLatLngMapper,
                   LatLngToTargetVertexMapperT &latLngToTargetVertexMapper,
                   TarVertexToOutputLocT &tarVertexToOutputLoc)
            : inputLocationToLatLngMapper(inputLocationToLatLngMapper),
              latLngToTargetVertexMapper(latLngToTargetVertexMapper),
              tarVertexToOutputLoc(tarVertexToOutputLoc) {}

    bool mapLocation(const typename InputLocationToLatLngMapperT::InputType &inputLoc,
                    int &outLoc, const int tarLocToAvoid = INVALID_ID) {
        const LatLng &latLng = inputLocationToLatLngMapper(inputLoc);
        const int tarVertex = latLngToTargetVertexMapper.mapToTargetVertex(latLng);
        if (tarVertex == INVALID_VERTEX)
            return false;

        outLoc = tarVertexToOutputLoc(tarVertex, inputLoc, tarLocToAvoid);
        return true;
    }

private:

    InputLocationToLatLngMapperT &inputLocationToLatLngMapper;
    LatLngToTargetVertexMapperT &latLngToTargetVertexMapper;
    TarVertexToOutputLocT &tarVertexToOutputLoc;

};