# ******************************************************************************
# MIT License
#
# Copyright (c) 2020 Valentin Buchhold
# Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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


add_executable(ConvertGraph ConvertGraph.cc)
target_compile_definitions(ConvertGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ConvertGraph ${PROJ_LIBRARY} ${RoutingKit_LIBRARY})

add_executable(GenerateODPairs GenerateODPairs.cc)
target_compile_definitions(GenerateODPairs PRIVATE CSV_IO_NO_THREAD)
if(OpenMP_FOUND)
  target_link_libraries(GenerateODPairs OpenMP::OpenMP_CXX)
endif()


add_executable(GenerateRandomVehicles GenerateRandomVehicles.cc)
target_compile_definitions(GenerateRandomVehicles PRIVATE CSV_IO_NO_THREAD)

add_executable(DrawRandomDepartureTimes DrawRandomDepartureTimes.cc)
target_compile_definitions(DrawRandomDepartureTimes PRIVATE CSV_IO_NO_THREAD)

add_executable(OsmToCarAndPassengerGraph OsmToCarAndPassengerGraph.cc)
target_compile_definitions(OsmToCarAndPassengerGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(OsmToCarAndPassengerGraph ${PROJ_LIBRARY} ${RoutingKit_LIBRARY})

add_executable(TransformLocations TransformLocations.cc)
target_compile_definitions(TransformLocations PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(TransformLocations ${PROJ_LIBRARY} ${RoutingKit_LIBRARY})


add_executable(CheckExistingAttributesInNetworkFile CheckExistingAttributesInNetworkFile.cc
        ../Algorithms/KaRRi/BaseObjects/StopInfo.h
        ../Algorithms/KaRRi/RouteState/ScheduledStop.h
        ../Algorithms/KaRRi/RouteState/RouteStateData.h
        ../Algorithms/KaRRi/RouteState/RouteStateUpdater.h
        ../Algorithms/KaRRi/RouteState/FixedRouteStateUpdater.h
        ../Algorithms/KaRRi/EllipticBCH/EllipticBucketsUpdater.h
        ../Algorithms/KaRRi/RouteState/RouteStateDataType.h
        ../Algorithms/KaRRi/LastStopSearches/LastStopBucketsUpdater.h
        ../Algorithms/KaRRi/EllipticBCH/BucketsWrapper.h
        ../Algorithms/KaRRi/AssignmentManager.h
)
target_compile_definitions(CheckExistingAttributesInNetworkFile PRIVATE CSV_IO_NO_THREAD)

add_executable(GenerateGeoJsonForGraph GenerateGeoJsonForGraph.cc)
target_compile_definitions(GenerateGeoJsonForGraph PRIVATE CSV_IO_NO_THREAD)

add_executable(GenerateGeoJsonForVehicles GenerateGeoJsonForVehicles.cc)
target_compile_definitions(GenerateGeoJsonForVehicles PRIVATE CSV_IO_NO_THREAD)


add_executable(GenerateGeoJsonForRequests GenerateGeoJsonForRequests.cc)
target_compile_definitions(GenerateGeoJsonForRequests PRIVATE CSV_IO_NO_THREAD)

