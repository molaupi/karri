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

add_executable(AddStaTraversalCost AddStaTraversalCost.cc)
target_compile_definitions(AddStaTraversalCost PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(AddStaTraversalCost PRIVATE kassert ${RoutingKit_LIBRARY} kassert)

add_executable(BuildTrafficFlowBasedSubgraph BuildTrafficFlowBasedSubgraph.cc)
target_compile_definitions(BuildTrafficFlowBasedSubgraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(BuildTrafficFlowBasedSubgraph PRIVATE kassert ${RoutingKit_LIBRARY} kassert)

add_executable(CheckExistingAttributesInNetworkFile CheckExistingAttributesInNetworkFile.cc)
target_compile_definitions(CheckExistingAttributesInNetworkFile PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(CheckExistingAttributesInNetworkFile ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})

add_executable(ComputeFreeFlowFlows ComputeFreeFlowFlows.cc)
target_compile_definitions(ComputeFreeFlowFlows PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ComputeFreeFlowFlows ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})

add_executable(ConvertGraph ConvertGraph.cc)
target_compile_definitions(ConvertGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ConvertGraph ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})

add_executable(FlowsRoutingKitToCsv FlowsRoutingKitToCsv.cc)
target_compile_definitions(FlowsRoutingKitToCsv PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(FlowsRoutingKitToCsv ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})

add_executable(GenerateGeoJsonForFlows GenerateGeoJsonForFlows.cc)
target_compile_definitions(GenerateGeoJsonForFlows PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForFlows PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})

add_executable(GenerateGeoJsonForGraph GenerateGeoJsonForGraph.cc)
target_compile_definitions(GenerateGeoJsonForGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForGraph PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})

add_executable(GenerateGeoJsonForODPairs GenerateGeoJsonForODPairs.cc)
target_compile_definitions(GenerateGeoJsonForODPairs PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForODPairs PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})

add_executable(GenerateGeoJsonForVehicles GenerateGeoJsonForVehicles.cc)
target_compile_definitions(GenerateGeoJsonForVehicles PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForVehicles PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})

add_executable(GenerateODPairs GenerateODPairs.cc)
target_compile_definitions(GenerateODPairs PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateODPairs kassert ${BOOST_LIBRARIES})
if(OpenMP_FOUND)
  target_link_libraries(GenerateODPairs OpenMP::OpenMP_CXX)
endif()

add_executable(GenerateRandomVehicles GenerateRandomVehicles.cc)
target_compile_definitions(GenerateRandomVehicles PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateRandomVehicles ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert)

add_executable(DrawRandomDepartureTimes DrawRandomDepartureTimes.cc)
target_compile_definitions(DrawRandomDepartureTimes PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(DrawRandomDepartureTimes ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert)


add_executable(ODPairsCsvToRoutingKit ODPairsCsvToRoutingKit.cc)
target_compile_definitions(ODPairsCsvToRoutingKit PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ODPairsCsvToRoutingKit ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert)

add_executable(OsmToCarAndPassengerGraph OsmToCarAndPassengerGraph.cc)
target_compile_definitions(OsmToCarAndPassengerGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(OsmToCarAndPassengerGraph ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})

add_executable(TransformLocations TransformLocations.cc)
target_compile_definitions(TransformLocations PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(TransformLocations ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})
