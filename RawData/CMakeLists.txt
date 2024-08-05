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


add_executable(AddSingleEdgeByOsmNodeIds AddSingleEdgeByOsmNodeIds.cc)
target_compile_definitions(AddSingleEdgeByOsmNodeIds PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(AddSingleEdgeByOsmNodeIds kassert ${BOOST_LIBRARIES})

add_executable(AddStaTraversalCost AddStaTraversalCost.cc)
target_compile_definitions(AddStaTraversalCost PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(AddStaTraversalCost kassert ${BOOST_LIBRARIES})

add_executable(ComputeFreeFlowPaths ComputeFreeFlowPaths.cc)
target_compile_definitions(ComputeFreeFlowPaths PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ComputeFreeFlowPaths kassert ${RoutingKit_LIBRARY} ${BOOST_LIBRARIES})
target_include_directories(ComputeFreeFlowPaths SYSTEM PRIVATE ${RoutingKit_INCLUDE_DIR})


add_executable(ComputePDLocsForRequests ComputePDLocsForRequests.cc)
target_compile_definitions(ComputePDLocsForRequests PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ComputePDLocsForRequests kassert ${RoutingKit_LIBRARY} ${BOOST_LIBRARIES})
target_include_directories(ComputePDLocsForRequests SYSTEM PRIVATE ${RoutingKit_INCLUDE_DIR})

add_executable(ConvertGraph ConvertGraph.cc)
target_compile_definitions(ConvertGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ConvertGraph ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})
target_include_directories(ConvertGraph SYSTEM PRIVATE ${RoutingKit_INCLUDE_DIR})

add_executable(FlowsRoutingKitToCsv FlowsRoutingKitToCsv.cc)
target_compile_definitions(FlowsRoutingKitToCsv PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(FlowsRoutingKitToCsv ${PROJ_LIBRARY} ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})
target_include_directories(FlowsRoutingKitToCsv SYSTEM PRIVATE ${RoutingKit_INCLUDE_DIR})

add_executable(GenerateODPairs GenerateODPairs.cc)
target_compile_definitions(GenerateODPairs PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateODPairs kassert ${BOOST_LIBRARIES})
if(OpenMP_FOUND)
  target_link_libraries(GenerateODPairs OpenMP::OpenMP_CXX)
endif()


add_executable(GenerateRandomVehicles GenerateRandomVehicles.cc)
target_compile_definitions(GenerateRandomVehicles PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateRandomVehicles kassert  ${BOOST_LIBRARIES})

add_executable(DrawRandomDepartureTimes DrawRandomDepartureTimes.cc)
target_compile_definitions(DrawRandomDepartureTimes PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(DrawRandomDepartureTimes kassert  ${BOOST_LIBRARIES})

add_executable(ODPairsCsvToRoutingKit ODPairsCsvToRoutingKit.cc)
target_compile_definitions(ODPairsCsvToRoutingKit PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ODPairsCsvToRoutingKit ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})
target_include_directories(ODPairsCsvToRoutingKit SYSTEM PRIVATE ${RoutingKit_INCLUDE_DIR})

add_executable(ODPairsRoutingKitToCsv ODPairsRoutingKitToCsv.cc)
target_compile_definitions(ODPairsRoutingKitToCsv PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(ODPairsRoutingKitToCsv ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})
target_include_directories(ODPairsRoutingKitToCsv SYSTEM PRIVATE ${RoutingKit_INCLUDE_DIR})

add_executable(OsmToCarAndPassengerGraph OsmToCarAndPassengerGraph.cc)
target_compile_definitions(OsmToCarAndPassengerGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(OsmToCarAndPassengerGraph ${RoutingKit_LIBRARY} kassert ${BOOST_LIBRARIES})
target_include_directories(OsmToCarAndPassengerGraph SYSTEM PRIVATE ${RoutingKit_INCLUDE_DIR})

add_executable(RequestsToODPairs RequestsToODPairs.cc)
target_compile_definitions(RequestsToODPairs PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(RequestsToODPairs kassert ${BOOST_LIBRARIES})

add_executable(TransformLocations TransformLocations.cc)
target_compile_definitions(TransformLocations PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(TransformLocations ${PROJ_LIBRARY} kassert ${BOOST_LIBRARIES})


add_executable(CheckExistingAttributesInNetworkFile CheckExistingAttributesInNetworkFile.cc)
target_compile_definitions(CheckExistingAttributesInNetworkFile PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(CheckExistingAttributesInNetworkFile kassert  ${BOOST_LIBRARIES})

add_executable(GenerateGeoJsonForGraph GenerateGeoJsonForGraph.cc)
target_compile_definitions(GenerateGeoJsonForGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForGraph PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})

add_executable(GenerateGeoJsonForVehicles GenerateGeoJsonForVehicles.cc)
target_compile_definitions(GenerateGeoJsonForVehicles PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForVehicles PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})


add_executable(GenerateGeoJsonForRequests GenerateGeoJsonForRequests.cc)
target_compile_definitions(GenerateGeoJsonForRequests PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForRequests PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})


add_executable(GenerateGeoJsonForPaths GenerateGeoJsonForPaths.cc)
target_compile_definitions(GenerateGeoJsonForPaths PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(GenerateGeoJsonForPaths PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})

add_executable(FielbaumToKaRRiInputGraph FielbaumToKaRRiInputGraph.cc)
target_compile_definitions(FielbaumToKaRRiInputGraph PRIVATE CSV_IO_NO_THREAD)
target_link_libraries(FielbaumToKaRRiInputGraph PRIVATE nlohmann_json::nlohmann_json kassert ${BOOST_LIBRARIES})
