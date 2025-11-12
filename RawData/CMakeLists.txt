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

add_executable(CategorizeRequestsByCoreAndPeriphery CategorizeRequestsByCoreAndPeriphery.cc)
target_compile_definitions(CategorizeRequestsByCoreAndPeriphery PRIVATE CSV_IO_NO_THREAD)
target_compile_options(CategorizeRequestsByCoreAndPeriphery PRIVATE ${FULL_WARNINGS})
target_link_libraries(CategorizeRequestsByCoreAndPeriphery kassert fast_cpp_csv_parser)

add_executable(ConvertGraph ConvertGraph.cc)
target_compile_definitions(ConvertGraph PRIVATE CSV_IO_NO_THREAD)
target_compile_options(ConvertGraph PRIVATE ${FULL_WARNINGS})
target_link_libraries(ConvertGraph proj routingkit kassert rapidxml fast_cpp_csv_parser)

add_executable(GenerateODPairs GenerateODPairs.cc)
target_compile_definitions(GenerateODPairs PRIVATE CSV_IO_NO_THREAD)
target_compile_options(GenerateODPairs PRIVATE ${FULL_WARNINGS})
target_link_libraries(GenerateODPairs kassert fast_cpp_csv_parser)
if(OpenMP_FOUND)
  target_link_libraries(GenerateODPairs OpenMP::OpenMP_CXX)
endif()

add_executable(GenerateRandomVehicles GenerateRandomVehicles.cc)
target_compile_definitions(GenerateRandomVehicles PRIVATE CSV_IO_NO_THREAD)
target_compile_options(GenerateRandomVehicles PRIVATE ${FULL_WARNINGS})
target_link_libraries(GenerateRandomVehicles kassert fast_cpp_csv_parser)

add_executable(DrawRandomDepartureTimes DrawRandomDepartureTimes.cc)
target_compile_definitions(DrawRandomDepartureTimes PRIVATE CSV_IO_NO_THREAD)
target_compile_options(DrawRandomDepartureTimes PRIVATE ${FULL_WARNINGS})
target_link_libraries(DrawRandomDepartureTimes fast_cpp_csv_parser)

add_executable(OsmToCarAndPassengerGraph OsmToCarAndPassengerGraph.cc)
target_compile_definitions(OsmToCarAndPassengerGraph PRIVATE CSV_IO_NO_THREAD)
target_compile_options(OsmToCarAndPassengerGraph PRIVATE ${FULL_WARNINGS})
target_link_libraries(OsmToCarAndPassengerGraph proj routingkit kassert fast_cpp_csv_parser)

add_executable(TransformLocations TransformLocations.cc)
target_compile_definitions(TransformLocations PRIVATE CSV_IO_NO_THREAD)
target_compile_options(TransformLocations PRIVATE ${FULL_WARNINGS})
target_link_libraries(TransformLocations proj routingkit kassert fast_cpp_csv_parser)

add_executable(CheckExistingAttributesInNetworkFile CheckExistingAttributesInNetworkFile.cc)
target_compile_definitions(CheckExistingAttributesInNetworkFile PRIVATE CSV_IO_NO_THREAD)
target_compile_options(CheckExistingAttributesInNetworkFile PRIVATE ${FULL_WARNINGS})
target_link_libraries(CheckExistingAttributesInNetworkFile kassert fast_cpp_csv_parser)

add_executable(GenerateGeoJsonForGraph GenerateGeoJsonForGraph.cc)
target_compile_definitions(GenerateGeoJsonForGraph PRIVATE CSV_IO_NO_THREAD)
target_compile_options(GenerateGeoJsonForGraph PRIVATE ${FULL_WARNINGS})
target_link_libraries(GenerateGeoJsonForGraph PRIVATE nlohmann_json kassert fast_cpp_csv_parser)

add_executable(GenerateGeoJsonForVehicles GenerateGeoJsonForVehicles.cc)
target_compile_definitions(GenerateGeoJsonForVehicles PRIVATE CSV_IO_NO_THREAD)
target_compile_options(GenerateGeoJsonForVehicles PRIVATE ${FULL_WARNINGS})
target_link_libraries(GenerateGeoJsonForVehicles PRIVATE nlohmann_json kassert fast_cpp_csv_parser)

add_executable(GenerateGeoJsonForRequests GenerateGeoJsonForRequests.cc)
target_compile_definitions(GenerateGeoJsonForRequests PRIVATE CSV_IO_NO_THREAD)
target_compile_options(GenerateGeoJsonForRequests PRIVATE ${FULL_WARNINGS})
target_link_libraries(GenerateGeoJsonForRequests PRIVATE nlohmann_json kassert fast_cpp_csv_parser)

add_executable(TransformRequestsToLatLng TransformRequestsToLatLng.cc)
target_compile_definitions(TransformRequestsToLatLng PRIVATE CSV_IO_NO_THREAD)
target_compile_options(TransformRequestsToLatLng PRIVATE ${FULL_WARNINGS})
target_link_libraries(TransformRequestsToLatLng kassert)