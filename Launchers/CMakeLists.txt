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

# Executable for P2P Algorithms
add_executable(RunP2PAlgo RunP2PAlgo.cc)
target_compile_definitions(RunP2PAlgo PRIVATE CSV_IO_NO_THREAD)
target_compile_options(RunP2PAlgo PRIVATE ${FULL_WARNINGS})
target_link_libraries(RunP2PAlgo routingkit kassert fast_cpp_csv_parser)
if(OpenMP_FOUND)
    target_link_libraries(RunP2PAlgo OpenMP::OpenMP_CXX)
endif()


# Executable for KaRRi Dynamic Taxi Sharing Simulation
add_executable(karri KaRRi.cpp)
target_compile_definitions(karri PRIVATE CSV_IO_NO_THREAD)
target_compile_options(karri PRIVATE ${FULL_WARNINGS})
target_link_libraries(karri routingkit kassert fast_cpp_csv_parser vectorclass hwloc TBB::tbb TBB::tbbmalloc)
if(OpenMP_FOUND)
    target_link_libraries(karri OpenMP::OpenMP_CXX)
endif()

# include KaRRi compile time parameters
include(KaRRi_compile_time_parameters.cmake)