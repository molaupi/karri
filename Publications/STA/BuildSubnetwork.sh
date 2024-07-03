#!/bin/bash

inputDir=$1
fullNetworkName=$2            # Expects full vehicle and pedestrian networks at $inputDir/Graphs/${fullNetworkName}_{veh,psg}.gr.bin as well as inner boundary at $inputDir/Boundaries/${fullNetworkName}_Inner.poly
flowFile=$3
radius=$4
flowFac=$5
dependencyInstallDir=${6}    # Optional: Path to headers of dependencies

veh_name=${fullNetworkName}_veh
psg_name=${fullNetworkName}_psg

parentPath=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
sourceDir=$parentPath/../..
binaryDir=$sourceDir/Build/Release

mkdir -p $inputDir/CHs $inputDir/Graphs $inputDir/ODPairs $inputDir/Requests $inputDir/Vehicles

# Build generators
cmake -D CMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}" -S $sourceDir -B $binaryDir
cmake --build $binaryDir --target BuildTrafficFlowBasedSubgraph

# Build subnetwork based on traffic flow
flowBaseName=$(basename $flowFile .csv)
$binaryDir/RawData/BuildTrafficFlowBasedSubgraph \
  -veh-g $inputDir/Graphs/${veh_name}.gr.bin \
  -psg-g $inputDir/Graphs/${psg_name}.gr.bin \
  -f $flowFile\
  -rho $radius \
  -flow-fac $flowFac \
  -o $inputDir/Graphs/${fullNetworkName}_${flowBaseName}_fac${flowFac}_r${radius}
