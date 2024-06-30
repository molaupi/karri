#!/bin/bash

inputDir=$1
fullNetworkName=$2            # Expects full vehicle network at $inputDir/Graphs/${fullNetworkName}_{veh,psg}.gr.bin as well as vehicles file at $inputDir/Vehicles/${fullNetworkName}_${numVehicles}.csv
subNetworkName=$3
numVehicles=$4

fullNetwork=$inputDir/Graphs/${fullNetworkName}_veh.gr.bin
subNetwork=$inputDir/Graphs/${subNetworkName}.veh_subnetwork.gr.bin
vehicles=$inputDir/Vehicles/${fullNetworkName}_${numVehicles}.csv

parentPath=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
sourceDir=$parentPath/../..
binaryDir=$sourceDir/Build/Release

# Build generators
cmake -D CMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}" -S $sourceDir -B $binaryDir
cmake --build $binaryDir --target TransformLocations

# Map initial locations of vehicles to closest locations in subnetwork
$binaryDir/RawData/TransformLocations -src-g $fullNetwork -tar-g $subNetwork -v $vehicles -in-repr edge-id -out-repr edge-id -o ${inputDir}/Vehicles/${subNetworkName}_${numVehicles}_locs.csv

# Add other vehicle properties to the new vehicle file
python3 $sourceDir/RawData/merge_csv_files.py  -i1 ${inputDir}/Vehicles/${subNetworkName}_${numVehicles}_locs.csv -c1 initial_location -i2 ${vehicles} -c2 start_of_service_time end_of_service_time capacity -o ${inputDir}/Vehicles/${subNetworkName}_${numVehicles}.csv
rm ${inputDir}/Vehicles/${subNetworkName}_${numVehicles}_locs.csv
rm ${inputDir}/Vehicles/${subNetworkName}_${numVehicles}_locs.vertexmatches.csv