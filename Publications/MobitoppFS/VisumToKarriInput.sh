#!/bin/bash

# Transform a Visum .net network into a vehicle network and a pedestrian network in KaRRi input format.

visumNetwork=$1 # Path to input network in Visum format
visumMode=$2 # Mode for vehicles (e.g. CAR)
visumCoordinateSystemCode=$3 # EPSG code of global coordinate system used in Visum network (e.g. 4326 for WGS84) 
visumCoordinatePrecision=$4 # Precision of coordinate values in Visum network (converter multiplies coordinate values with this number to obtain 32-bit integers; should not exceed 10'000'000)
karriSourceDir=$5 # Path to top-level directory of KaRRi
outputDir=$6 # Path to write KaRRi input files to

networkName=$(basename $visumNetwork .net)

# Convert Visum .net file into individual CSV files
visumDir=$(dirname ${visumNetwork})
separateVisumFilesDir=${visumDir}/SeparateVisumFiles/
mkdir -p ${separateVisumFilesDir}

python3 split_visum_into_separate_files.py -i ${visumNetwork} -o ${separateVisumFilesDir}

# Build network conversion utilities bundled with KaRRi
cmake -S ${karriSourceDir} -B ${karriSourceDir}/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build ${karriSourceDir}/Build/Release --target ConvertGraph GenerateIdenticalPedestrianGraph GenerateRandomVehicles

# Convert graph from Visum to KaRRi vehicle network
${karriSourceDir}/Build/Release/RawData/ConvertGraph -s visum -d binary -scc -ts ${visumMode} -cs ${visumCoordinateSystemCode} -cp ${visumCoordinatePrecision} -ap 1 \
	-a coordinate edge_id free_flow_speed lat_lng length mobitopp_link_id road_geometry speed_limit travel_time vertex_id \
	-i ${separateVisumFilesDir} -o ${outputDir}/${networkName}

# Generate KaRRi pedestrian network which is topologically identical but uses fixed walking speed of 4.5km/h for travel times
${karriSourceDir}/Build/Release/RawData/GenerateIdenticalPedestrianGraph -g ${outputDir}/${networkName}.gr.bin -s 4.5 \
	-a coordinate edge_id free_flow_speed lat_lng length mobitopp_link_id road_geometry speed_limit travel_time vertex_id \
	-o ${outputDir}
