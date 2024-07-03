#!/bin/bash

inputDir=$1
fullNetworkName=$2            # Expects full vehicle and pedestrian networks at $inputDir/Graphs/${fullNetworkName}_{veh,psg}.gr.bin as well as inner boundary at $inputDir/Boundaries/${fullNetworkName}_Inner.poly
requestsFile=$3               # Path to original request file
dependencyInstallDir=${4}    # Optional: Path to headers of dependencies

veh_name=${fullNetworkName}_veh
psg_name=${fullNetworkName}_psg

parentPath=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
sourceDir=$parentPath/../..
binaryDir=$sourceDir/Build/Release

mkdir -p $inputDir/CHs $inputDir/Graphs $inputDir/ODPairs $inputDir/Requests $inputDir/Vehicles

# Build generators
cmake -D CMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}" -S $sourceDir -B $binaryDir
cmake --build $binaryDir --target GenerateRandomVehicles GenerateODPairs TransformLocations DrawRandomDepartureTimes RunP2PAlgo


# Generate synthetic request set mirroring input
reqBaseName=$(basename $requestsFile .csv)
cp $requestsFile $inputDir/Requests/${reqBaseName}.csv
requestsFile=$inputDir/Requests/${reqBaseName}
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${veh_name}.gr.bin -o $inputDir/CHs/${veh_name}_time
$binaryDir/RawData/TransformLocations -src-g $inputDir/Graphs/${veh_name}.gr.bin -tar-g $inputDir/Graphs/${veh_name}.gr.bin -p ${requestsFile}.csv -in-repr edge-id -out-repr vertex-id -o $inputDir/ODPairs/${reqBaseName}_as_vertices_OD.csv
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${veh_name}.gr.bin -h $inputDir/CHs/${veh_name}_time.ch.bin -d $inputDir/ODPairs/${reqBaseName}_as_vertices_OD.csv -o $inputDir/Requests/${reqBaseName}_direct_travel_times.csv
meanTripTimeInOriginalRequests=$(awk -F',' 'NR>1 {sum += $1; ++n} END {print sum/n}' $inputDir/Requests/${reqBaseName}_direct_travel_times.csv)
meanTripTimeInOriginalRequests=$(printf "%.0f\n" "$meanTripTimeInOriginalRequests")
echo "Mean trip time in original requests: $meanTripTimeInOriginalRequests"
numODPairs=$(wc -l < ${requestsFile}.csv)
let "numODPairs = $numODPairs - 1"
$binaryDir/RawData/GenerateODPairs -n $numODPairs -d $meanTripTimeInOriginalRequests -geom -psg -g $inputDir/Graphs/${veh_name}.gr.bin -a $inputDir/Boundaries/${fullNetworkName}_Inner.poly -o $inputDir/ODPairs/${reqBaseName}_synthetic_OD.csv
$binaryDir/RawData/TransformLocations -src-g $inputDir/Graphs/${veh_name}.gr.bin -tar-g $inputDir/Graphs/${veh_name}.gr.bin -psg -p $inputDir/ODPairs/${reqBaseName}_synthetic_OD.csv -in-repr vertex-id -out-repr edge-id -o $inputDir/ODPairs/${reqBaseName}_synthetic_OD.csv
$binaryDir/RawData/DrawRandomDepartureTimes -p $inputDir/ODPairs/${reqBaseName}_synthetic_OD.csv -t ${requestsFile}.csv -o $inputDir/Requests/${reqBaseName}_synthetic


# Generate synthetic vehicle sets
minDepTime=$(awk -F',' 'BEGIN { min=100000000 } NR>1 && $3 < min { min=$3} END { print min }' ${requestsFile}.csv)
maxDepTime=$(awk -F',' 'BEGIN { max=0 } NR>1 && $3 > max { max=$3} END { print max }' ${requestsFile}.csv)
startOfServiceTime=$minDepTime
let "endOfServiceTime=$maxDepTime + 3*$meanTripTimeInOriginalRequests/10" # vehicles operate up to 3*meanTripTime after last request (factor 10 for conversion from tenth of seconds to seconds)
echo $endOfServiceTime
veh_nums=(1000 2000 3000 4000)
for numVehicles in ${veh_nums[@]}; do
  $binaryDir/RawData/GenerateRandomVehicles -n $numVehicles -start $startOfServiceTime -end $endOfServiceTime -c 4 -g $inputDir/Graphs/${veh_name}.gr.bin -a $inputDir/Boundaries/${fullNetworkName}_Inner.poly -o $inputDir/Vehicles/${fullNetworkName}_${numVehicles}
done