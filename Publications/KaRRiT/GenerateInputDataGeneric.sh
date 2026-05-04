#!/bin/bash
# Generates KaRRi input data for any region for which we know an underlying road network in OSM format.
# Randomly generates OD-pairs (geometrically distributed OD-distances) and initial locations of vehicles (uniformly distributed).
# Request times are drawn randomly according to a reference distribution that is additionally given.

baseDir=$1                    # Expects .osm.pbf network files at $baseDir/Inputs/RawData/OSM/, Writes outputs to $baseDir/Inputs/
networkName=$2                # Name of network (Ruhr)
boundariesDir=$3              # Expects boundary files for ${networkName}_Inner.poly at $boundariesDir/
instanceName=$4               # Name of instance (Ruhr-1pct, Ruhr-10pct)
numVehicles=$5                # Default Values: Ruhr-1pct: 3000, Ruhr-10pct: 30000
numRequests=$6                # Default Values: Ruhr-1pct: 49707, Ruhr-10pct: 447555
meanTripTime=$7               # Mean of geometric distribution of OD-distances in tenths of seconds. Default Values: Ruhr-1pct: 7200, Ruhr-10pct: 6600
refRequestFile=$8             # File containing reference distribution of request times
refRequestFileTimeColName=$9  # Name of column that contains times in reference file
passengerMode=${10}           # Mode of transportation for passengers (pedestrian or cyclist)
dependencyInstallDir=${11}    # Optional: Path to headers of dependencies

if [ "${passengerMode}" != "pedestrian" ] && [ "${passengerMode}" != "cyclist" ]; then
  echo "Passenger mode \"${passengerMode}\" is not known. Possible values: \"pedestrian\", \"cyclist\""
  exit 1
fi

inputDir=$baseDir/Inputs
inputGraph=$inputDir/RawData/OSM/$networkName

parentPath=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
sourceDir=$parentPath/../..
binaryDir=$sourceDir/Build/Release

mkdir -p $inputDir/CHs $inputDir/Graphs $inputDir/ODPairs $inputDir/Requests $inputDir/Vehicles

#########################
#  Building Generators  #
#########################

cmake -D CMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}" -S $sourceDir -B $binaryDir
cmake --build $binaryDir --target OsmToCarAndPassengerGraph GenerateRandomVehicles GenerateODPairs TransformLocations DrawRandomDepartureTimes RunP2PAlgo

#########################
# Setting Up Input Data #
#########################

name=${instanceName}_${passengerMode}
veh_name=${name}_veh
psg_name=${name}_psg

# For a more precise mapping of edges in the vehicle and passenger networks, remove -no-union-nodes flag.
# Leads to larger vehicle network with many vertices of degree 2, though.
$binaryDir/RawData/OsmToCarAndPassengerGraph -psg-mode ${passengerMode} -no-union-nodes -no-veh-on-service -a lat_lng osm_node_id capacity free_flow_speed length num_lanes osm_road_category road_geometry speed_limit travel_time -i $inputGraph -co $inputDir/Graphs/${veh_name} -po $inputDir/Graphs/${psg_name}
$binaryDir/RawData/GenerateRandomVehicles -n $numVehicles -g $inputDir/Graphs/${veh_name}.gr.bin -a $boundariesDir/${networkName}_Inner.poly -o $inputDir/Vehicles/${name}
$binaryDir/RawData/GenerateODPairs -n $numRequests -d $meanTripTime -geom -psg -g $inputDir/Graphs/${veh_name}.gr.bin -a $boundariesDir/${networkName}_Inner.poly -o $inputDir/ODPairs/${name}_OD.csv
$binaryDir/RawData/TransformLocations -src-g $inputDir/Graphs/${veh_name}.gr.bin -tar-g $inputDir/Graphs/${veh_name}.gr.bin -psg -p $inputDir/ODPairs/${name}_OD.csv -in-repr vertex-id -out-repr edge-id -o $inputDir/ODPairs/${name}_OD.csv
$binaryDir/RawData/DrawRandomDepartureTimes -p $inputDir/ODPairs/${name}_OD.csv -t $refRequestFile -t-col-name ${refRequestFileTimeColName} -o $inputDir/Requests/${name}
#rm $inputDir/ODPairs/${name}_OD.csv

###########################
# Preprocessing the Graph #
###########################

echo "Constructing CHs..."
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${veh_name}.gr.bin -o $inputDir/CHs/${veh_name}_time
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${psg_name}.gr.bin -o $inputDir/CHs/${psg_name}_time
echo "done."
