#!/bin/bash
# Generates KaRRi input data. Assumes that we know a request file with DRT queries from a MATSim simulation run that
# contains request times as well as origin and destination edges for a reference road network of the desired region.
# These requests (and a file containing a description of a fleet of vehicles) are transformed into query data on our OSM
# based input graph.

baseDir=$1                 # Expects .osm.pbf network files at $baseDir/Inputs/RawData/OSM/, Writes outputs to $baseDir/Inputs/
networkName=$2             # Name of network (Berlin)
instanceName=$3            # Name of instance (Berlin-1pct, Berlin-10pct)
knownRequestsFile=$4       # File containing known requests
knownVehiclesFile=$5       # File containing known vehicles
graphOfRefRequests=$6      # Road network in binary format on which the known requests were generated. Should geographically match the desired network.
passengerMode=$7           # Mode of transportation for passengers (pedestrian or cyclist)
dependencyInstallDir=$8    # Optional: Path to headers of dependencies

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

cmake -D CMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}" -S $sourceDir -B $binaryDir
cmake --build $binaryDir --target OsmToCarAndPassengerGraph TransformLocations RunP2PAlgo

#########################
# Setting Up Input Data #
#########################

name=${instanceName}_${passengerMode}
veh_name=${name}_veh
psg_name=${name}_psg

# For a more precise mapping of edges in the vehicle and passenger networks, remove -no-union-nodes flag.
# Leads to larger vehicle network with many vertices of degree 2, though.
$binaryDir/RawData/OsmToCarAndPassengerGraph -psg-mode ${passengerMode} -no-union-nodes -a lat_lng osm_node_id capacity free_flow_speed length num_lanes osm_road_category road_geometry speed_limit travel_time -i $inputGraph -co $inputDir/Graphs/${veh_name} -po $inputDir/Graphs/${psg_name}
$binaryDir/RawData/TransformLocations -src-g $graphOfRefRequests -tar-g $inputDir/Graphs/${veh_name}.gr.bin -psg -v $knownVehiclesFile -l-col-name initial_location -in-repr edge-id -out-repr edge-id -o $inputDir/Vehicles/${name}
python3 $sourceDir/RawData/merge_csv_files.py -i1 $inputDir/Vehicles/${name}.csv -c1 initial_location -i2 $knownVehiclesFile -c2 start_service_time end_service_time seating_capacity -co2 start_of_service_time end_of_service_time capacity -o $inputDir/Vehicles/${name}.csv
$binaryDir/RawData/TransformLocations -src-g $graphOfRefRequests -tar-g $inputDir/Graphs/${veh_name}.gr.bin -psg -p $knownRequestsFile -o-col-name pickup_spot -d-col-name dropoff_spot -in-repr edge-id -out-repr edge-id -o $inputDir/ODPairs/${name}_OD.csv
python3 $sourceDir/RawData/merge_csv_files.py -i1 $inputDir/ODPairs/${name}_OD.csv -c1 origin destination -i2 $knownRequestsFile -c2 min_dep_time -co2 req_time -o $inputDir/Requests/${name}.csv
#rm $inputDir/ODPairs/${name}_OD.csv

###########################
# Preprocessing the Graph #
###########################

echo "Constructing CHs..."
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${veh_name}.gr.bin -o $inputDir/CHs/${veh_name}_time
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${psg_name}.gr.bin -o $inputDir/CHs/${psg_name}_time
echo "done."
