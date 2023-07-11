#!/bin/bash
# Generates KaRRi input data. Assumes that we know a request file with DRT queries from a MobiTopp simulation run that
# contains request times as well as origin and destination locations ((given as EPSG-31467 coordinates) in the desired
# region. These requests are transformed into query data on our OSM based input graph. Vehicles are generated with
# random initial locations.
#
# Stuttgart-mobiTopp data is based on proprietary data so request file and boundaries files cannot be published.
# For an open source alternative for Stuttgart, GenerateInputDataGeneric.sh may be used with OSM relations as boundaries
# and the request files from the OpenBerlin scenario as reference files.

baseDir=$1                 # Expects .osm.pbf network files at $baseDir/Inputs/RawData/OSM/, Writes outputs to $baseDir/Inputs/
networkName=$2             # Name of network (Stuttgart)
boundariesDir=$3           # Expects boundary files for ${networkName}_Inner.poly at $boundariesDir/
instanceName=$4            # Name of instance (Stuttgart-mobiTopp)
numVehicles=$5             # Default Value: 500
knownRequestsFile=$6       # File containing known requests with locFrom, locTo and req_time columns
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

#########################
#  Building Generators  #
#########################

cmake -D CMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}" -S $sourceDir -B $binaryDir
cmake --build $binaryDir --target OsmToCarAndPassengerGraph GenerateRandomVehicles TransformLocations RunP2PAlgo

#########################
# Setting Up Input Data #
#########################

name=${instanceName}_${passengerMode}
veh_name=${name}_veh
psg_name=${name}_psg

# For a more precise mapping of edges in the vehicle and passenger networks, remove -no-union-nodes flag.
# Leads to larger vehicle network with many vertices of degree 2, though.
$binaryDir/RawData/OsmToCarAndPassengerGraph -psg-mode ${passengerMode} -no-union-nodes -no-veh-on-service -a lat_lng osm_node_id capacity free_flow_speed length num_lanes osm_road_category road_geometry speed_limit travel_time -i $inputGraph -co $inputDir/Graphs/${veh_name} -po $inputDir/Graphs/${psg_name}
$binaryDir/RawData/GenerateRandomVehicles -n $numVehicles -g $inputDir/Graphs/${veh_name}.gr.bin -a $boundariesDir/${networkName}_Inner.poly -start 0 -end 100000 -o $inputDir/Vehicles/${name}
$binaryDir/RawData/TransformLocations -tar-g $inputDir/Graphs/${veh_name}.gr.bin -psg -p $knownRequestsFile -o-col-name locFrom -d-col-name locTo -in-repr epsg-31467 -out-repr edge-id -o $inputDir/ODPairs/${name}_OD.csv
python3 $sourceDir/RawData/merge_csv_files.py -i1 $inputDir/ODPairs/${name}_OD.csv -c1 origin destination -i2 $knownRequestsFile -c2 req_time -co2 req_time -o $inputDir/Requests/${name}.csv
#rm $inputDir/ODPairs/${name}_OD.csv

###########################
# Preprocessing the Graph #
###########################

echo "Constructing CHs..."
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${veh_name}.gr.bin -o $inputDir/CHs/${veh_name}_time
$binaryDir/Launchers/RunP2PAlgo -a CH -g $inputDir/Graphs/${psg_name}.gr.bin -o $inputDir/CHs/${psg_name}_time
echo "done."
