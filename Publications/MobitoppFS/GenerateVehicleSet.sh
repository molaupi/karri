#!/bin/bash

# Construct vehicle set with random initial locations on KaRRi network 

karriNetwork=$1 # Path to KaRRi input network in binary format
karriSourceDir=$2 # Path to top-level directory of KaRRi
numVehicles=$3 # Number of vehicles to use in KaRRi fleet
capacity=$4 # Seating capacity of vehicles
startOfServiceTime=$5 # Start of service time of vehicles in tenths of seconds (e.g. 0 for 00:00:00 of day)
endOfServiceTime=$6 # End of service time of vehicles in tenths of seconds (e.g. 864000 for 24:00:00 of day, i.e. 00:00:00 of next day)
outputName=$7 # Path to output vehicles file

networkName=$(basename $karriNetwork .gr.bin)

# Build vehicle generation utility bundled with KaRRi
cmake -S ${karriSourceDir} -B ${karriSourceDir}/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build ${karriSourceDir}/Build/Release --target GenerateRandomVehicles

# Generate vehicles at random locations
${karriSourceDir}/Build/Release/RawData/GenerateRandomVehicles -g ${karriNetwork} -n ${numVehicles} -c ${capacity} -start ${startOfServiceTime} -end ${endOfServiceTime} \
	-o ${outputName}
