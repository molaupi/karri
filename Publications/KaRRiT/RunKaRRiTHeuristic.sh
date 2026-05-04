#!/bin/bash
# Runs KaRRiT in heuristic configuration.

# Read input arguments and set defaults if necessary:
baseDir=$1
instanceName=$2 # Known instance names: Berlin-1pct, Berlin-10pct, Ruhr-1pct, Ruhr-10pct
outputDir=$3 # Directory to write output to
numThreads=$4 # Number of threads to use, default = 1
passengerMode=$5 # Optional: mode of transportation of passenger (pedestrian or cyclist), default = pedestrian
radius=$6 # Optional: Walking radius, default = 0
waitTime=$7 # Optional: Maximum wait time, default = 600
maxNumPDLocs=$8 # Optional: Sample maxNumPDLocs pickups/dropoffs from the radius around the origin/destination
dependencyInstallDir=$9 # Optional: Path to dependencies if not on PATH variable

[ -z ${numThreads} ] && numThreads=1
[ -z ${passengerMode} ] && passengerMode=pedestrian
[ -z ${radius} ] && radius=0
[ -z ${waitTime} ] && waitTime=600
[ -z ${maxNumPDLocs} ] && maxNumPDLocs=0

inputDir=$baseDir/Inputs

# Build KaRRiT in default configuration:
parentPath=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $parentPath

sourceDir=./../..
binaryDir=$sourceDir/Build/Release/KaRRiTHeuristic/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}"\
	-DKARRI_PALS_STRATEGY=COL \
  -DKARRI_DALS_STRATEGY=COL \
  -DKARRI_OUTPUT_VEHICLE_PATHS=OFF \
  -DKARRI_ELLIPTIC_BCH_USE_SIMD=OFF \
  -DKARRI_ELLIPTIC_BCH_LOG_K=0 \
  -DKARRI_PD_DISTANCES_USE_SIMD=OFF \
  -DKARRI_PD_DISTANCES_LOG_K=0 \
  -DKARRI_PALS_USE_SIMD=OFF \
  -DKARRI_PALS_LOG_K=0 \
  -DKARRI_DALS_USE_SIMD=OFF \
  -DKARRI_DALS_LOG_K=0 \
  -DKARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD=ON \
  -DKARRI_TRANSFER_DIRECT_DISTANCES_LOG_K=3 \
  -DKARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE=ON \
  -DKARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD=ON \
  -DKARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K=3 \
  -DKARRI_TRANSFER_TALS_STRAT=PHAST \
  -DKARRI_TRANSFER_TALS_USE_SIMD=ON \
  -DKARRI_TRANSFER_TALS_LOG_K=3 \
  -DKARRI_TRANSFER_HEURISTIC_LEVEL=1 \
	-S $sourceDir -B $binaryDir
cmake --build $binaryDir --target karri


# Run KaRRiT:
name=${instanceName}_${passengerMode}
vehName=${name}_veh
psgName=${name}_psg

mkdir -p $outputDir/${name}

$binaryDir/Launchers/karri \
  -p-radius ${radius} \
  -d-radius ${radius} \
  -max-num-p ${maxNumPDLocs} \
  -max-num-d ${maxNumPDLocs} \
  -w ${waitTime} \
  -max-num-threads ${numThreads} \
  -always-veh \
  -veh-g $inputDir/Graphs/${vehName}.gr.bin \
  -psg-g $inputDir/Graphs/${psgName}.gr.bin \
  -v $inputDir/Vehicles/${name}.csv \
  -r $inputDir/Requests/${name}.csv \
  -veh-h $inputDir/CHs/${vehName}_time.ch.bin \
  -psg-h $inputDir/CHs/${psgName}_time.ch.bin \
  -o $outputDir/${name}/karrit-heuristic_${radius}_${waitTime}_${maxNumPDLocs}