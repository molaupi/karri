#!/bin/bash
# Runs KaRRi in default configuration (using collective last stop searches, sorted buckets, and SIMD instructions).

# Read input arguments and set defaults if necessary:
baseDir=$1
instanceName=$2 # Known instance names: Berlin-1pct, Berlin-10pct, Ruhr-1pct, Ruhr-10pct
outputDir=$3 # Directory to write output to
passengerMode=$4 # Optional: mode of transportation of passenger (pedestrian or cyclist), default = pedestrian
radius=$5 # Optional: Walking radius, default = 300
waitTime=$6 # Optional: Maximum wait time, default = 300
dependencyInstallDir=$7 # Optional: Path to dependencies if not on PATH variable

[ -z ${passengerMode} ] && passengerMode=pedestrian
[ -z ${radius} ] && radius=300
[ -z ${waitTime} ] && waitTime=300

inputDir=$baseDir/Inputs

# Build KaRRi in default configuration:
parentPath=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $parentPath

sourceDir=./../..
binaryDir=$sourceDir/Build/Release/COL_SIMD
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}"\
	-DKARRI_ELLIPTIC_BCH_USE_SIMD=ON -DKARRI_ELLIPTIC_BCH_LOG_K=4 \
	-DKARRI_PALS_STRATEGY=COL -DKARRI_PALS_USE_SIMD=ON -DKARRI_PALS_LOG_K=4 \
	-DKARRI_DALS_STRATEGY=COL -DKARRI_DALS_USE_SIMD=ON -DKARRI_DALS_LOG_K=4 \
	-DKARRI_PD_DISTANCES_USE_SIMD=ON -DKARRI_PD_DISTANCES_LOG_K=4 \
	-DKARRI_PSG_COST_SCALE=1 \
	-DKARRI_VEH_COST_SCALE=1 \
	-S $sourceDir -B $binaryDir
cmake --build $binaryDir --target karri


# Run KaRRi:
name=${instanceName}_${passengerMode}
vehName=${name}_veh
psgName=${name}_psg

mkdir -p $outputDir/${name}

$binaryDir/Launchers/karri \
  -p-radius ${radius} \
  -d-radius ${radius} \
  -w ${waitTime} \
  -veh-g $inputDir/Graphs/${vehName}.gr.bin \
  -psg-g $inputDir/Graphs/${psgName}.gr.bin \
  -v $inputDir/Vehicles/${name}.csv \
  -r $inputDir/Requests/${name}.csv \
  -veh-h $inputDir/CHs/${vehName}_time.ch.bin \
  -psg-h $inputDir/CHs/${psgName}_time.ch.bin \
  -o $outputDir/${name}/karri-col-simd_${radius}_${waitTime}