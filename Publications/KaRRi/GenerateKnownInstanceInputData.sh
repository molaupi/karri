#!/bin/bash
# Generates KaRRi input data.

baseDir=$1
instanceName=$2 # Known instance names: Berlin-1pct, Berlin-10pct, Ruhr-1pct, Ruhr-10pct
passengerMode=$3 # Possible modes: pedestrian, cyclist
dependencyInstallDir=$4

parentPath=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $parentPath

if [ "${instanceName}" = "Berlin-1pct" ]; then
  echo "Generating input data for Berlin-1pct instance."
  sh GenerateInputDataByTransformingMATSimData.sh $baseDir Berlin Berlin-1pct ./MATSim_Output/Requests_MATSim_Berlin-1pct.csv ./MATSim_Output/Vehicles_MATSim_Berlin-1pct.csv ./MATSim_Output/MATSim_Berlin.gr.bin $passengerMode $dependencyInstallDir
  exit 0
fi

if [ "${instanceName}" = "Berlin-10pct" ]; then
  echo "Generating input data for Berlin-10pct instance."
  sh GenerateInputDataByTransformingMATSimData.sh $baseDir Berlin Berlin-10pct ./MATSim_Output/Requests_MATSim_Berlin-10pct.csv ./MATSim_Output/Vehicles_MATSim_Berlin-10pct.csv ./MATSim_Output/MATSim_Berlin.gr.bin $passengerMode $dependencyInstallDir
  exit 0
fi

if [ "${instanceName}" = "Ruhr-1pct" ]; then
  echo "Generating input data for Ruhr-1pct instance."
  sh GenerateInputDataGeneric.sh $baseDir Ruhr ./BoundaryPolygons/ Ruhr-1pct 3000 49707 7200 ./MATSim_Output/Requests_MATSim_Berlin-1pct.csv min_dep_time $passengerMode $dependencyInstallDir
  exit 0
fi

if [ "${instanceName}" = "Ruhr-10pct" ]; then
  echo "Generating input data for Ruhr-10pct instance."
  sh GenerateInputDataGeneric.sh $baseDir Ruhr ./BoundaryPolygons/ Ruhr-10pct 30000 447555 6600 ./MATSim_Output/Requests_MATSim_Berlin-10pct.csv min_dep_time $passengerMode $dependencyInstallDir
  exit 0
fi

echo "Instance \'${instanceName}\' is not known. Known instances are Berlin-1pct, Berlin-10pct, Ruhr-1pct, and Ruhr-10pct."
exit 1
