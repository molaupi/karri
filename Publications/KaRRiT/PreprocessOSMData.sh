#!/bin/bash
# Extracts OSM data for smaller network from larger surrounding network

baseDir=$1 # Base dir
surroundingNetworkName=$2 # Name of surrounding network (Germany, Europe, ...). Expects file ${baseDir}/Inputs/RawData/OSM/${surroundingNetworkName}_Highways.osm.pbf
networkName=$3 # Name of the smaller network
boundariesDir=$4 # Expects boundary files for ${networkName}_Inner.poly and ${networkName}_Outer.poly at $boundariesDir/
osmium=$5 # Path to osmium tool (if not given, will expect that osmium is installed and callable anywhere)

if [ -z ${osmium} ]; then
	osmium=osmium # If user did not specify location of osmium executable, assume it is installed on default PATH
fi

osmDir=${baseDir}/Inputs/RawData/OSM

# For inner area take all roads, streets and pedestrian pathways
$osmium extract -p $boundariesDir/${networkName}_Inner.poly -o $osmDir/${networkName}_Inner.osm.pbf $osmDir/${surroundingNetworkName}_Highways.osm.pbf

# For outer area take only major roads
$osmium extract -p $boundariesDir/${networkName}_Outer.poly -o $osmDir/${networkName}_Outer.osm.pbf $osmDir/${surroundingNetworkName}_MainHighways.osm.pbf

# Merge to a shared network
$osmium merge -o $osmDir/${networkName}.osm.pbf $osmDir/${networkName}_Inner.osm.pbf $osmDir/${networkName}_Outer.osm.pbf
rm $osmDir/${networkName}_Inner.osm.pbf $osmDir/${networkName}_Outer.osm.pbf