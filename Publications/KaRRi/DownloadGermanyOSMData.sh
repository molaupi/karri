#!/bin/bash
# Downloads entire road network of Germany ( >= 4 GiB) in OSM format.
baseDir=$1

osmDir=$baseDir/Inputs/RawData/OSM

mkdir -p $osmDir
wget -O $osmDir/Germany_Complete.osm.pbf https://download.geofabrik.de/europe/germany-latest.osm.pbf