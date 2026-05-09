#!/bin/bash
# Downloads entire road network of Europe (>= 25 GiB!) in OSM format.
baseDir=$1

osmDir=$baseDir/Inputs/RawData/OSM

mkdir -p $osmDir
cd $osmDir

wget -O Europe_Complete.osm.pbf https://download.geofabrik.de/europe-latest.osm.pbf