#!/bin/bash
# Filters OSM data for road network of given network and maintains only required roads.
baseDir=$1
network=$2
osmium=$3 # Path to osmium tool (if not given, will expect that osmium is installed and callable anywhere)

osmDir=$baseDir/Inputs/RawData/OSM

if [ -z ${osmium} ]; then
  osmium=osmium # If user did not specify location of osmium executable, assume it is installed on default PATH
fi

cd $osmDir

# Network containing all roads, streets and pathways accessible to motor traffic and/or pedestrian traffic
$osmium tags-filter -o ${network}_Highways.osm.pbf ${network}_Complete.osm.pbf w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link,unclassified,residential,living_street,service,pedestrian,track,footway,bridleway,cycleway,steps,path #,corridor
$osmium sort -o ${network}_Highways_Sorted.osm.pbf ${network}_Highways.osm.pbf
mv -f ${network}_Highways_Sorted.osm.pbf ${network}_Highways.osm.pbf

# Network containing only larger roads meant for through traffic
$osmium tags-filter -o ${network}_MainHighways.osm.pbf ${network}_Highways.osm.pbf w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link