#!/bin/bash
# Filters OSM data for road network of Germany and maintains only required roads.
baseDir=$1
osmium=$2 # Path to osmium tool (if not given, will expect that osmium is installed and callable anywhere)

osmDir=$baseDir/Inputs/RawData/OSM

if [ -z ${osmium} ]; then
  osmium=osmium # If user did not specify location of osmium executable, assume it is installed on default PATH
fi

cd $osmDir

$osmium tags-filter -o Germany_Highways.osm.pbf Germany_Complete.osm.pbf w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link,unclassified,residential,living_street,service,pedestrian,track,footway,bridleway,cycleway,steps,path #,corridor
$osmium sort -o Germany_Highways_Sorted.osm.pbf Germany_Highways.osm.pbf
mv -f Germany_Highways_Sorted.osm.pbf Germany_Highways.osm.pbf
#osmium tags-filter -o Germany_MainHighways.osm.pbf Germany_Highways.osm.pbf w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link
