#!/bin/bash

for f in *.TXT; do
  python splitter.py $f
done


for f in *.nmea; do
  gpsbabel -i nmea -f $f -o gpx -F "${f%.nmea}.gpx"
done
