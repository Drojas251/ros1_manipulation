#!/bin/bash

# Script to command pick place actions
# Input Args:
#   -o : object name to pick up (large, medium, small)
#   -l : location name to place (red, blue, green, yellow, purple)

while getopts ":o:l:" opt; do
  case $opt in
    o)
      object="$OPTARG"
      ;;
    l)
      location="$OPTARG"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

rosrun my_robot_functions_python pick_place_client.py object="$object" location="$location"