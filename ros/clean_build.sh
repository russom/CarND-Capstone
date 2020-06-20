#!/bin/bash
# Script to clean the tree from all compiled files, build the ROS nodes and
# source the environment
#
# M. Russo, 2020

# Remove the dedicated output directories
cd `dirname $0`

rm -rf build
rm -rf devel

# We're done cleaning
echo Cleaned up the project!

# Build
catkin_make

# We're done building
echo Project built!

# Source environment

source devel/setup.sh

# We're done setting uo the env
echo Environment set!
