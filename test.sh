#!/bin/bash
rm -rf ./bin
#rm -rf ./build

mkdir build
cd build

#cmake ..
#cmake .. "-DTESTING=RouteModel" 
#cmake .. "-DTESTING=RMNodeClass" 
#cmake .. "-DTESTING=RMSNodes" 
#cmake .. "-DTESTING=NodeDist" 
#cmake .. "-DTESTING=NodeToRoad" 
#cmake .. "-DTESTING=FindNeighbors" 
#cmake .. "-DTESTING=FindClosest" 
#cmake .. "-DTESTING=AStarStub" 
cmake .. "-DTESTING=AStarSearch"

make 
../bin/test