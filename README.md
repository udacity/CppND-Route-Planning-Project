# CppND-Route-Planning-Project
This project is part of udacity C++ Nano degree. I implemented a route planner using A* search algorithm to find and visualize the best route between two points in a given map. The user will be asked to enter the (x,y) coordinates of the start and end points then the program will output the route visualized on the given map using the io2d liberary. My contribution to this project is presented in the ```route-planner.cpp```.<img src="Route-Planner.png" width="600" height="450" />
## Cloning
When cloning this project, be sure to use the --recurse-submodules flag. Using HTTPS:

```
git clone https://github.com/AbdelrahmanAbdeldaim/CppND-Route-Planning-Project.git --recurse-submodules
```
or with SSH:

```git clone git@github.com:AbdelrahmanAbdeldaim/CppND-Route-Planning-Project.git --recurse-submodules```
## Dependencies for Running Locally
* cmake >= 3.11.3
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 7.4.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same instructions as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* IO2D
  * Installation instructions for all operating systems can be found [here](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md)
  * This library must be built in a place where CMake `find_package` will be able to find it
 

## Compiling and Running

### Compiling
To compile the project, first, create a `build` directory and change to that directory:
```
mkdir build && cd build
```
From within the `build` directory, then run `cmake` and `make` as follows:
```
cmake ..
make
```
### Running
The executable will be placed in the `build` directory. From within `build`, you can run the project as follows:
```
./OSM_A_star_search
```
Or to specify a map file:
```
./OSM_A_star_search -f ../<your_osm_file.osm>
```
