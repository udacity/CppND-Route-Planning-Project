# Ubuntu 16.04 installation
Thanks for all the other students that have listed most of these steps in the chat room / in the knowledge base.
I discovered I needed a few more steps with Ubuntu 16.4, so here they are listed from the start.

## Packages and tools install
Install packages from the normal distribution:
```
$ sudo apt update
$ sudo apt install build-essential
$ sudo apt install libcairo2-dev
$ sudo apt install libgraphicsmagick1-dev
$ sudo apt install libpng-dev
```
## Install recent GCC
The original gcc version (5.4) is giving me a missing "variant" error when compiling io2d, so install version 7 with:
```
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
$ sudo apt-get update
$ sudo apt-get install g++-7
```

Then you need to link the g++-7 to your g++ command, then test version:
```
$ sudo rm /usr/bin/g++
$ sudo ln -s /usr/bin/g++-7 /usr/bin/g++
$ g++ --version
g++ (Ubuntu 7.4.0-1ubuntu1~16.04~ppa1) 7.4.0
```
## Install recent GIT
Ubuntu git package is a bit old and features like stash don't work with VS-Code, so update git:
```
$ git --version
git version 2.7.4
$ sudo add-apt-repository ppa:git-core/ppa
$ sudo apt-get update
$ sudo apt-get install git
$ git --version
git version 2.21.0
```
## Compile recent Cmake
Cmake ubuntu package version is too low, so download V3.14 from (https://cmake.org/download/)
The compilation may take a small while...
```
$ sudo apt-get purge cmake
$ tar -xzvf cmake-3.14.3.tar.gz
$ cd cmake-3.14.3
$ ./bootstrap
$ make
$ sudo make install 
```
## Compile io2d
Then compile and install io2d:
```
$ git clone --recurse-submodules https://github.com/cpp-io2d/P0267_RefImpl
$ cd P0267_RefImpl
$ mkdir Debug
$ cd Debug
$ cmake --config Debug "-DCMAKE_BUILD_TYPE=Debug" ..
$ cmake --build .
$ make
$ sudo make install
```
## Compile and run project code
At that point the project code should be compilable and run:
```
$ git clone https://github.com/udacity/CppND-Route-Planning-Project.git --recurse-submodules
$ cd CppND-Route-Planning-Project
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ../bin/CppND-Route-Planning-Project -f ../map.osm 
```

I have included ./build.sh and ./test.sh in the root to facilitate the compilation and run.