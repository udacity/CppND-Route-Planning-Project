cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(OSM_A_star_search)

# Project Output Paths
set(MAINFOLDER ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH "${MAINFOLDER}/lib")

# Locate Project Prerequisites
find_package(io2d REQUIRED)
find_package(Cairo)
find_package(GraphicsMagick)

# Add Build Targets
set(IO2D_WITHOUT_SAMPLES 1)
set(IO2D_WITHOUT_TESTS 1)
add_subdirectory(thirdparty/pugixml)
add_subdirectory(thirdparty/googletest)

# Find all executables
file(GLOB project_SRCS src/*.cpp src/*.h)

# Add project executable
add_executable(OSM_A_star_search ${project_SRCS})

target_link_libraries(OSM_A_star_search
    PRIVATE io2d::io2d
    PUBLIC pugixml
)

if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    target_link_libraries(OSM_A_star_search PUBLIC pthread)
endif()

if(MSVC)
	target_compile_options(OSM_A_star_search PUBLIC /D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING /wd4459)
endif()

# Create a library for unit tests
add_library(route_planner OBJECT src/route_planner.cpp src/model.cpp src/route_model.cpp)
target_include_directories(route_planner PRIVATE thirdparty/pugixml/src)

# Add testing executable
add_executable(test test/utest_rp_a_star_search.cpp)
target_link_libraries(test gtest_main route_planner pugixml)
add_test(NAME test COMMAND test)
unset(TESTING CACHE)
