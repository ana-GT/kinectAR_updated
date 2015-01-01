cmake_minimum_required( VERSION 2.8 )
set( CMAKE_CXX_FLAGS "-std=gnu++11" )
# Use our Find CMake files
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Modules )

# OpenCV: Necessary if your OpenCV is installed in a non-default location
set( OpenCV_DIR /home/ana/local/share/OpenCV )


# Find OpenCV and ALVAR
find_package( alvar REQUIRED )
include_directories( ${ALVAR_INCLUDE_DIRS} )
message( STATUS "Alvar include: ${ALVAR_INCLUDE_DIRS} " )
include_directories( /home/ana/local/include )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( PCL )
include_directories( ${PCL_INCLUDE_DIRS} )

find_package( PkgConfig )
pkg_check_modules( OSG openscenegraph )
include_directories( ${OSG_INCLUDE_DIRS} )

find_package( sns )
include_directories( ${SNS_INCLUDE_DIRS} )

find_package( ach )
include_directories( ${ACH_INCLUDE_DIRS} )

find_package( amino )
include_directories( ${AMINO_INCLUDE_DIRS} )


include_directories( ${CMAKE_SOURCE_DIR}/include )

add_library( kinectAR ${CMAKE_SOURCE_DIR}/src/kinectAR.cpp ${CMAKE_SOURCE_DIR}/src/kinectMarker.cpp ${CMAKE_SOURCE_DIR}/src/imageReceiver.cpp )
target_link_libraries( kinectAR ${ALVAR_LIBRARIES} ${OpenCV_LIBRARIES} ${SNS_LIBRARIES} ${ACH_LIBRARIES} ${AMINO_LIBRARIES} )
set_target_properties( kinectAR PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )

add_subdirectory( executables ) 
