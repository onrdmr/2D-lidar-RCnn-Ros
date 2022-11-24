cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_robot_plugin)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
  message_generation
  message_runtime
)



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
## Generate services in the 'srv' folder 
add_service_files( 
   FILES 
   demo_srv.srv 
 ) 

generate_messages( 
   DEPENDENCIES 
   std_msgs 
) 


catkin_package()

# SET(CMAKE_PREFIX_PATH "/home/onur/geos-3.11.0/cmake")


include_directories( 
   include 
   ${catkin_INCLUDE_DIRS} 
) 

find_package(gazebo REQUIRED)
# SET(GEOS_DIR ${CMAKE_HOME_DIRECTORY}/geos)
# SET(GEOS_INCLUDE_DIRS ${CMAKE_HOME_DIRECTORY}/geos/include)
# SET(GEOS_LIBRARY_DIRS ${CMAKE_HOME_DIRECTORY}/geos/_build/lib)

# MESSAGE(${GEOS_DIR}/cmake)

# LIST(APPEND CMAKE_MODULE_PATH ${GEOS_DIR}/cmake)
find_package(GEOS REQUIRED)
# SET(GEOS_LIBRARIES "")
# LIST(APPEND  ${GEOS_DIR}/cmake)

# MESSAGE("${GEOS_INCLUDE_DIRS}: " ${GEOS_DIR})
# include_directories(${GEOS_INCLUDE_DIRS})
# link_directories(${GEOS_LIBRARY_DIRS})




include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# MESSAGE("geos_c " GEOS::geos_c)
add_library(exploration_scan SHARED exploration_scan.cc )
add_dependencies(exploration_scan mastering_ros_demo_pkg_generate_messages_cpp) 
target_link_libraries(exploration_scan PRIVATE ${GAZEBO_LIBRARIES} GEOS::geos_c ${catkin_LIBRARIES})

SET(DEVEL_LIB ${CMAKE_HOME_DIRECTORY}/../devel/lib)
LIST(APPEND GAZEBO_PLUGIN_PATH ":")
LIST(APPEND GAZEBO_PLUGIN_PATH ${DEVEL_LIB})

#SET($ENV{GAZEBO_PLUGIN_PATH} ${GAZEBO_PLUGIN_PATH})
SET(EXPORT_GAZEBO_PLUGIN "export GAZEBO_PLUGIN_PATH=" ${GAZEBO_PLUGIN_PATH} "\n")

MESSAGE("GAZEBO_PLUGIN_PATH " ${GAZEBO_PLUGIN_PATH})
if(NOT EXISTS ${DEVEL_LIB}/../setup.written) 
 file(APPEND ${DEVEL_LIB}/../setup.sh ${EXPORT_GAZEBO_PLUGIN})
 file(WRITE ${DEVEL_LIB}/../setup.written 1)
endif()

