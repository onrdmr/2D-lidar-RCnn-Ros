# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# find_package(GAZEBO)
# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")




cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_map_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
)

find_package(catkin REQUIRED COMPONENTS
)

find_package(Boost REQUIRED COMPONENTS system)

add_service_files( 
   FILES 
   demo_srv.srv 
 ) 

generate_messages( 
   DEPENDENCIES 
   std_msgs 
) 


catkin_package()


find_package(gazebo REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(factory SHARED factory.cc)
add_dependencies(factory mastering_ros_demo_pkg_generate_messages_cpp) 

target_link_libraries(factory
  ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
)

# SET(DEVEL_LIB ${CMAKE_HOME_DIRECTORY}/../devel/lib)
# LIST(APPEND GAZEBO_PLUGIN_PATH ":")
# LIST(APPEND GAZEBO_PLUGIN_PATH ${DEVEL_LIB})

SET(EDITOR_MODEL $ENV{HOME}/building_editor_models)
LIST(APPEND GAZEBO_MODEL_PATH ":")
LIST(APPEND GAZEBO_MODEL_PATH ${EDITOR_MODEL})


#SET($ENV{GAZEBO_PLUGIN_PATH} ${GAZEBO_PLUGIN_PATH})
# SET(EXPORT_GAZEBO_PLUGIN "export GAZEBO_PLUGIN_PATH=" ${GAZEBO_PLUGIN_PATH})
SET(EXPORT_GAZEBO_MODEL "export GAZEBO_MODEL_PATH=" ${GAZEBO_MODEL_PATH} "\n")

SET(DEVEL_LIB ${CMAKE_HOME_DIRECTORY}/../devel/lib)


MESSAGE("GAZEBO_MODEL_PATH " ${GAZEBO_MODEL_PATH})
if(NOT EXISTS ${DEVEL_LIB}/../setup_model.written) 
 file(APPEND ${DEVEL_LIB}/../setup.sh ${EXPORT_GAZEBO_MODEL})
 file(WRITE ${DEVEL_LIB}/../setup_model.written 1)
endif()


# MESSAGE("GAZEBO_PLUGIN_PATH " ${GAZEBO_PLUGIN_PATH})
# if(NOT EXISTS ${DEVEL_LIB}/../setup_plugin.written) 
#  file(APPEND ${DEVEL_LIB}/../setup.sh ${EXPORT_GAZEBO_PLUGIN})
#  file(WRITE ${DEVEL_LIB}/../setup_plugin.written 1)
# endif()

