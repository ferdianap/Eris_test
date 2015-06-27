cmake_minimum_required(VERSION 2.8.3)
project(eris)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation #genmsg # in hydro called message_generation
  message_filters
  sensor_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)
#find_package(Eigen REQUIRED)
find_package(visor REQUIRED vattnlib)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  smEntity.msg
  episode.msg
  ffiDatabase.msg
  tag.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  query.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ${MESSAGE_DEPENDENCIES}
  std_msgs
  visor
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
	LIBRARIES eris
  CATKIN_DEPENDS roscpp std_msgs message_runtime ${MESSAGE_DEPENDENCIES} vattnlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
include_directories( include
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib # for fftw3
  /usr/local/include
  ${visor_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(eris
#   src/${PROJECT_NAME}/eris.cpp
# )


#############
## Library ##
#############
# FILE OPERATIONS
add_library           (file_ops  src/file_ops/file_operations.cpp )
add_dependencies      (file_ops  ${eris_EXPORTED_TARGETS} )
target_link_libraries (file_ops  ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )

# ENCODE TMP FFI
add_library           (encode_tmp_ffi  src/memoryproc/encode_tmp_ffi.cpp )
add_dependencies      (encode_tmp_ffi  ${eris_EXPORTED_TARGETS} )
target_link_libraries (encode_tmp_ffi  ${catkin_LIBRARIES} )


# MEMORYPROCESSING
add_library           (memproc
  src/memoryproc/hricase1.cpp
  src/memoryproc/hricase2.cpp
  src/memoryproc/hricase3.cpp
)
add_dependencies      (memproc  ${eris_EXPORTED_TARGETS} )
target_link_libraries (memproc
  ${catkin_LIBRARIES}
  ${visor_LIBRARIES}
  objext
  Feature
  file_ops
  encode_tmp_ffi
)

# CUSTOM OPTION DESCRIPTION ######## USE THESE FOR THE LATEST BOOST VERSION (> 1.57)
#add_library           (co_desc  src/file_ops/CustomOptionDescription.cpp )
#add_dependencies      (co_desc  ${eris_EXPORTED_TARGETS} )
#target_link_libraries (co_desc  ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )

# OPTION PRINTER
#add_library           (opt_printer  src/file_ops/OptionPrinter.cpp )
#add_dependencies      (opt_printer  ${eris_EXPORTED_TARGETS} )
#target_link_libraries (opt_printer  ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}  co_desc )
#############################################################################

###########
## nodes ##
###########
# SM ENCODER
add_executable        (encoder_node
  src/encoder_node.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies      (encoder_node
  eris_generate_messages_cpp
  ${eris_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries (encoder_node
  ${catkin_LIBRARIES}
  file_ops
  memproc
  # ${OpenCV_LIBRARIES}
)

# EM ENCODER (CURRENTLY SERVES NO PURPOSE)
# add_executable        (em_encoder_node  src/em_encoder_node.cpp )
# add_dependencies      (em_encoder_node  eris_generate_messages_cpp )
# target_link_libraries (em_encoder_node  ${catkin_LIBRARIES} )

# DUMMY MEMORY CREATOR (DMC)
add_executable        (dmc_node  src/dmc_node.cpp )
add_dependencies      (dmc_node  eris_generate_messages_cpp )
target_link_libraries (dmc_node  ${catkin_LIBRARIES} file_ops)

# QUERY SERVER
add_executable        (query_server  src/query_server.cpp )
add_dependencies      (query_server  eris_generate_messages_cpp )
target_link_libraries (query_server
  ${catkin_LIBRARIES}
  memproc
  encode_tmp_ffi
)

# EM QUERY SERVER
#add_executable        (em_query_server  src/em_query_server.cpp )
#add_dependencies      (em_query_server  eris_generate_messages_cpp )
#target_link_libraries (em_query_server  ${catkin_LIBRARIES}  file_ops )

# QUERY CLIENT
add_executable        (query_client  src/query_client.cpp )
add_dependencies      (query_client  eris_generate_messages_cpp )
target_link_libraries (query_client  ${catkin_LIBRARIES}  boost_program_options
  #opt_printer
)

# REVISE KNOWLEDGE NODE
add_executable        (revise_knowledge_node  src/revise_knowledge_node.cpp )
add_dependencies      (revise_knowledge_node  eris_generate_messages_cpp )
target_link_libraries (revise_knowledge_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  file_ops
  encode_tmp_ffi
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS eris eris_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_eris.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # add_rostest_gtest(test_memory_module test/core.test test/memory_module.cpp)
  # target_link_libraries(test_memory_module ${catkin_LIBRARIES} file_ops)
  # add_rostest_gtest(test_tro test/core.test test/tro.cpp)
  # target_link_libraries(test_tro ${catkin_LIBRARIES} file_ops)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

