cmake_minimum_required(VERSION 2.8.3)
project(visor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
	rospy
  std_msgs
  # uvc_camera #usb_cam 
  message_generation
  message_filters
  # dynamic_reconfigure
)


#find_package(catkin REQUIRED dynamic_reconfigure)
#generate_dynamic_reconfigure_options(
#  cfg/VisualAttentionParams.cfg
#)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

##find_package(PCL 1.2 REQUIRED)
##link_directories(${PCL_LIBRARY_DIRS})
##add_definitions(${PCL_DEFINITIONS})

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
	# DIRECTORY msg # NO NEED
  FILES
  # gistElemArray.msg # TODO= comment this
  textureInfo.msg
  colorInfo.msg
  localTexture.msg
  localColor.msg
  blobInfo.msg
  detEntityArray.msg
  sceneChanges.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	call.srv
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
  sensor_msgs
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
  LIBRARIES vattnlib
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs message_runtime ${MESSAGE_DEPENDENCIES} #usb_cam uvc_camera
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/lib # for fftw3
  /usr/local/include
)

#############
## Library ##
#############
# # gist
# add_library           (simg  src/Gist/standalone_image.c )

# add_library           (gist  src/Gist/gist.c )
# target_link_libraries (gist  simg )

# add_library           (gist_bridge  src/Gist/gist_bridge.cpp )
# target_link_libraries (gist_bridge  gist  simg )

# Visual Attention
add_library           (salmap  src/Vattn/saliency_map.cpp )
target_link_libraries (salmap  ${OpenCV_LIBRARIES} )

add_library           (extdat  src/Vattn/extract_data.cpp )
target_link_libraries (extdat  ${OpenCV_LIBRARIES} )

add_library           (vattnlib  src/Vattn/vattn.cpp )
target_link_libraries (vattnlib  ${OpenCV_LIBRARIES} salmap extdat)

# Object Extractor
add_library           (objext  src/object_extractor.cpp )
target_link_libraries (objext  ${OpenCV_LIBRARIES} )

# # color info
# add_library           (color  src/Color/color.cpp )
# add_dependencies      (color  ${visor_EXPORTED_TARGETS} )

# COLOR DESCRIPTOR ---------------------------------------
# ADDRESSLIB
add_library           (address  src/MPEG7FexLib_src_OpenCV22/AddressLib/address.c )

add_library           (vopio  src/MPEG7FexLib_src_OpenCV22/AddressLib/vopio.c )
target_link_libraries (vopio  address )

add_library           (nhood  src/MPEG7FexLib_src_OpenCV22/AddressLib/nhood.c )
target_link_libraries (nhood  address vopio )

add_library           (inter  src/MPEG7FexLib_src_OpenCV22/AddressLib/inter.c )
target_link_libraries (inter  address vopio nhood)

add_library           (intra  src/MPEG7FexLib_src_OpenCV22/AddressLib/intra.c )
target_link_libraries (intra  address vopio nhood intra)

# DESCRIPTORS
add_library           (color_layout  src/MPEG7FexLib_src_OpenCV22/Descriptors/ColorLayout.cpp )
add_library           (color_quant  src/MPEG7FexLib_src_OpenCV22/Descriptors/ColorQuant.cpp )
add_library           (color_space  src/MPEG7FexLib_src_OpenCV22/Descriptors/ColorSpace.cpp )
add_library           (color_structure  src/MPEG7FexLib_src_OpenCV22/Descriptors/ColorStructure.cpp )
add_library           (contour_shape  src/MPEG7FexLib_src_OpenCV22/Descriptors/ContourShape.cpp )
add_library           (dominant_color  src/MPEG7FexLib_src_OpenCV22/Descriptors/DominantColor.cpp )
target_link_libraries (dominant_color  color_space color_quant)
add_library           (edge_hist  src/MPEG7FexLib_src_OpenCV22/Descriptors/EdgeHist.cpp )
add_library           (face_recog  src/MPEG7FexLib_src_OpenCV22/Descriptors/FaceRecognition.cpp )
add_library           (homo_txt  src/MPEG7FexLib_src_OpenCV22/Descriptors/HomoTexture.cpp )
add_library           (region_shape  src/MPEG7FexLib_src_OpenCV22/Descriptors/RegionShape.cpp )
add_library           (scalable_color  src/MPEG7FexLib_src_OpenCV22/Descriptors/ScalableColor.cpp )

# EXTRACTION
add_library           (color_layout_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/ColorLayoutExtraction.cpp )
target_link_libraries (color_layout_ext  vopio color_layout)

add_library           (color_space_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/ColorSpaceExtraction.cpp )
target_link_libraries (color_space_ext  vopio color_space)

add_library           (color_structure_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/ColorStructureExtraction.cpp )
target_link_libraries (color_structure_ext  vopio color_structure)

add_library           (contour_shape_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/ContourShapeExtraction.cpp )
target_link_libraries (contour_shape_ext  vopio contour_shape)

add_library           (imgutil  src/MPEG7FexLib_src_OpenCV22/Extraction/imgutil.c )

add_library           (dominant_color_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/DominantColorExtraction.cpp )
target_link_libraries (dominant_color_ext  vopio dominant_color imgutil color_space_ext )

add_library           (edge_hist_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/EdgeHistExtraction.cpp )
target_link_libraries (edge_hist_ext  vopio edge_hist )

add_library           (face_recog_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/FaceRecognitionExtraction.cpp )
target_link_libraries (face_recog_ext  vopio face_recog )

add_library           (gof_gop_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/GoFGoPColorExtraction.cpp )
target_link_libraries (gof_gop_ext  vopio scalable_color scalable_color_ext)

add_library           (homo_txt_extractor  src/MPEG7FexLib_src_OpenCV22/Extraction/homo_texture_extractor.cpp )
add_library           (homo_txt_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/HomoTextureExtraction.cpp )
target_link_libraries (homo_txt_ext  vopio homo_txt homo_txt_extractor)

add_library           (region_shape_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/RegionShapeExtraction.cpp )
target_link_libraries (region_shape_ext  vopio region_shape)

add_library           (scalable_color_ext  src/MPEG7FexLib_src_OpenCV22/Extraction/ScalableColorExtraction.cpp )
target_link_libraries (scalable_color_ext  vopio intra scalable_color color_space_ext)

# OTHERS
add_library           (convert  src/MPEG7FexLib_src_OpenCV22/Convert.cpp )
add_dependencies      (convert  ${visor_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
target_link_libraries (convert  vopio )

add_library           (frame  src/MPEG7FexLib_src_OpenCV22/Frame.cpp )
add_dependencies      (frame  ${visor_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
target_link_libraries (frame  convert vopio )

add_library           (Feature  src/MPEG7FexLib_src_OpenCV22/Feature.cpp )
add_dependencies      (Feature  ${visor_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
target_link_libraries (Feature
  frame
  convert
  color_structure_ext
  dominant_color_ext
  color_layout_ext
  scalable_color_ext
  gof_gop_ext
  edge_hist_ext
  homo_txt_ext
  contour_shape_ext
  region_shape_ext
  color_space_ext
  face_recog_ext
)
# END OF COLOR DESCRIPTOR ---------------------------------------

###########
## nodes ##
###########
# Image Channel Divider
add_executable        (img_chnldvdr_node  src/img_chnldvdr_node.cpp )
target_link_libraries (img_chnldvdr_node  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} )

# Saliency Detection Node
add_executable        (saliency_det_node  src/saliency_det_node.cpp )
add_dependencies      (saliency_det_node  visor_gencfg visor_generate_messages_cpp )
target_link_libraries (saliency_det_node  ${catkin_LIBRARIES}  vattnlib ${OpenCV_LIBRARIES} )

# IMG RESIZER
add_executable        (img_resizer_node  src/img_resizer_node.cpp )
target_link_libraries (img_resizer_node  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} )

# Attention Window Node
add_executable        (attn_window_node  src/attn_window_node.cpp )
add_dependencies      (attn_window_node  ${visor_EXPORTED_TARGETS} )
target_link_libraries (attn_window_node  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} objext)

# Local Color Feature
add_executable        (local_color_node  src/local_color_node.cpp )
target_link_libraries (local_color_node  ${catkin_LIBRARIES}
  #color # THis is the naive approach
  Feature
)

# Local Shape
add_executable        (local_shape_node  src/local_shape_node.cpp )
add_dependencies      (local_shape_node
  visor_generate_messages_cpp
  # simg #not used for now
  # gist #not used for now
  # gist_bridge #not used for now
)
target_link_libraries (local_shape_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # simg        #not used for now
  # gist        #not used for now
  # fftw3f      #not used for now
  # gist_bridge #not used for now
  Feature
)

# Scenectrl
add_executable(scenectrl src/scenectrl.cpp)
add_dependencies(scenectrl visor_generate_messages_cpp)
target_link_libraries(scenectrl
	${catkin_LIBRARIES}
)

# Imageloader
add_executable(imgloader src/imgloader.cpp)
add_dependencies(imgloader visor_generate_messages_cpp)
target_link_libraries(imgloader
	${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

############################################################

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(visor_node visor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(visor_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS visor visor_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_visor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
