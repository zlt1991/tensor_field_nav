cmake_minimum_required(VERSION 2.8.3)
project(rviz_textured_quads)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs sensor_msgs shape_msgs rviz pluginlib class_loader cv_bridge message_generation geometry_msgs sensor_msgs cmake_modules eigen_conversions rviz)
find_package(Eigen REQUIRED)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(OpenGL REQUIRED)

## I (original author) prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

include_directories(SYSTEM "msg_gen/cpp/include")

add_message_files(
  FILES
  TexturedQuad.msg
  TexturedQuadArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
  LIBRARIES
    rviz_textured_quads_custom
    ${OPENGL_LIBRARIES}
  INCLUDE_DIRS
  	src
    ${OPENGL_INCLUDE_DIR}
  CATKIN_DEPENDS
    roscpp 
    rospy
    roslib
    std_msgs 
    sensor_msgs
    shape_msgs
    rviz 
    pluginlib 
    class_loader
    cv_bridge
  	geometry_msgs
  	sensor_msgs
  	eigen_conversions
  	rviz
)


include_directories(
	src
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${OPENGL_INCLUDE_DIR}
                    ${QT_INCLUDE_DIR})
                    
link_directories(${catkin_LIBRARY_DIRS})


# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/mesh_display_custom.h)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(TEXTURED_QUAD_LIB rviz_textured_quads_custom)

add_library(${TEXTURED_QUAD_LIB}_core src/text_node.h src/mesh_display_custom.cpp ${MOC_SOURCES})
target_link_libraries(${TEXTURED_QUAD_LIB}_core ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Eigen_LIBRARIES})

add_dependencies(${TEXTURED_QUAD_LIB}_core ${catkin_EXPORTED_TARGETS})

add_library(${TEXTURED_QUAD_LIB} src/plugin_init.cpp)
target_link_libraries(${TEXTURED_QUAD_LIB} ${TEXTURED_QUAD_LIB}_core ${catkin_LIBRARIES})

add_dependencies(${TEXTURED_QUAD_LIB} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${TEXTURED_QUAD_LIB} ${TEXTURED_QUAD_LIB}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY textures
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz_textured_quads_custom
)

install(FILES
  mesh_display_custom_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
