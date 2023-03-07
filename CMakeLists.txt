cmake_minimum_required(VERSION 2.8.3)
project(kmu_gui)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
  roscpp
  std_msgs
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  #OpenCV
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  #${OpenCV_LIBS}
  )

set(INCLUDE_DIRECTORIES
  #${OpenCV_LIBS}
  )


##########################################
##  Be Carefull editing below this line ##
##########################################

## Compile as C++14
add_compile_options(-std=c++14)

## need this for qt auto generated headers to include!
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###
###  Add Non ROS dependencies
###

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

###
###  Add any required ros packages here
###
find_package(catkin REQUIRED COMPONENTS message_generation ${ROS_DEPENDS})
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  device_control_pkg_msg/battery.msg
  device_control_pkg_msg/capture.msg
  device_control_pkg_msg/fan.msg
  device_control_pkg_msg/gas.msg
  motor_control_pkg_msg/joystick.msg
  motor_control_pkg_msg/motor.msg

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
  message_generation
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR}
  ${INCLUDE_DIRECTORIES}
)

###########
## Build ##
###########

file(GLOB HDRS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES
  "nodes/*.cpp"
)




###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files

#find_package(Qt5Widgets)
#find_package(Qt5PrintSupport)
#find all the qt UI stuff
file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
)

message("Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (UI_HDRS  ${UI})

message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)

# find all resource files
file(GLOB QRC
    "rc/*.qrc"
)

###
### compile
###

message("buliding ${PROJECT_NAME} node: ")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})

  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES} ${OPENGL_gl_LIBRARY} ${GLUT_LIBRARY} ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")

  list(APPEND NODE_LIST ${NODE_NAME})

endforeach(NODE_FILE)


install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

