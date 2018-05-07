cmake_minimum_required(VERSION 2.8.3)
project(pal_joint_trajectory_controller)


#set (CMAKE_BUILD_TYPE Debug)
set(REQUIRED_CATKIN_COMPONENTS
    actionlib
    angles
    roscpp
    urdf
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    control_msgs
    trajectory_msgs
    joint_trajectory_controller
    math_utils
    pal_ros_utils)

# Load catkin and all dependencies required for this package
find_package(catkin
    REQUIRED COMPONENTS ${REQUIRED_CATKIN_COMPONENTS} cmake_modules
)

# Declare catkin package
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${REQUIRED_CATKIN_COMPONENTS}
    LIBRARIES ${PROJECT_NAME}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fPIC")

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/joint_trajectory_controller.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
    add_subdirectory("test/")
endif()


# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# TODO: Install test resource files as well?
