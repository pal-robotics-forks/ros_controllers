find_package(catkin
  REQUIRED COMPONENTS
    actionlib
    controller_manager
    xacro
    roscpp
    urdf
)

find_package(rostest REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_add_gtest(trajectory_interface_test trajectory_interface_test.cpp)
target_link_libraries(trajectory_interface_test ${catkin_LIBRARIES})

add_executable(rrbot rrbot.cpp)
target_link_libraries(rrbot ${catkin_LIBRARIES})

add_executable(rrbot_wrapping rrbot_wrapping.cpp)
target_link_libraries(rrbot_wrapping ${catkin_LIBRARIES})

add_dependencies(tests rrbot)
add_dependencies(tests rrbot_wrapping)
add_dependencies(tests ${PROJECT_NAME})

add_rostest_gtest(joint_trajectory_controller_test
                  joint_trajectory_controller.test
                  joint_trajectory_controller_test.cpp)
target_link_libraries(joint_trajectory_controller_test ${catkin_LIBRARIES})

add_rostest_gtest(joint_trajectory_controller_vel_test
                  joint_trajectory_controller_vel.test
                  joint_trajectory_controller_test.cpp)
target_link_libraries(joint_trajectory_controller_vel_test ${catkin_LIBRARIES})

add_rostest_gtest(joint_trajectory_controller_wrapping_test
                  joint_trajectory_controller_wrapping.test
                  joint_trajectory_controller_wrapping_test.cpp)
target_link_libraries(joint_trajectory_controller_wrapping_test ${catkin_LIBRARIES})
