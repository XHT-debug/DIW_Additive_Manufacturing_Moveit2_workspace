cmake_minimum_required(VERSION 3.8)
project(ur3_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  moveit_task_constructor_core
  moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  rviz_visual_tools
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  tf2
  ament_index_cpp
  my_custom_msgs
  controller_manager_msgs
)

find_package(my_custom_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# export CMAKE_PREFIX_PATH="/home/xht/moveit2_workspace/install/moveit_task_constructor_core/share/moveit_task_constructor_core/cmake"


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(ur3_moveit src/ur3_moveit.cpp)
add_executable(collect_ur3_moveit src/collect_ur3_moveit.cpp)
add_executable(adjust_ur3_moveit src/adjust_ur3_moveit.cpp)
add_executable(new_ur3_moveit src/new_ur3_moveit.cpp)
add_executable(tcp_velocity src/tcp_velocity.cpp)
add_executable(velocity_output src/velocity_output.cpp)
add_executable(pilz_ur_moveit src/pilz_ur_moveit.cpp)
add_executable(test_delay src/test_delay.cpp)
add_executable(velocity_collect src/velocity_collect.cpp)
add_executable(pressure_active_adjust src/pressure_active_adjust.cpp)
add_executable(servo_ur3_moveit src/servo_ur3_moveit.cpp)
add_executable(test_gongyi src/test_gongyi.cpp)
add_executable(switch_move_mode src/switch_move_mode.cpp)
add_executable(DIW_ui_moveit src/DIW_ui_moveit.cpp)
add_executable(distance_collect src/distance_collect.cpp)
add_executable(theta_data_collect src/theta_data_collect.cpp)
add_executable(servo_move src/servo_move.cpp)
add_executable(test_forward_velocity_controller src/test_forward_velocity_controller.cpp)
add_executable(test_forward_position_controller src/test_forward_position_controller.cpp)

ament_target_dependencies(ur3_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(collect_ur3_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(adjust_ur3_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(new_ur3_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(tcp_velocity ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(velocity_output ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(pilz_ur_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(test_delay ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(velocity_collect ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(pressure_active_adjust ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(servo_ur3_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(test_gongyi ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(switch_move_mode ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(DIW_ui_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(distance_collect ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(theta_data_collect ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(servo_move ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(test_forward_velocity_controller ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(test_forward_position_controller ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

target_include_directories(ur3_moveit PUBLIC include)
target_include_directories(collect_ur3_moveit PUBLIC include)
target_include_directories(adjust_ur3_moveit PUBLIC include)
target_include_directories(new_ur3_moveit PUBLIC include)
target_include_directories(tcp_velocity PUBLIC include)
target_include_directories(velocity_output PUBLIC include)
target_include_directories(pilz_ur_moveit PUBLIC include)
target_include_directories(test_delay PUBLIC include)
target_include_directories(velocity_collect PUBLIC include)
target_include_directories(pressure_active_adjust PUBLIC include)
target_include_directories(servo_ur3_moveit PUBLIC include)
target_include_directories(test_gongyi PUBLIC include)
target_include_directories(switch_move_mode PUBLIC include)
target_include_directories(DIW_ui_moveit PUBLIC include)
target_include_directories(distance_collect PUBLIC include)
target_include_directories(theta_data_collect PUBLIC include)
target_include_directories(servo_move PUBLIC include)
target_include_directories(test_forward_velocity_controller PUBLIC include)
target_include_directories(test_forward_position_controller PUBLIC include)

install(TARGETS ur3_moveit  DESTINATION lib/${PROJECT_NAME})
install(TARGETS collect_ur3_moveit  DESTINATION lib/${PROJECT_NAME})
install(TARGETS adjust_ur3_moveit  DESTINATION lib/${PROJECT_NAME})
install(TARGETS new_ur3_moveit  DESTINATION lib/${PROJECT_NAME})
install(TARGETS tcp_velocity  DESTINATION lib/${PROJECT_NAME})
install(TARGETS velocity_output  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pilz_ur_moveit  DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_delay  DESTINATION lib/${PROJECT_NAME})
install(TARGETS velocity_collect  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pressure_active_adjust  DESTINATION lib/${PROJECT_NAME})
install(TARGETS servo_ur3_moveit  DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_gongyi  DESTINATION lib/${PROJECT_NAME})
install(TARGETS switch_move_mode  DESTINATION lib/${PROJECT_NAME})
install(TARGETS DIW_ui_moveit  DESTINATION lib/${PROJECT_NAME})
install(TARGETS distance_collect  DESTINATION lib/${PROJECT_NAME})
install(TARGETS theta_data_collect  DESTINATION lib/${PROJECT_NAME})
install(TARGETS servo_move  DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_forward_velocity_controller  DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_forward_position_controller  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# add_executable(ur3_moveit src/ur3_moveit.cpp)
# target_include_directories(ur3_moveit PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(ur3_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   ur3_moveit
#   "moveit_ros_planning_interface"
#   "rclcpp"
# )

# install(TARGETS ur3_moveit
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
