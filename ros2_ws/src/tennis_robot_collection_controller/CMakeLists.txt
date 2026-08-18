cmake_minimum_required(VERSION 3.8)
project(tennis_robot_collection_controller)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tennis_robot_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(collection_path_canonicalization
  src/collection_path_canonicalization.cpp
  src/collection_execution_context_contract.cpp
  src/collection_tracking_core.cpp)
set_target_properties(collection_path_canonicalization PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(collection_path_canonicalization PUBLIC cxx_std_17)
target_include_directories(collection_path_canonicalization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(collection_path_canonicalization nav_msgs)
target_link_libraries(collection_path_canonicalization OpenSSL::Crypto)

add_library(collection_nav2_controller SHARED
  src/collection_nav2_controller.cpp
  src/collection_progress_goal_checker.cpp)
target_compile_features(collection_nav2_controller PUBLIC cxx_std_17)
target_include_directories(collection_nav2_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(collection_nav2_controller
  nav2_core pluginlib rclcpp rclcpp_lifecycle tennis_robot_msgs tf2 tf2_ros)
target_link_libraries(collection_nav2_controller collection_path_canonicalization)
pluginlib_export_plugin_description_file(nav2_core collection_nav2_controller.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS collection_path_canonicalization
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS collection_nav2_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES collection_nav2_controller.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_collection_path_canonicalization
    test/test_collection_path_canonicalization.cpp)
  target_link_libraries(test_collection_path_canonicalization
    collection_path_canonicalization)
  ament_add_gtest(test_collection_nav2_controller_plugin
    test/test_collection_nav2_controller_plugin.cpp)
  ament_target_dependencies(test_collection_nav2_controller_plugin nav2_core pluginlib)
  ament_add_gtest(test_collection_progress_goal_checker
    test/test_collection_progress_goal_checker.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_collection_progress_goal_checker collection_nav2_controller)
  ament_target_dependencies(test_collection_progress_goal_checker
    nav2_core rclcpp rclcpp_lifecycle tennis_robot_msgs)
  set_tests_properties(test_collection_nav2_controller_plugin PROPERTIES
    ENVIRONMENT
      "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  ament_add_gtest(test_collection_tracking_core
    test/test_collection_tracking_core.cpp)
  target_link_libraries(test_collection_tracking_core collection_path_canonicalization)
  ament_add_gtest(test_collection_nav2_controller_runtime
    test/test_collection_nav2_controller_runtime.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_collection_nav2_controller_runtime collection_nav2_controller)
  ament_target_dependencies(test_collection_nav2_controller_runtime
    pluginlib rclcpp rclcpp_lifecycle tennis_robot_msgs)
  # Phase 6B cross-language parity: loads the plugin at runtime (pluginlib) and
  # feeds it a context+path produced by the pure Python serializer, so it links
  # only the exported canonicalization lib, not the plugin target.
  ament_add_gtest(test_collection_execution_context_parity
    test/test_collection_execution_context_parity.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_collection_execution_context_parity collection_path_canonicalization)
  ament_target_dependencies(test_collection_execution_context_parity
    nav2_core pluginlib rclcpp rclcpp_lifecycle tennis_robot_msgs nav_msgs)
  set_tests_properties(test_collection_execution_context_parity PROPERTIES
    ENVIRONMENT
      "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  add_launch_test(test/test_collection_controller_server_isolated.launch.py TIMEOUT 60)
  set_tests_properties(test_test_collection_controller_server_isolated.launch.py PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}/../tennis_robot:${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
endif()

ament_export_include_directories(include)
ament_export_libraries(collection_path_canonicalization)
ament_package()
