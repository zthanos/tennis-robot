cmake_minimum_required(VERSION 3.16)
project(tennis_ball_contact_system)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(tennis_ball_compliant_contact_system SHARED
  src/TennisBallContactSystem.cc)
target_compile_features(tennis_ball_compliant_contact_system PUBLIC cxx_std_17)
target_compile_options(tennis_ball_compliant_contact_system PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(tennis_ball_compliant_contact_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tennis_ball_compliant_contact_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)

if(BUILD_TESTING)
  enable_testing()
  add_executable(contact_model_test test/ContactModelTest.cc)
  target_compile_features(contact_model_test PUBLIC cxx_std_17)
  target_compile_options(contact_model_test PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(contact_model_test PRIVATE include)
  target_link_libraries(contact_model_test gz-sim8::gz-sim8)
  add_test(NAME contact_model_test COMMAND contact_model_test)
endif()

install(TARGETS tennis_ball_compliant_contact_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_package()
