cmake_minimum_required(VERSION 3.16)
project(phase2_v3_drive_5a_system)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(v3_drive_5a_system SHARED V3Drive5aSystem.cc)
target_compile_features(v3_drive_5a_system PUBLIC cxx_std_17)
target_compile_options(v3_drive_5a_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(v3_drive_5a_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-msgs10::gz-msgs10)

add_library(v3_tilt_5b_system SHARED V3Tilt5bSystem.cc)
target_compile_features(v3_tilt_5b_system PUBLIC cxx_std_17)
target_compile_options(v3_tilt_5b_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(v3_tilt_5b_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-msgs10::gz-msgs10)

add_library(v3_intake_5c_system SHARED V3Intake5cSystem.cc)
target_compile_features(v3_intake_5c_system PUBLIC cxx_std_17)
target_compile_options(v3_intake_5c_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(v3_intake_5c_system gz-sim8::gz-sim8 gz-plugin2::gz-plugin2 gz-msgs10::gz-msgs10)

add_library(v3_basket_5c2_observer SHARED V3Basket5c2Observer.cc)
target_compile_features(v3_basket_5c2_observer PUBLIC cxx_std_17)
target_compile_options(v3_basket_5c2_observer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(v3_basket_5c2_observer gz-sim8::gz-sim8 gz-plugin2::gz-plugin2 gz-msgs10::gz-msgs10)

add_library(v3_launch_5d1_system SHARED V3Launch5d1System.cc)
target_compile_features(v3_launch_5d1_system PUBLIC cxx_std_17)
target_compile_options(v3_launch_5d1_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(v3_launch_5d1_system gz-sim8::gz-sim8 gz-plugin2::gz-plugin2 gz-msgs10::gz-msgs10)

add_library(v3_platen_p261_system SHARED V3PlatenP261System.cc)
target_compile_features(v3_platen_p261_system PUBLIC cxx_std_17)
target_compile_options(v3_platen_p261_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(v3_platen_p261_system gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)

add_library(v3_rebound_p261_observer SHARED V3ReboundP261Observer.cc)
target_compile_features(v3_rebound_p261_observer PUBLIC cxx_std_17)
target_compile_options(v3_rebound_p261_observer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(v3_rebound_p261_observer gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
