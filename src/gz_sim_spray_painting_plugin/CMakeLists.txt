cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(gz_sim_spray_painting_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(gz-sim8        REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10      REQUIRED)
find_package(gz-plugin2     REQUIRED COMPONENTS register)

add_library(SprayPaintPlugin SHARED
  src/SprayPaintPlugin.cc
)

target_include_directories(SprayPaintPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(SprayPaintPlugin
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-plugin2::register
)

install(TARGETS SprayPaintPlugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
