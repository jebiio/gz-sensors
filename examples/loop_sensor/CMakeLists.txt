cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(loop_sensor)

find_package(OpenCV QUIET)
find_package(gz-sensors7 REQUIRED
  # Find built-in sensors
  COMPONENTS
    altimeter
    optical_flow

)
find_package(sdformat13 PATHS /usr/local/lib/cmake/ REQUIRED)

include_directories(/usr/include/gz/rendering7)


# A simple example of how to find custom sensors
# add_subdirectory(../../ odometer)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC
  gz-sensors7::gz-sensors7

  # Link to custom sensors
  gz-sensors7::optical_flow

  # Link to built-in sensors
  gz-sensors7::altimeter
  ${OpenCV_LIBS}
)
