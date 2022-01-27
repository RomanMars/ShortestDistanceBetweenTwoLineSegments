cmake_minimum_required(VERSION 3.0.0)
project(ShortestDistanceBetweenTwoLineSegments VERSION 0.1.0)

include(CTest)
enable_testing()

# file(GLOB ShortestDistanceBetweenTwoLineSegments
#      "*.h"
#      "*.cpp"
# )

add_executable(ShortestDistanceBetweenTwoLineSegments main.cpp ${ShortestDistanceBetweenTwoLineSegments})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

target_include_directories(ShortestDistanceBetweenTwoLineSegments PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/libs/")
