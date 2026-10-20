cmake_minimum_required(VERSION 3.10)

project(kpp_magang)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(program src/main.cpp)
add_executable(calib src/calib.cpp)
add_executable(color src/color.cpp)


target_link_libraries(program ${OpenCV_LIBS})
target_link_libraries(calib ${OpenCV_LIBS})
target_link_libraries(color ${OpenCV_LIBS})

