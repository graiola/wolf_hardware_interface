cmake_minimum_required(VERSION 3.5)
project(wolf_hardware_interface)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    set(ROS_VERSION "ros")
    catkin_package(INCLUDE_DIRS include/${ROS_VERSION} LIBRARIES wolf_robot_hw)
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")
elseif(${ament_cmake_FOUND})
    set(ROS_VERSION "ros2")
    ament_export_include_directories(include/${ROS_VERSION})
    ament_export_libraries(wolf_robot_hw)
    ament_package()
    add_definitions(-DROS2)
    message(STATUS "Add ROS2 definition")
endif()

find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(wolf_controller_utils REQUIRED)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

include_directories(
  include/${ROS_VERSION}
  ${hardware_interface_INCLUDE_DIRS}
  ${transmission_interface_INCLUDE_DIRS}
  ${wolf_controller_utils_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(wolf_robot_hw src/${ROS_VERSION}/wolf_robot_hw.cpp)
target_link_libraries(wolf_robot_hw
    ${hardware_interface_LIBRARIES}
    ${transmission_interface_LIBRARIES}
    ${wolf_controller_utils_LIBRARIES})

install(TARGETS wolf_robot_hw
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(
  DIRECTORY include/${ROS_VERSION}/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
