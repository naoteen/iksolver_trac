cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    trac_ik_lib
    tf        #ここにtf関係を追加
    tf2
    tf2_ros
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
    pkg_nlopt
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
)


add_executable(trac_ik_jointpub src/trac_ik_jointpub.cpp)
target_link_libraries(trac_ik_jointpub
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(pos_force_controller src/pos_force_controller.cpp)
target_link_libraries(pos_force_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
