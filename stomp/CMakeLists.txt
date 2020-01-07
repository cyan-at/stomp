cmake_minimum_required(VERSION 2.8.3)
project(stomp)
set(CMAKE_CXX_FLAGS "-std=c++11")

# set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation)
# find_package(OpenMP REQUIRED)

find_package(Eigen3 REQUIRED)
# find_package(PkgConfig)
# pkg_search_module(Eigen3 REQUIRED eigen3)

find_package(yaml-cpp)

add_service_files(
  FILES
  HandlePlanStompSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stomp
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen3 yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  # src/chomp.cpp
  src/stomp_utils.cpp
  src/ur_kin.cpp
  src/covariant_movement_primitive.cpp
  src/policy_improvement.cpp
  src/stomp.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

# 2019-12-31 wrap debug printfs/couts with preprocessor macro
# turn off to disable
# target_compile_definitions(stomp PRIVATE DEBUG_VERBOSE=1)
target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG=1)
target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_COST=1)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(
    ${PROJECT_NAME}
    # 2019-07-14 OSX: install yaml-cpp from https://github.com/jbeder/yaml-cpp
    # then sudo make install install
    # will generate below
    /usr/local/lib/libyaml-cpp.a
    ${roscpp_LIBRARIES}
    ${catkin_LIBRARIES}
  )
else()
  target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
  )
endif()

#################################################

add_executable(test_ur_kin
  test/test_ur_kin.cpp
)

add_executable(test_stomp2d
  test/stomp_2d_test.cpp
)

# add_executable(test_cmp
#   test/test_cmp.cpp
# )
# add_executable(stomp_2d_00
#   test/stomp_2d_00.cpp
# )
#add_executable(test_eigen_perf
#  test/eigen_perf_test.cpp
#)
#add_executable(stomp_constrained_test
#  test/stomp_constrained_test.cpp
#)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  # ${OpenMP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

add_definitions(${EIGEN3_DEFINITIONS})
# add_definitions(${OpenMP_CXX_FLAGS})
#add_definitions(-DEIGEN_NO_DEBUG -DNDEBUG)

target_link_libraries(test_stomp2d ${PROJECT_NAME})

target_link_libraries(test_ur_kin ${PROJECT_NAME})

# target_link_libraries(test_cmp stomp)
# target_link_libraries(stomp_2d_00 stomp ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(stomp ${OpenMP_CXX_FLAGS})

#target_link_libraries(test_eigen_perf ${Eigen_LIBRARIES} ${OpenMP_CXX_FLAGS})
#target_link_libraries(stomp_constrained_test ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})

# add_executable(stomp_test_distributions
#   test/test_distributions.cpp
#   src/stomp_constrained.cpp
# )
# target_link_libraries(stomp_test_distributions ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
