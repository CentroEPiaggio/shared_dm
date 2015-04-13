cmake_minimum_required(VERSION 2.8)

project(dual_manipulation_shared)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  moveit_msgs
  kdl_conversions
  tf
)

add_message_files(
  FILES
  planner_item.msg
  graph.msg
  grasp_trajectory.msg
  ik_response.msg
  pe.msg
  peArray.msg
)

add_service_files(
  FILES
  ik_service.srv
  ik_service_legacy.srv
  planner_service.srv
  scene_object_service.srv
  state_manager_service.srv
  gui_target_service.srv
  estimate.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs moveit_msgs)

catkin_package(
  INCLUDE_DIRS include/
  LIBRARIES ${PROJECT_NAME} lemon
#  CATKIN_DEPENDS roscpp
   #DEPENDS
   )
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)

add_subdirectory(src/lemon)

add_library(${PROJECT_NAME} SHARED src/databasemapper.cpp src/geometry_tools.cpp src/serialization_utils.cpp src/parsing_utils.cpp src/databasewriter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} sqlite3)
add_dependencies(${PROJECT_NAME} dual_manipulation_shared_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
         EXPORT ${PROJECT_NAME}
#         ARCHIVE DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT lib
#         RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT bin
         LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}" COMPONENT shlib
)

add_executable(test_serialization test/test_serialization.cpp)
target_link_libraries(test_serialization ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(test_serialization dual_manipulation_shared_gencpp)

add_executable(test_deserialization test/test_deserialization.cpp )
target_link_libraries(test_deserialization ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(test_deserialization dual_manipulation_shared_gencpp)

add_executable(test_create_grasp_transitions test/test_create_grasp_transitions.cpp)
target_link_libraries(test_create_grasp_transitions ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(test_create_grasp_transitions dual_manipulation_shared_gencpp)

add_executable(convert_rh_to_lh test/convert_RH_to_LH_grasps.cpp)
target_link_libraries(convert_rh_to_lh ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(convert_rh_to_lh dual_manipulation_shared_gencpp)

add_executable(convert_ik_from_legacy test/convert_ik_from_legacy.cpp)
target_link_libraries(convert_ik_from_legacy ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(convert_ik_from_legacy dual_manipulation_shared_gencpp)

add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND echo 'Installing lemon library...'
    COMMAND make install
    WORKING_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION}/../../
)
