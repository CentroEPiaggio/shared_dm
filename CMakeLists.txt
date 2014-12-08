cmake_minimum_required(VERSION 2.8)

project(dual_manipulation_shared)


find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#add_library(dual_manipulation_shared main.cpp)
#target_link_libraries(dual_manipulation_shared ${catkin_LIBRARIES})

#install(TARGETS dual_manipulation_shared RUNTIME DESTINATION lib)
