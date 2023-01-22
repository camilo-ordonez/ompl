cmake_minimum_required(VERSION 3.13.4)

project(planners)
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ompl)
message(STATUS ${OMPL_INCLUDE_DIRS})



include_directories(${OMPL_INCLUDE_DIRS} ${BASE_INCLUDE_DIRECTORIES})

add_executable(${PROJECT_NAME} RigidBodyPlanning_RRTStar.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${OMPL_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${BASE_LIBRARIES})

#target_link_libraries(${PROJECT_NAME} ompl)



#cmake_minimum_required(VERSION 3.1)
#project(my_library)

#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#include_directories(.)

#add_library(tools tools.cpp)
#add_executable(main main.cpp)
#target_link_libraries(main tools)
