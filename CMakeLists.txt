cmake_minimum_required(VERSION 3.10)
project(whole_body_controller LANGUAGES CXX)

#Set environment variables
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(raisim_DIR ./libs/raisim/lib/cmake/raisim)

find_package(raisim CONFIG REQUIRED)

include_directories(
    ./include 
    ./libs/eigen3/include 
    ./libs/osqp/include
    ./libs/osqp/include/linux
    ./libs/qpoases/include 
)

add_executable(${PROJECT_NAME} 
    main.cpp 
    ./src/a1.cpp
    ./src/gait_generator.cpp
    ./src/stance_controller.cpp
    #./src/mpc_osqp.cc
)

#Link raisim
target_link_libraries(${PROJECT_NAME} PUBLIC raisim::raisim pthread)
target_include_directories(${PROJECT_NAME} PUBLIC ./libs/raisim/include)