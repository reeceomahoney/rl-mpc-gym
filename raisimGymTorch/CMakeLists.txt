cmake_minimum_required(VERSION 3.10)
project(a1_env)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/raisimGymTorch/env/bin)

####################
### dependencies ###
####################
add_subdirectory(../libs/pybind11 pybind11)

list(APPEND CMAKE_PREFIX_PATH ../libs/raisim)
find_package(raisim CONFIG REQUIRED)

find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -g")

set(osqp_DIR ${CMAKE_SOURCE_DIR}/../libs/osqp/build)
find_package(osqp REQUIRED)

#######################
### src and linking ###
#######################
pybind11_add_module(${PROJECT_NAME} 
    raisimGymTorch/env/raisim_gym.cpp 
    raisimGymTorch/env/Yaml.cpp
)

add_library(mpclib
    SHARED ${CMAKE_SOURCE_DIR}/../tests/whole_body_controller.cpp 
    ${CMAKE_SOURCE_DIR}/../src/a1.cpp
    ${CMAKE_SOURCE_DIR}/../src/gait_generator.cpp
    ${CMAKE_SOURCE_DIR}/../src/stance_controller.cpp
    ${CMAKE_SOURCE_DIR}/../src/swing_controller.cpp
    ${CMAKE_SOURCE_DIR}/../src/mpc_osqp.cc
    ${CMAKE_SOURCE_DIR}/../src/locomotion_controller.cpp

    ${CMAKE_SOURCE_DIR}/../include/a1.hpp
    ${CMAKE_SOURCE_DIR}/../include/gait_generator.hpp
    ${CMAKE_SOURCE_DIR}/../include/stance_controller.hpp
    ${CMAKE_SOURCE_DIR}/../include/swing_controller.hpp
    ${CMAKE_SOURCE_DIR}/../include/mpc_osqp.hpp
    ${CMAKE_SOURCE_DIR}/../include/locomotion_controller.hpp
)

target_include_directories(mpclib
    PUBLIC ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/../libs/eigen3/include
    ${CMAKE_SOURCE_DIR}/../libs/raisim/include 
    ${CMAKE_SOURCE_DIR}/../libs/osqp/include 
    ${CMAKE_SOURCE_DIR}/../libs/qpoases/include
)

target_link_libraries(mpclib 
    PRIVATE raisim::raisim
    PRIVATE osqp::osqp
    ${CMAKE_SOURCE_DIR}/../libs/qpoases/build/libs/libqpOASES.a
)

target_include_directories(${PROJECT_NAME} 
    PUBLIC ${CMAKE_SOURCE_DIR}/raisimGymTorch/env 
    ${CMAKE_SOURCE_DIR}/raisimGymTorch/env/envs/a1_env 
    ${CMAKE_SOURCE_DIR}/../libs/eigen3/include 
    ${CMAKE_SOURCE_DIR}/../tests 
    ${CMAKE_SOURCE_DIR}/../include 
    ${CMAKE_SOURCE_DIR}/../libs/raisim/include 
    ${CMAKE_SOURCE_DIR}/../libs/osqp/include 
    ${CMAKE_SOURCE_DIR}/../libs/qpoases/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE mpclib)

target_compile_options(${PROJECT_NAME} PRIVATE -fPIC -O3 -g -mno-avx2)
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DRAISIMGYM_TORCH_ENV_NAME=${PROJECT_NAME}")
message("raisim os ${RAISIM_OS}")
message("raisim arch ${RAISIM_ARCH}")

if (CMAKE_BUILD_TYPE STREQUAL "DEBUG" OR CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("[RAISIM_GYM] BUILDING THE DEBUG APP for ${PROJECT_NAME}")
    add_executable(${PROJECT_NAME}_debug_app raisimGymTorch/env/debug_app.cpp raisimGymTorch/env/Yaml.cpp)
    target_link_libraries(${PROJECT_NAME}_debug_app 
        PRIVATE mpclib
        PRIVATE raisim::raisim)
    target_include_directories(${PROJECT_NAME}_debug_app 
        PUBLIC ${CMAKE_SOURCE_DIR}/raisimGymTorch/env 
        ${CMAKE_SOURCE_DIR}/raisimGymTorch/env/envs/a1_env 
        ${CMAKE_SOURCE_DIR}/../libs/eigen3/include 
        ${CMAKE_SOURCE_DIR}/../tests 
        ${CMAKE_SOURCE_DIR}/../include 
        ${CMAKE_SOURCE_DIR}/../libs/raisim/include 
        ${CMAKE_SOURCE_DIR}/../libs/osqp/include 
        ${CMAKE_SOURCE_DIR}/../libs/qpoases/include
    )
    target_compile_options(${PROJECT_NAME}_debug_app PRIVATE -fPIC -g -O0 -mno-avx2)
endif()