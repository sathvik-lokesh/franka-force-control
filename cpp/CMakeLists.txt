cmake_minimum_required(VERSION 3.20)
project(franka_force_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# MuJoCo comes from the pip wheel in .venv, Eigen is header-only and fetched (no sudo needed).
set(MUJOCO_DIR "${CMAKE_SOURCE_DIR}/../.venv/lib/python3.12/site-packages/mujoco")

include(FetchContent)
FetchContent_Declare(eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  SOURCE_SUBDIR do_not_configure_eigen)
FetchContent_MakeAvailable(eigen)

add_executable(m5 m5.cpp)
target_include_directories(m5 PRIVATE ${MUJOCO_DIR}/include ${eigen_SOURCE_DIR})
target_link_libraries(m5 PRIVATE ${MUJOCO_DIR}/libmujoco.so.3.17.0)
set_target_properties(m5 PROPERTIES BUILD_RPATH ${MUJOCO_DIR})
target_compile_options(m5 PRIVATE -Wall -Wextra -O3)
