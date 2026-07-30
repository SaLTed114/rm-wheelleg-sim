cmake_minimum_required(VERSION 3.22)

project(rm_balance_sim LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(MuJoCo REQUIRED)

function(balance_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

function(balance_configure_mujoco_runtime target)
  if(WIN32 AND MUJOCO_RUNTIME)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_RUNTIME}"
        "$<TARGET_FILE_DIR:${target}>"
      VERBATIM)
  elseif(MUJOCO_RUNTIME_DIR)
    set_property(TARGET ${target} PROPERTY BUILD_RPATH "${MUJOCO_RUNTIME_DIR}")
  endif()
endfunction()

add_library(balance_control_core STATIC
  src/control/control_core.c)
target_include_directories(balance_control_core
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
balance_enable_warnings(balance_control_core)

add_library(balance_simulation STATIC
  src/sim/mujoco_adapter.cpp
  src/sim/mujoco_plant.cpp
  src/sim/simulation_runner.cpp)
target_include_directories(balance_simulation
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sim")
target_link_libraries(balance_simulation
  PUBLIC balance_control_core mujoco::mujoco)
balance_enable_warnings(balance_simulation)

add_executable(rm_balance_sim src/sim/main.cpp)
target_link_libraries(rm_balance_sim PRIVATE balance_simulation)
balance_enable_warnings(rm_balance_sim)
balance_configure_mujoco_runtime(rm_balance_sim)

include(CTest)
if(BUILD_TESTING)
  add_executable(control_core_test tests/control_core_test.c)
  target_link_libraries(control_core_test PRIVATE balance_control_core)
  balance_enable_warnings(control_core_test)
  add_test(NAME control_core_zero_output COMMAND control_core_test)

  add_executable(mujoco_smoke_test tests/mujoco_smoke_test.cpp)
  target_link_libraries(mujoco_smoke_test PRIVATE balance_simulation)
  balance_enable_warnings(mujoco_smoke_test)
  balance_configure_mujoco_runtime(mujoco_smoke_test)
  add_test(NAME mujoco_model_smoke
    COMMAND mujoco_smoke_test
      "${CMAKE_CURRENT_SOURCE_DIR}/models/MJCF/COD-2026RoboMaster-Balance.xml")
endif()
