# deploy/tests/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(go2_runtime_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(runtime_tests
  runtime/test_main.cpp
  runtime/test_keyboard_map.cpp
  runtime/test_command_mux.cpp
  runtime/test_safety.cpp
  runtime/test_behavior.cpp
  runtime/test_replay.cpp)

target_include_directories(runtime_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_compile_options(runtime_tests PRIVATE -Wall -Wextra)

add_test(NAME runtime_tests COMMAND runtime_tests)
