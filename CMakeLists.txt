cmake_minimum_required(VERSION 3.16)

project(shu_auto_aim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(srm_protocol STATIC
  src/protocol/self_aim_protocol.cpp
)

target_include_directories(srm_protocol
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(srm_protocol PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

include(CTest)
if(BUILD_TESTING)
  add_executable(srm_protocol_test
    tests/self_aim_protocol_test.cpp
  )
  target_link_libraries(srm_protocol_test PRIVATE srm_protocol)
  target_compile_options(srm_protocol_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_protocol_test COMMAND srm_protocol_test)
endif()
