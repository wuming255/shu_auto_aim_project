cmake_minimum_required(VERSION 3.16)

project(shu_auto_aim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio calib3d)

add_library(srm_serial STATIC
  src/third_party/serial/src/serial.cc
  src/third_party/serial/src/impl/unix.cc
)

target_include_directories(srm_serial
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/serial/include
)

target_compile_options(srm_serial PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

if(UNIX AND NOT APPLE)
  target_link_libraries(srm_serial PUBLIC pthread rt)
endif()

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

add_library(srm_frame_source STATIC
  src/io/frame_source.cpp
)

target_include_directories(srm_frame_source
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_frame_source PUBLIC ${OpenCV_LIBS})

target_compile_options(srm_frame_source PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_traditional_detector STATIC
  src/vision/traditional_detector.cpp
)

target_include_directories(srm_traditional_detector
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_traditional_detector PUBLIC ${OpenCV_LIBS})

target_compile_options(srm_traditional_detector PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_pose_solver STATIC
  src/vision/pose_solver.cpp
)

target_include_directories(srm_pose_solver
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_pose_solver PUBLIC ${OpenCV_LIBS})

target_compile_options(srm_pose_solver PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_target_selector STATIC
  src/vision/target_selector.cpp
)

target_include_directories(srm_target_selector
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_target_selector
  PUBLIC
    srm_pose_solver
    ${OpenCV_LIBS}
)

target_compile_options(srm_target_selector PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_target_tracker STATIC
  src/vision/target_tracker.cpp
)

target_include_directories(srm_target_tracker
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_target_tracker
  PUBLIC
    srm_target_selector
    ${OpenCV_LIBS}
)

target_compile_options(srm_target_tracker PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_aim_compensator STATIC
  src/control/aim_compensator.cpp
)

target_include_directories(srm_aim_compensator
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(srm_aim_compensator
  PUBLIC
    srm_target_tracker
)

target_compile_options(srm_aim_compensator PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_auto_aim_pipeline STATIC
  src/control/auto_aim_pipeline.cpp
)

target_include_directories(srm_auto_aim_pipeline
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_auto_aim_pipeline
  PUBLIC
    srm_aim_compensator
)

target_compile_options(srm_auto_aim_pipeline PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_transport STATIC
  src/io/self_aim_transport.cpp
)

target_include_directories(srm_transport
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(srm_transport
  PUBLIC
    srm_protocol
    srm_serial
)

target_compile_options(srm_transport PRIVATE
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

  add_executable(srm_transport_test
    tests/self_aim_transport_test.cpp
  )
  target_link_libraries(srm_transport_test PRIVATE srm_transport)
  target_compile_options(srm_transport_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_transport_test COMMAND srm_transport_test)

  add_executable(srm_frame_source_test
    tests/frame_source_test.cpp
  )
  target_link_libraries(srm_frame_source_test PRIVATE srm_frame_source)
  target_compile_options(srm_frame_source_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_frame_source_test COMMAND srm_frame_source_test)

  add_executable(srm_traditional_detector_test
    tests/traditional_detector_test.cpp
  )
  target_link_libraries(srm_traditional_detector_test PRIVATE srm_traditional_detector)
  target_compile_options(srm_traditional_detector_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_traditional_detector_test COMMAND srm_traditional_detector_test)

  add_executable(srm_pose_solver_test
    tests/pose_solver_test.cpp
  )
  target_link_libraries(srm_pose_solver_test PRIVATE srm_pose_solver)
  target_compile_options(srm_pose_solver_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_pose_solver_test COMMAND srm_pose_solver_test)

  add_executable(srm_target_selector_test
    tests/target_selector_test.cpp
  )
  target_link_libraries(srm_target_selector_test PRIVATE srm_target_selector)
  target_compile_options(srm_target_selector_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_target_selector_test COMMAND srm_target_selector_test)

  add_executable(srm_target_tracker_test
    tests/target_tracker_test.cpp
  )
  target_link_libraries(srm_target_tracker_test PRIVATE srm_target_tracker)
  target_compile_options(srm_target_tracker_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_target_tracker_test COMMAND srm_target_tracker_test)

  add_executable(srm_aim_compensator_test
    tests/aim_compensator_test.cpp
  )
  target_link_libraries(srm_aim_compensator_test PRIVATE srm_aim_compensator)
  target_compile_options(srm_aim_compensator_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_aim_compensator_test COMMAND srm_aim_compensator_test)

  add_executable(srm_auto_aim_pipeline_test
    tests/auto_aim_pipeline_test.cpp
  )
  target_link_libraries(srm_auto_aim_pipeline_test PRIVATE srm_auto_aim_pipeline)
  target_compile_options(srm_auto_aim_pipeline_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_auto_aim_pipeline_test COMMAND srm_auto_aim_pipeline_test)
endif()
