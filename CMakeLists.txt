cmake_minimum_required(VERSION 3.16)

project(shu_auto_aim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SRM_ENABLE_GUI "Build optional debug windows when OpenCV HighGUI is available" ON)
if(SRM_ENABLE_GUI)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio calib3d OPTIONAL_COMPONENTS highgui)
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio calib3d)
endif()

# 算法和输入库只链接基础组件，HighGUI 仅由应用窗口使用。
set(SRM_OPENCV_LIBS ${OpenCV_LIBS})
list(REMOVE_ITEM SRM_OPENCV_LIBS opencv_highgui)

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

target_link_libraries(srm_frame_source PUBLIC ${SRM_OPENCV_LIBS})

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

target_link_libraries(srm_traditional_detector PUBLIC ${SRM_OPENCV_LIBS})

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

target_link_libraries(srm_pose_solver PUBLIC ${SRM_OPENCV_LIBS})

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
    ${SRM_OPENCV_LIBS}
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

add_library(srm_safety_controller STATIC
  src/control/safety_controller.cpp
)

target_include_directories(srm_safety_controller
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_safety_controller
  PUBLIC
    srm_auto_aim_pipeline
)

target_compile_options(srm_safety_controller PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_auto_aim_runtime STATIC
  src/control/auto_aim_runtime.cpp
)

target_include_directories(srm_auto_aim_runtime
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_auto_aim_runtime
  PUBLIC
    srm_safety_controller
    srm_transport
    srm_frame_source
)

target_compile_options(srm_auto_aim_runtime PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_app_support STATIC
  src/app/application_options.cpp
  src/app/runtime_csv_logger.cpp
  src/app/runtime_statistics.cpp
)

target_include_directories(srm_app_support
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(srm_app_support
  PUBLIC
    srm_auto_aim_runtime
)

target_compile_options(srm_app_support PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_executable(srm_auto_aim_app
  src/app/main.cpp
  src/app/debug_viewer.cpp
)

target_link_libraries(srm_auto_aim_app
  PRIVATE
    srm_app_support
    srm_traditional_detector
)

if(SRM_ENABLE_GUI AND OpenCV_highgui_FOUND)
  target_compile_definitions(srm_auto_aim_app PRIVATE SRM_HAVE_HIGHGUI=1)
  target_link_libraries(srm_auto_aim_app PRIVATE opencv_highgui)
  message(STATUS "SRM optional debug windows: enabled")
else()
  message(STATUS "SRM optional debug windows: unavailable (headless replay supported)")
endif()

target_compile_options(srm_auto_aim_app PRIVATE
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

  add_executable(srm_safety_controller_test
    tests/safety_controller_test.cpp
  )
  target_link_libraries(srm_safety_controller_test PRIVATE srm_safety_controller)
  target_compile_options(srm_safety_controller_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_safety_controller_test COMMAND srm_safety_controller_test)

  add_executable(srm_auto_aim_runtime_test
    tests/auto_aim_runtime_test.cpp
  )
  target_link_libraries(srm_auto_aim_runtime_test PRIVATE srm_auto_aim_runtime)
  target_compile_options(srm_auto_aim_runtime_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_auto_aim_runtime_test COMMAND srm_auto_aim_runtime_test)

  add_executable(srm_application_support_test
    tests/application_support_test.cpp
  )
  target_link_libraries(srm_application_support_test PRIVATE srm_app_support)
  target_compile_options(srm_application_support_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_application_support_test COMMAND srm_application_support_test)

  add_test(
    NAME srm_application_app_smoke_test
    COMMAND ${CMAKE_COMMAND}
      -DAPP=$<TARGET_FILE:srm_auto_aim_app>
      -DINPUT=${CMAKE_CURRENT_SOURCE_DIR}/tests/data/application_smoke.ppm
      -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/application_smoke
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/application_smoke.cmake
  )
endif()
