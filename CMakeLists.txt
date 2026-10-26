cmake_minimum_required(VERSION 3.16)

project(shu_auto_aim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SRM_ENABLE_GUI "Build optional debug windows when OpenCV HighGUI is available" ON)
if(SRM_ENABLE_GUI)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d dnn OPTIONAL_COMPONENTS highgui)
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d dnn)
endif()

# 算法和输入库只链接基础组件，HighGUI 仅由应用窗口使用。
set(SRM_OPENCV_LIBS ${OpenCV_LIBS})
list(REMOVE_ITEM SRM_OPENCV_LIBS opencv_highgui)

option(SRM_ENABLE_MVS "Build optional Hikrobot MVS USB camera input" ON)
if(SRM_ENABLE_MVS AND UNIX AND NOT APPLE)
  find_path(SRM_MVS_INCLUDE_DIR MvCameraControl.h
    HINTS /opt/MVS/include /opt/MvCamCtrlSDK/include
          "${CMAKE_CURRENT_SOURCE_DIR}/sp_vision_25-main/io/hikrobot/include")
  if(SRM_MVS_INCLUDE_DIR STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}/sp_vision_25-main/io/hikrobot/include")
    message(STATUS "SRM MVS headers: reference fallback; full SDK 4.7 ABI requires matching development headers (see docs/deployment.md)")
  endif()
endif()

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
  src/io/image_geometry.cpp
  src/io/frame_source.cpp
  src/io/mvs_camera.cpp
)

target_include_directories(srm_frame_source
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(srm_frame_source PUBLIC ${SRM_OPENCV_LIBS})
if(SRM_ENABLE_MVS AND UNIX AND NOT APPLE AND SRM_MVS_INCLUDE_DIR)
  target_compile_definitions(srm_frame_source PRIVATE SRM_HAVE_MVS=1)
  target_include_directories(srm_frame_source SYSTEM PRIVATE "${SRM_MVS_INCLUDE_DIR}")
  target_link_libraries(srm_frame_source PRIVATE ${CMAKE_DL_LIBS})
  message(STATUS "SRM MVS input: enabled (runtime loaded on demand); headers: ${SRM_MVS_INCLUDE_DIR}")
else()
  message(STATUS "SRM MVS input: unavailable (image/video/OpenCV supported)")
  if(SRM_ENABLE_MVS AND UNIX AND NOT APPLE)
    message(STATUS "For MVS input, set SRM_MVS_INCLUDE_DIR to the SDK development include directory")
  endif()
endif()

target_compile_options(srm_frame_source PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)

add_library(srm_traditional_detector STATIC
  src/vision/traditional_detector.cpp
  src/vision/number_classifier.cpp
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
  src/vision/rotating_target.cpp
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
  src/control/fixed_guard.cpp
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
  src/app/terminal_quit_input.cpp
  src/app/video_recorder.cpp
  src/app/application_options.cpp
  src/app/runtime_csv_logger.cpp
  src/app/runtime_statistics.cpp
  src/app/tracking_debug.cpp
)
target_compile_definitions(srm_app_support PRIVATE
  SRM_DEFAULT_NUMBER_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/armor_number")

add_library(srm_camera_calibration STATIC src/vision/camera_calibration.cpp)
target_include_directories(srm_camera_calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(srm_camera_calibration PUBLIC srm_pose_solver srm_frame_source)
target_compile_options(srm_camera_calibration PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
add_executable(srm_calibrate_camera src/app/calibrate_camera.cpp)
target_link_libraries(srm_calibrate_camera PRIVATE srm_camera_calibration)
target_compile_options(srm_calibrate_camera PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)

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
    srm_camera_calibration
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

option(SRM_ENABLE_ROS2_SIMULATOR "Build optional ROS 2 simulator auto aim" OFF)
if(SRM_ENABLE_ROS2_SIMULATOR)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(rm_interfaces REQUIRED)
  add_library(srm_ros_image STATIC src/io/ros_image.cpp)
  target_include_directories(srm_ros_image PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(srm_ros_image PUBLIC srm_camera_calibration)
  ament_target_dependencies(srm_ros_image PUBLIC sensor_msgs)
  add_library(srm_simulator_control STATIC src/io/simulator_control.cpp)
  target_link_libraries(srm_simulator_control PUBLIC srm_safety_controller srm_ros_image)
  ament_target_dependencies(srm_simulator_control PUBLIC rm_interfaces tf2_msgs)
  add_executable(srm_simulator_app src/app/simulator_main.cpp src/app/debug_viewer.cpp)
  target_link_libraries(srm_simulator_app PRIVATE srm_simulator_control srm_app_support srm_traditional_detector)
  target_compile_definitions(srm_simulator_app PRIVATE
    SRM_DEFAULT_NUMBER_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/armor_number")
  ament_target_dependencies(srm_simulator_app PUBLIC rclcpp sensor_msgs)
  if(SRM_ENABLE_GUI AND OpenCV_highgui_FOUND)
    target_compile_definitions(srm_simulator_app PRIVATE SRM_HAVE_HIGHGUI=1)
    target_link_libraries(srm_simulator_app PRIVATE opencv_highgui)
  endif()
  foreach(target srm_ros_image srm_simulator_control srm_simulator_app)
    target_compile_options(${target} PRIVATE
      $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
  endforeach()
endif()

include(CTest)
if(BUILD_TESTING)
  if(SRM_ENABLE_ROS2_SIMULATOR)
    add_executable(srm_ros_image_test tests/ros_image_test.cpp)
    target_link_libraries(srm_ros_image_test PRIVATE srm_ros_image)
    add_test(NAME srm_ros_image_test COMMAND srm_ros_image_test)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME srm_simulator_ros_smoke_test COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/simulator_ros_smoke.py
      --launcher ${CMAKE_CURRENT_SOURCE_DIR}/run.sh
      --output ${CMAKE_CURRENT_BINARY_DIR}/simulator_ros_smoke)
    set_tests_properties(srm_simulator_ros_smoke_test PROPERTIES TIMEOUT 30
      ENVIRONMENT "SRM_SIM_APP=$<TARGET_FILE:srm_simulator_app>")
    add_test(NAME srm_simulator_control_smoke_test COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/simulator_control_smoke.py
      --launcher ${CMAKE_CURRENT_SOURCE_DIR}/run.sh
      --output ${CMAKE_CURRENT_BINARY_DIR}/simulator_control_smoke)
    set_tests_properties(srm_simulator_control_smoke_test PROPERTIES TIMEOUT 30
      ENVIRONMENT "SRM_SIM_APP=$<TARGET_FILE:srm_simulator_app>")
  endif()
  add_executable(srm_rotating_target_test tests/rotating_target_test.cpp)
  target_link_libraries(srm_rotating_target_test PRIVATE srm_target_tracker)
  add_test(NAME srm_rotating_target_test COMMAND srm_rotating_target_test)
  add_executable(srm_terminal_quit_test tests/terminal_quit_test.cpp)
  target_link_libraries(srm_terminal_quit_test PRIVATE srm_app_support util)
  add_test(NAME srm_terminal_quit_test COMMAND srm_terminal_quit_test)
  add_executable(srm_calibration_test tests/calibration_test.cpp)
  target_link_libraries(srm_calibration_test PRIVATE srm_camera_calibration)
  add_test(NAME srm_calibration_test COMMAND srm_calibration_test)
  add_test(NAME srm_calibration_tool_smoke_test COMMAND ${CMAKE_COMMAND}
    -DAPP=$<TARGET_FILE:srm_auto_aim_app>
    -DTOOL=$<TARGET_FILE:srm_calibrate_camera>
    -DFIXTURES=$<TARGET_FILE:srm_calibration_test>
    -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/calibration_tool_smoke
    -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/calibration_tool_smoke.cmake)
  add_test(NAME srm_calibration_app_smoke_test COMMAND ${CMAKE_COMMAND}
    -DAPP=$<TARGET_FILE:srm_auto_aim_app>
    -DINPUT=${CMAKE_CURRENT_SOURCE_DIR}/tests/data/application_smoke.ppm
    -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/calibration_smoke
    -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/calibration_smoke.cmake)
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

  if(SRM_ENABLE_MVS AND UNIX AND NOT APPLE AND SRM_MVS_INCLUDE_DIR)
    add_library(srm_mvs_mock SHARED tests/mvs_mock_sdk.cpp)
    target_include_directories(srm_mvs_mock SYSTEM PRIVATE "${SRM_MVS_INCLUDE_DIR}")
    add_library(srm_mvs_missing_symbol SHARED tests/mvs_missing_symbol.cpp)
    add_executable(srm_mvs_camera_test tests/mvs_camera_test.cpp)
    target_include_directories(srm_mvs_camera_test SYSTEM PRIVATE "${SRM_MVS_INCLUDE_DIR}")
    target_link_libraries(srm_mvs_camera_test PRIVATE srm_frame_source srm_mvs_mock)
    # 显式传入真实运行库执行离线像素转换；不纳入无 SDK 的默认 CTest。
    add_executable(srm_mvs_sdk_conversion_test tests/mvs_sdk_conversion_test.cpp)
    target_include_directories(srm_mvs_sdk_conversion_test SYSTEM PRIVATE "${SRM_MVS_INCLUDE_DIR}")
    target_link_libraries(srm_mvs_sdk_conversion_test PRIVATE ${CMAKE_DL_LIBS})
    foreach(target srm_mvs_mock srm_mvs_missing_symbol srm_mvs_camera_test srm_mvs_sdk_conversion_test)
      target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
    endforeach()
    add_test(NAME srm_mvs_camera_test COMMAND srm_mvs_camera_test
      $<TARGET_FILE:srm_mvs_mock> $<TARGET_FILE:srm_mvs_missing_symbol>)
    add_test(NAME srm_mvs_app_smoke_test COMMAND ${CMAKE_COMMAND}
      -DAPP=$<TARGET_FILE:srm_auto_aim_app>
      -DSDK=$<TARGET_FILE:srm_mvs_mock>
      -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/mvs_smoke
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/mvs_smoke.cmake)
  endif()

  add_executable(srm_traditional_detector_test
    tests/traditional_detector_test.cpp
  )
  target_link_libraries(srm_traditional_detector_test PRIVATE srm_traditional_detector)
  target_compile_definitions(srm_traditional_detector_test PRIVATE
    SRM_TEST_NUMBER_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/armor_number")
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

  add_executable(srm_fixed_guard_test
    tests/fixed_guard_test.cpp
  )
  target_link_libraries(srm_fixed_guard_test PRIVATE srm_auto_aim_pipeline)
  target_compile_options(srm_fixed_guard_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
  )
  add_test(NAME srm_fixed_guard_test COMMAND srm_fixed_guard_test)

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

  add_executable(srm_video_recording_test tests/video_recording_test.cpp)
  target_link_libraries(srm_video_recording_test PRIVATE srm_app_support)
  target_compile_options(srm_video_recording_test PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
  add_test(NAME srm_video_recording_test COMMAND srm_video_recording_test)
  add_test(NAME srm_recording_app_smoke_test COMMAND ${CMAKE_COMMAND}
    -DAPP=$<TARGET_FILE:srm_auto_aim_app>
    -DVERIFY=$<TARGET_FILE:srm_video_recording_test>
    -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/recording_smoke
    -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/recording_smoke.cmake)

  add_test(
    NAME srm_application_app_smoke_test
    COMMAND ${CMAKE_COMMAND}
      -DAPP=$<TARGET_FILE:srm_auto_aim_app>
      -DINPUT=${CMAKE_CURRENT_SOURCE_DIR}/tests/data/application_smoke.ppm
      -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/application_smoke
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/application_smoke.cmake
  )

  set(SRM_LAUNCHER_TEST_SDK_ARGS)
  if(TARGET srm_mvs_mock)
    list(APPEND SRM_LAUNCHER_TEST_SDK_ARGS -DSDK=$<TARGET_FILE:srm_mvs_mock>)
  endif()
  add_test(NAME srm_launcher_smoke_test COMMAND ${CMAKE_COMMAND}
    -DROOT=${CMAKE_CURRENT_SOURCE_DIR}
    -DAPP=$<TARGET_FILE:srm_auto_aim_app>
    -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/launcher_smoke
    ${SRM_LAUNCHER_TEST_SDK_ARGS}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/launcher_smoke.cmake)
endif()
