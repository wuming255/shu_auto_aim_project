file(MAKE_DIRECTORY "${OUTPUT_DIR}")
file(WRITE "${OUTPUT_DIR}/camera.yml" "%YAML:1.0\n---\nschema_version: 1\nimage_width: 64\nimage_height: 48\norientation_deg: 0\ncamera_matrix: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [80., 0., 32., 0., 81., 24., 0., 0., 1.]\ndistortion_coefficients: [0.01, -0.02, 0.001, 0.002, 0.]\n")
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --calibration "${OUTPUT_DIR}/camera.yml" --max-frames 1
  --summary "${OUTPUT_DIR}/summary.txt" RESULT_VARIABLE result ERROR_VARIABLE stderr)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "valid full calibration must load: ${stderr}")
endif()
file(READ "${OUTPUT_DIR}/summary.txt" summary)
foreach(expected "processed_frames=1" "nonzero_fire_request_count=0" "calibration_loaded=1" "extrinsics_loaded=0"
    "acquisition_sample_count=1" "detection_sample_count=1" "loop_sample_count=1")
  if(NOT summary MATCHES "${expected}")
    message(FATAL_ERROR "missing ${expected}: ${summary}")
  endif()
endforeach()
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --calibration "${OUTPUT_DIR}/camera.yml" --file-rotation 180 --max-frames 1
  RESULT_VARIABLE result ERROR_VARIABLE stderr)
if(result EQUAL 0 OR NOT stderr MATCHES "orientation")
  message(FATAL_ERROR "orientation mismatch must fail: ${stderr}")
endif()
file(READ "${OUTPUT_DIR}/camera.yml" camera)
string(REPLACE "image_width: 64" "image_width: 128" camera "${camera}")
file(WRITE "${OUTPUT_DIR}/wrong_size.yml" "${camera}")
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --calibration "${OUTPUT_DIR}/wrong_size.yml" --max-frames 1
  RESULT_VARIABLE result ERROR_VARIABLE stderr)
if(result EQUAL 0 OR NOT stderr MATCHES "resolution")
  message(FATAL_ERROR "resolution mismatch must fail: ${stderr}")
endif()
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --live
  --calibration "${OUTPUT_DIR}/camera.yml" RESULT_VARIABLE result ERROR_VARIABLE stderr)
if(result EQUAL 0 OR NOT stderr MATCHES "requires full --calibration and --extrinsics")
  message(FATAL_ERROR "uncalibrated live control must fail before starting transport: ${stderr}")
endif()
# The rejected output must never truncate the calibration input.
file(SHA256 "${OUTPUT_DIR}/camera.yml" before)
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --calibration "${OUTPUT_DIR}/camera.yml" --log "${OUTPUT_DIR}/./camera.yml"
  RESULT_VARIABLE result ERROR_VARIABLE stderr)
file(SHA256 "${OUTPUT_DIR}/camera.yml" after)
if(result EQUAL 0 OR NOT before STREQUAL after)
  message(FATAL_ERROR "calibration input was not protected from aliased logging")
endif()
