file(MAKE_DIRECTORY "${OUTPUT_DIR}")
file(REMOVE "${OUTPUT_DIR}/run.avi" "${OUTPUT_DIR}/failure.avi" "${OUTPUT_DIR}/partial-run.avi")
execute_process(COMMAND "${APP}" --list-cameras --mvs-library "${SDK}"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 0 OR NOT output MATCHES "serial=MOCK-A")
  message(FATAL_ERROR "MVS enumeration failed: ${result} ${output} ${error}")
endif()
execute_process(COMMAND "${APP}" --camera-info --mvs-library "${SDK}"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 0 OR NOT output MATCHES "ExposureTime RW current=1000")
  message(FATAL_ERROR "MVS query failed: ${result} ${output} ${error}")
endif()
execute_process(COMMAND "${APP}" --input mvs --mvs-library "${SDK}" --dry-run
  --max-frames 2 --log "${OUTPUT_DIR}/run.csv" --summary "${OUTPUT_DIR}/run.txt"
  --debug-dir "${OUTPUT_DIR}/debug"
  --record "${OUTPUT_DIR}/run.avi"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "MVS simulated acquisition failed: ${result} ${output} ${error}")
endif()
file(READ "${OUTPUT_DIR}/run.txt" summary)
foreach(expected "processed_frames=2" "error_count=0" "dry_run=1"
    "nonzero_fire_request_count=0" "transport_sent_frame_count=0")
  if(NOT summary MATCHES "(^|\n)${expected}(\n|$)")
    message(FATAL_ERROR "MVS summary missing ${expected}: ${summary}")
  endif()
endforeach()
if(NOT EXISTS "${OUTPUT_DIR}/debug/frame_000000.png")
  message(FATAL_ERROR "MVS debug image missing")
endif()
if(NOT EXISTS "${OUTPUT_DIR}/run.avi" OR NOT error MATCHES "recorded_frames=2")
  message(FATAL_ERROR "MVS recording was not finalized: ${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env SRM_TEST_MVS_FAIL=read
  "${APP}" --input mvs --mvs-library "${SDK}" --dry-run
  --summary "${OUTPUT_DIR}/failure.txt" --log "${OUTPUT_DIR}/failure.csv"
  --record "${OUTPUT_DIR}/failure.avi"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 1 OR NOT error MATCHES "MV_CC_GetImageBuffer failed: 0x80000007")
  message(FATAL_ERROR "MVS timeout did not exit with diagnosis: ${result} ${output} ${error}")
endif()
file(READ "${OUTPUT_DIR}/failure.txt" summary)
foreach(expected "processed_frames=0" "error_count=1" "end_of_stream_count=0"
    "nonzero_fire_request_count=0" "transport_sent_frame_count=0" "safe_count=1")
  if(NOT summary MATCHES "(^|\n)${expected}(\n|$)")
    message(FATAL_ERROR "MVS failure summary missing ${expected}: ${summary}")
  endif()
endforeach()
if(EXISTS "${OUTPUT_DIR}/failure.avi")
  message(FATAL_ERROR "MVS zero-frame failure published an empty recording")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env SRM_TEST_MVS_FAIL=read_after_two
  "${APP}" --input mvs --mvs-library "${SDK}" --dry-run
  --record "${OUTPUT_DIR}/partial-run.avi" --summary "${OUTPUT_DIR}/partial-run.txt"
  RESULT_VARIABLE result ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 1 OR NOT error MATCHES "recorded_frames=2" OR
   NOT EXISTS "${OUTPUT_DIR}/partial-run.avi")
  message(FATAL_ERROR "MVS acquisition error did not seal the previous frames: ${result} ${error}")
endif()
