file(MAKE_DIRECTORY "${OUTPUT_DIR}")
set(INPUT "${OUTPUT_DIR}/input.png")
execute_process(COMMAND "${VERIFY}" --make-input "${INPUT}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "cannot generate recording fixture")
endif()
set(video "${OUTPUT_DIR}/raw video.avi")
file(REMOVE "${video}")
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --loop --max-frames 4 --record "${video}" --record-fps 25 --record-codec FFV1
  --summary "${OUTPUT_DIR}/summary.txt" --debug-dir "${OUTPUT_DIR}/debug"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 30)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "recording failed: ${result} ${output} ${error}")
endif()
execute_process(COMMAND "${VERIFY}" "${video}" 4 25 "${INPUT}" "${OUTPUT_DIR}/debug/frame_000000.png"
  RESULT_VARIABLE result TIMEOUT 30)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "recorded frames are not the raw input, or video was not sealed")
endif()
file(SHA256 "${video}" before)
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --record "${video}" RESULT_VARIABLE result ERROR_VARIABLE error TIMEOUT 10)
file(SHA256 "${video}" after)
if(NOT result EQUAL 1 OR NOT error MATCHES "already exists" OR NOT before STREQUAL after)
  message(FATAL_ERROR "existing recording was not protected: ${result} ${error}")
endif()
# CSV must not truncate the existing recording before the recorder checks it.
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --record "${video}" --log "${video}" RESULT_VARIABLE result ERROR_VARIABLE error TIMEOUT 10)
file(SHA256 "${video}" after)
if(NOT result EQUAL 1 OR NOT before STREQUAL after)
  message(FATAL_ERROR "aliased log overwrote the existing recording: ${result} ${error}")
endif()
file(REMOVE "${OUTPUT_DIR}/alias.avi")
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --record "${OUTPUT_DIR}/alias.avi" --summary "${OUTPUT_DIR}/./alias.avi"
  RESULT_VARIABLE result ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 1 OR NOT error MATCHES "conflicts" OR EXISTS "${OUTPUT_DIR}/alias.avi")
  message(FATAL_ERROR "summary/video path conflict was not rejected: ${result} ${error}")
endif()
file(REMOVE "${OUTPUT_DIR}/unsupported.avi")
execute_process(COMMAND "${APP}" --input image --source "${INPUT}" --dry-run
  --record "${OUTPUT_DIR}/unsupported.avi" --record-codec ZZZZ
  RESULT_VARIABLE result ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 1 OR NOT error MATCHES "encoder" OR EXISTS "${OUTPUT_DIR}/unsupported.avi")
  message(FATAL_ERROR "unsupported encoder did not fail clearly: ${result} ${error}")
endif()
# Missing display must be diagnosed before trying to open physical hardware.
execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY
  "${APP}" --input mvs --dry-run --show-debug
  RESULT_VARIABLE result ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 1 OR NOT error MATCHES "desktop display|unavailable in this build")
  message(FATAL_ERROR "headless preview diagnosis missing: ${result} ${error}")
endif()
