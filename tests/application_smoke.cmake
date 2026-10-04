if(NOT DEFINED APP OR NOT DEFINED INPUT OR NOT DEFINED OUTPUT_DIR)
  message(FATAL_ERROR "APP, INPUT and OUTPUT_DIR must be provided")
endif()

file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")
set(DEBUG_DIR "${OUTPUT_DIR}/debug")
set(LOG_PATH "${OUTPUT_DIR}/run.csv")
set(SUMMARY_PATH "${OUTPUT_DIR}/summary.txt")

execute_process(
  COMMAND "${APP}"
    --input image
    --source "${INPUT}"
    --dry-run
    --intrinsics 800,800,32,24
    --max-frames 1
    --debug-dir "${DEBUG_DIR}"
    --debug-every 1
    --log "${LOG_PATH}"
    --summary "${SUMMARY_PATH}"
  RESULT_VARIABLE RESULT
  OUTPUT_VARIABLE STDOUT_TEXT
  ERROR_VARIABLE STDERR_TEXT
)

if(NOT RESULT EQUAL 0)
  message(FATAL_ERROR
    "offline application smoke test failed with ${RESULT}\nstdout:\n${STDOUT_TEXT}\nstderr:\n${STDERR_TEXT}")
endif()

foreach(REQUIRED_FILE IN ITEMS
    "${DEBUG_DIR}/frame_000000.png"
    "${LOG_PATH}"
    "${SUMMARY_PATH}")
  if(NOT EXISTS "${REQUIRED_FILE}")
    message(FATAL_ERROR "offline smoke output is missing: ${REQUIRED_FILE}")
  endif()
endforeach()

file(READ "${SUMMARY_PATH}" SUMMARY_TEXT)
foreach(REQUIRED_LINE IN ITEMS
    "processed_frames=1"
    "nonzero_fire_request_count=0"
    "dry_run=1")
  string(FIND "${SUMMARY_TEXT}" "${REQUIRED_LINE}" LINE_POSITION)
  if(LINE_POSITION EQUAL -1)
    message(FATAL_ERROR
      "offline summary is missing '${REQUIRED_LINE}'\nsummary:\n${SUMMARY_TEXT}")
  endif()
endforeach()

file(READ "${LOG_PATH}" LOG_TEXT)
string(FIND "${LOG_TEXT}" "fire_flag" HEADER_POSITION)
if(HEADER_POSITION EQUAL -1)
  message(FATAL_ERROR "offline CSV is missing fire_flag column\nCSV:\n${LOG_TEXT}")
endif()

message(STATUS "offline application smoke test passed: ${STDOUT_TEXT}")
