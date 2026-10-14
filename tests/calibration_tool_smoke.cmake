file(REMOVE_RECURSE "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")
execute_process(COMMAND "${FIXTURES}" "${OUTPUT_DIR}/fixtures" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "synthetic fixture generation failed")
endif()
# Use only the board images, not asymmetric geometry test images.
file(MAKE_DIRECTORY "${OUTPUT_DIR}/images")
file(GLOB boards "${OUTPUT_DIR}/fixtures/board_*.png")
file(COPY ${boards} DESTINATION "${OUTPUT_DIR}/images")
foreach(mode images video)
  if(mode STREQUAL "images")
    set(source "${OUTPUT_DIR}/images")
  else()
    set(source "${OUTPUT_DIR}/fixtures/boards.avi")
  endif()
  execute_process(COMMAND "${TOOL}" --input "${mode}" --source "${source}"
    --output "${OUTPUT_DIR}/${mode}_result" --board chessboard --columns 8 --rows 6
    --square-size-mm 30 --max-rms 0.6
    RESULT_VARIABLE result OUTPUT_VARIABLE stdout ERROR_VARIABLE stderr)
  if(NOT result EQUAL 0 OR NOT EXISTS "${OUTPUT_DIR}/${mode}_result/camera.yml")
    message(FATAL_ERROR "${mode} calibration failed: ${stdout}\n${stderr}")
  endif()
  execute_process(COMMAND "${APP}" --input image --source "${OUTPUT_DIR}/images/board_0.png"
    --dry-run --calibration "${OUTPUT_DIR}/${mode}_result/camera.yml" --max-frames 1
    --summary "${OUTPUT_DIR}/${mode}_summary.txt" RESULT_VARIABLE result ERROR_VARIABLE stderr)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "generated calibration could not load: ${stderr}")
  endif()
endforeach()
execute_process(COMMAND "${TOOL}" --input images --source "${OUTPUT_DIR}/images"
  --output "${OUTPUT_DIR}/missing_spec" --board chessboard
  RESULT_VARIABLE result ERROR_VARIABLE stderr)
if(result EQUAL 0 OR EXISTS "${OUTPUT_DIR}/missing_spec")
  message(FATAL_ERROR "missing physical board dimensions must fail before creating output")
endif()
