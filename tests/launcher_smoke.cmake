# Catch loss of root-relative paths, quoted arguments, config precedence and exit codes.
file(MAKE_DIRECTORY "${OUTPUT_DIR}/working elsewhere")
function(launch expected)
  execute_process(COMMAND "${CMAKE_COMMAND}" -E env "SRM_APP=${APP}"
    bash "${ROOT}/run.sh" ${ARGN}
    WORKING_DIRECTORY "${OUTPUT_DIR}/working elsewhere"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 15)
  if(NOT "${result}" STREQUAL "${expected}")
    message(FATAL_ERROR "launcher expected ${expected}, got ${result}: ${output} ${error}")
  endif()
endfunction()

launch(0 --help)
# Relative fixture must resolve from project root, even when invoked elsewhere.
# Keep tests offline/headless even after the user customizes run.conf.
launch(0 image --source tests/data/application_smoke.ppm --max-frames 1 --dry-run
  --no-show-debug --no-start-paused --playback-fps 0
  --debug-dir "${OUTPUT_DIR}/images with spaces"
  --log "${OUTPUT_DIR}/default.csv" --summary "${OUTPUT_DIR}/default.txt")
file(READ "${OUTPUT_DIR}/default.txt" summary)
foreach(expected "processed_frames=1" "dry_run=1" "error_count=0"
    "nonzero_fire_request_count=0" "transport_sent_frame_count=0")
  if(NOT summary MATCHES "(^|\n)${expected}(\n|$)")
    message(FATAL_ERROR "default configuration: missing ${expected}")
  endif()
endforeach()
if(NOT EXISTS "${OUTPUT_DIR}/images with spaces/frame_000000.png")
  message(FATAL_ERROR "quoted output directory was not preserved")
endif()

# Explicit config replaces the default; CLI still overrides config values.
file(WRITE "${OUTPUT_DIR}/custom config.conf"
  "input.type=image\ninput.source=tests/data/application_smoke.ppm\ninput.loop=true\nruntime.max_frames=3\nruntime.dry_run=true\n")
launch(0 --config "${OUTPUT_DIR}/custom config.conf" --max-frames 2
  --summary "${OUTPUT_DIR}/custom.txt")
file(READ "${OUTPUT_DIR}/custom.txt" summary)
if(NOT summary MATCHES "(^|\n)processed_frames=2(\n|$)")
  message(FATAL_ERROR "CLI did not override custom config")
endif()

launch(1 image --source "${OUTPUT_DIR}/missing.png" --debug-dir "${OUTPUT_DIR}/missing-images")
launch(2 unknown-mode)
launch(1 --config "${OUTPUT_DIR}/missing.conf")

if(DEFINED SDK AND NOT SDK STREQUAL "")
  launch(0 list --mvs-library "${SDK}")
  launch(0 info --mvs-library "${SDK}")
  # Realtime presets clear pause/pacing; headless tests explicitly disable preview.
  file(WRITE "${OUTPUT_DIR}/gui.conf"
    "input.type=video\ninput.source=demo.avi\nruntime.dry_run=true\nruntime.show_debug=true\nruntime.start_paused=true\nruntime.playback_fps=10\n")
  launch(0 mvs --mvs-library "${SDK}" --config "${OUTPUT_DIR}/gui.conf"
    --max-frames 2 --no-show-debug --debug-dir "${OUTPUT_DIR}/mvs-images" --log "${OUTPUT_DIR}/mvs.csv"
    --summary "${OUTPUT_DIR}/mvs.txt")
  file(READ "${OUTPUT_DIR}/mvs.txt" summary)
  foreach(expected "processed_frames=2" "dry_run=1" "error_count=0"
      "transport_sent_frame_count=0" "nonzero_fire_request_count=0")
    if(NOT summary MATCHES "(^|\n)${expected}(\n|$)")
      message(FATAL_ERROR "MVS preset: missing ${expected}")
    endif()
  endforeach()
endif()
