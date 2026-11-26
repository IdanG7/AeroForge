# ──────────────────────────────────────────────────────────────────────────
# Integration Tests
# ──────────────────────────────────────────────────────────────────────────

add_executable(test_detector_colorball test_detector_colorball.cpp)
target_link_libraries(test_detector_colorball PRIVATE af_pipeline Catch2::Catch2WithMain)
catch_discover_tests(test_detector_colorball)

add_executable(test_tracking_pipeline test_tracking_pipeline.cpp)
target_link_libraries(test_tracking_pipeline PRIVATE af_pipeline Catch2::Catch2WithMain)
catch_discover_tests(test_tracking_pipeline)

# ──────────────────────────────────────────────────────────────────────────
# Headless smoke tests of the real application (no window, no camera)
# ──────────────────────────────────────────────────────────────────────────

add_executable(af_make_fixture ../tools/make_fixture.cpp)
target_link_libraries(af_make_fixture PRIVATE af_core)

if(AEROFORGE_BUILD_APPS)
  set(AF_FIXTURE_VIDEO "${CMAKE_CURRENT_BINARY_DIR}/synthetic_tracking.avi")
  set(AF_CONFIG_DIR "${CMAKE_SOURCE_DIR}/configs")

  add_test(NAME smoke.make_fixture COMMAND af_make_fixture "${AF_FIXTURE_VIDEO}" 90)
  set_tests_properties(smoke.make_fixture PROPERTIES FIXTURES_SETUP af_video)

  # Replays the synthetic clip and follows the target selected with --roi
  # (the target starts at x=100, y=120 and is 64 px square, see tests/support/synthetic.hpp)
  add_test(NAME smoke.headless_replay
    COMMAND aeroforge-track --headless
            --config "${AF_CONFIG_DIR}/track_interactive.yml"
            --input "${AF_FIXTURE_VIDEO}" --roi 100,120,64,64)
  set_tests_properties(smoke.headless_replay PROPERTIES
    FIXTURES_REQUIRED af_video
    PASS_REGULAR_EXPRESSION "\"frames\": 90[^}]*\"frames_fresh_target\": (89|90)"
    TIMEOUT 120)

  add_test(NAME smoke.help COMMAND aeroforge-track --help)
  set_tests_properties(smoke.help PROPERTIES PASS_REGULAR_EXPRESSION "--headless")

  # Failure paths must exit non-zero with a message, never crash or hang
  add_test(NAME smoke.missing_config
    COMMAND aeroforge-track --headless --config does_not_exist.yml)
  add_test(NAME smoke.missing_video
    COMMAND aeroforge-track --headless
            --config "${AF_CONFIG_DIR}/track_interactive.yml" --input does_not_exist.avi)
  add_test(NAME smoke.bad_argument COMMAND aeroforge-track --no-such-flag)
  set_tests_properties(smoke.missing_config smoke.missing_video smoke.bad_argument PROPERTIES
    WILL_FAIL TRUE TIMEOUT 60)
endif()
