# ──────────────────────────────────────────────────────────────────────────
# Integration Tests
# ──────────────────────────────────────────────────────────────────────────

add_executable(test_detector_colorball test_detector_colorball.cpp)
target_link_libraries(test_detector_colorball PRIVATE af_core af_vision Catch2::Catch2WithMain)
catch_discover_tests(test_detector_colorball)
