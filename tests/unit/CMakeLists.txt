# ──────────────────────────────────────────────────────────────────────────
# Unit Tests
# ──────────────────────────────────────────────────────────────────────────

add_executable(test_ring_buffer test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer PRIVATE af_core Catch2::Catch2WithMain)
catch_discover_tests(test_ring_buffer)

add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid PRIVATE af_core Catch2::Catch2WithMain)
catch_discover_tests(test_pid)

add_executable(test_kalman test_kalman.cpp)
target_link_libraries(test_kalman PRIVATE af_core Catch2::Catch2WithMain)
catch_discover_tests(test_kalman)
