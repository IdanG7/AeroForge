# ──────────────────────────────────────────────────────────────────────────
# Unit Tests
# ──────────────────────────────────────────────────────────────────────────

function(af_add_unit_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE af_pipeline Catch2::Catch2WithMain)
  catch_discover_tests(${name})
endfunction()

af_add_unit_test(test_ring_buffer)
af_add_unit_test(test_pid)
af_add_unit_test(test_kalman)
af_add_unit_test(test_estimators)
af_add_unit_test(test_control_sim)
af_add_unit_test(test_safety_gate)
af_add_unit_test(test_config)
