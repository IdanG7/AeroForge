# ──────────────────────────────────────────────────────────────────────────
# Demo Viewer: Simple webcam preview for testing
# ──────────────────────────────────────────────────────────────────────────

add_executable(demo_viewer
  demo_viewer.cpp
)

target_link_libraries(demo_viewer
  PRIVATE
    af_core
)

if(AEROFORGE_WITH_IMGUI)
  target_link_libraries(demo_viewer PRIVATE af_ui)
endif()

set_target_properties(demo_viewer PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
