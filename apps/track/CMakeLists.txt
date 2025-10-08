# ──────────────────────────────────────────────────────────────────────────
# AeroForge-Track: Follow-mode application
# ──────────────────────────────────────────────────────────────────────────

add_executable(aeroforge-track
  aeroforge-track.cpp
)

target_link_libraries(aeroforge-track
  PRIVATE
    af_core
    af_vision
    af_tracking
    af_estimate
    af_control
)

if(AEROFORGE_WITH_IMGUI)
  target_link_libraries(aeroforge-track PRIVATE af_ui)
endif()

# Set output directory
set_target_properties(aeroforge-track PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Install
install(TARGETS aeroforge-track
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
