# ──────────────────────────────────────────────────────────────────────────
# AeroForge Tests
# ──────────────────────────────────────────────────────────────────────────

include(Catch)

# Unit tests
add_subdirectory(unit)

# Integration tests
add_subdirectory(integration)
