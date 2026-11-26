# ADR 0001: The native C++ tracker is the default; PyTracking is opt-in

Date: 2026-11-26. Status: accepted.

## Problem

The working tree hard-wired the PyTracking detector (a Python subprocess running a
PyTorch tracker) as the only detector. The fallback to the C++ tracker sat in a `catch`
block that could not be reached, because constructing the detector never fails; the
subprocess only starts on first use. On a machine without PyTorch, the pytracking
checkout and the model files, the application opened, showed video and could not track
anything. The module `type` in the config was ignored.

## Alternatives

1. Keep PyTracking as the default and document the Python setup as a requirement.
2. Probe for Python at startup and fall back automatically.
3. Choose the detector from the config; default to the native tracker.

## Decision

Option 3. `type: interactive` (or no detector module at all) selects the native tracker.
`type: pytracking` selects the Python bridge. Both implement `ITargetTracker`, so the
application has one code path for selection, reset and display.

## Why

- A clean checkout must give a working demonstration with only the C++ dependencies.
- Tests and CI cannot depend on a multi-gigabyte Python environment.
- An automatic fallback (option 2) hides which tracker is running; tracking quality and
  latency differ a lot between the two.

## Trade-offs

- The native tracker is template and feature based. It is weaker than DiMP under large
  appearance change, fast rotation or scale change.
- `configs/track_colorball.yml` now really runs the colour detector. Before, every config
  ended up in the interactive tracker regardless of its `type`.

## Limits

The PyTracking bridge is unchanged apart from bounded waits and restart-on-failure. It is
Windows-only in practice and synchronous; see `docs/TECHNICAL_DEBT.md`.
