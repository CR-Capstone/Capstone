# Reorganization and desktop validation

## Melty

- Fresh CMake build in `melty/build` using the shared root environment.
- CTest: **4/4 passed** (phase, runtime, plant, closed loop).
- Python/UI suite: **24 passed**.
- Offscreen smoke launched after relocation and from `/tmp`.
- Original firmware, physics, UI and tests are moved without source changes;
  launcher and documentation updated for the new layout.

## Droopy

- Physics and UI tests cover deterministic motion, momentum conservation,
  contact/friction, default gait, per-tick diagnostics, tamper detection,
  target generation, limits, settings, plots, recording and shutdown.
- Offscreen screenshots inspected for controls, physics inspector, live check
  badges and speed/torque plot lanes.
- Headless CLI and launcher from outside the project directory checked.
- Dense control-signal sampling is independent of display batches; pending edits
  leave consumed diagnostics unchanged.
- Two headless Claude design reviews compared the UI/explanations with Melty.
  See `DESIGN_REVIEW.md` for the review requirements.

## Limits

No hardware tests. The model remains exploratory and contact-driven heading is
sensitive to timestep. Plot display decimates dense history, while stored GUI CSV
signals are per physics tick. Very small timesteps cap retained plot history and
may run below requested playback speed; the UI reports those limits. The flow
summary is textual with topic buttons, not a full graphical block diagram.
