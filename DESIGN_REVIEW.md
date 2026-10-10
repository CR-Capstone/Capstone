# Desktop design review

A read-only headless Claude review compared Droopy against Melty's desktop UI,
interactive vector derivations, telemetry plots, and control-flow diagram.

## Design requirements

- Distinguish **pending UI commands** from the **consumed command and state of a
  physics tick**. Editing while paused must not rewrite history or imply a torque
  was applied when it was not.
- Offer selectable target/actual rotor speed, error, torque, and attitude plots.
  Label units and simulation timestamps; never overlay unlike units against an
  unlabeled common scale. Sampling must not depend on the number of ticks in a
  render batch.
- Explain the complete control chain: gait waveform/manual input → signed
  setpoints → speed clamp → proportional speed error → torque clamp → speed-limit
  gate → reaction torque and rotor dynamics.
- Recompute explanations from retained tick inputs and compare with reported
  physics results. Show source/provenance, numerical substitution, and a named
  pass/fail check rather than only generic equations.
- Draw feedback only where it exists: rotor speed feeds the motor servo. Chassis
  motion does not feed the open-loop gait. Do not imply trajectory control.
- Match Melty's dark palette, toolbar run/step/reset workflow, time/state badge,
  settings separation, and accessible scrollable explanations.
- Settings apply atomically after validation; distinguish restore-default-values
  from resetting the live simulation.
- Export control signals with clear units and timestamps so plots are auditable.

These requirements guide implementation; the project READMEs and tests describe
what is shipped and its limitations. This review is not validation of the physical
Droopy robot or certification of the model.
