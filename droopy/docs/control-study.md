# Bounded 9 lb control study

## Scope and reproduction

This is a **23-trial, deterministic comparison**, not a controller calibration or
an all-direction solution. It uses the illustrative 9 lb preset and only sends
rotor-speed targets through the existing motor/contact physics. It does not add a
yaw force, alter state, or script propulsion. Production controller code and
defaults were not changed.

Run from `Capstone/droopy` (not the Git repository root):

```sh
/home/Sam/src/Capstone/.venv/bin/python tools/control_study.py \
  --output docs/control-study.json
```

Each trial covers four complete gait cycles. The JSON records every trial's full
physics config and command, requested direction, cycle-level metrics, waveform
half-cycle aggregates, torque saturation, command clipping, and the exact
reproduction command. The default open-loop and fixed −15 bias cases, plus `gentle_open` and
`gentle_cycleavg_request_-160`, are repeated at 0.25 ms (half the normal 0.5 ms timestep).

Here “mean speed” means net displacement divided by elapsed time, not path
length divided by time. Cycle windows include a one-tick boundary tolerance;
per-cycle totals therefore need not sum exactly to run totals.

The requested heading used for steering comparisons is world -160° (and, for a
sign check, +160°). Heading error below is displacement-direction error, not body
yaw error. “Instant” is the existing controller currently documented as tuned
only for the original 3 lb model, run unchanged on 9 lb. “Cycle-average” is an
experimental script-only variant that holds one bias for a cycle based on the
previous cycle's circular-mean body yaw.

## Main measurements

| Trial | distance (m) | mean speed (m/s) | travel heading | requested-heading error | net yaw | later-cycle yaw p-p | max tilt | torque saturated | clipped |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 9 lb open loop | 0.242 | 0.050 | 176.2° | -23.8° to -160° | +31.1° | 83.8° | 3.5° | 2.3% | 0% |
| fixed bias -15 rad/s | 0.241 | 0.050 | -167.1° | **-7.1°** | +58.4° | 85.7° | 3.0° | 2.3% | 0% |
| fixed bias +15 rad/s | 0.251 | 0.052 | 178.0° | -22.0° to -160° | +36.3° | 85.0° | 3.7° | 2.6% | 0% |
| existing instant, request -160° | 0.213 | 0.044 | -173.2° | -13.2° | +43.0° | 78.4° | 2.3° | 2.1% | 0% |
| existing instant, request +160° | 0.220 | 0.046 | 169.8° | +9.8° | +23.2° | 79.9° | 3.6° | 2.2% | 0% |
| gentle open (0.75 amplitude, 0.80 frequency) | 0.069 | 0.011 | 164.7° | -35.3° to -160° | -4.9° | 42.9° | 2.6° | 1.7% | 0% |
| gentle fixed -15 | 0.069 | 0.011 | 178.9° | -21.1° | +12.1° | 42.9° | 2.9° | 1.7% | 0% |
| gentle instant, request -160° | 0.052 | 0.009 | 174.5° | -25.5° | +2.3° | 36.2° | 2.8° | 1.6% | 0% |
| gentle cycle-average, request -160° | 0.066 | 0.011 | 171.7° | -28.3° | +3.9° | 43.0° | 2.6° | 1.7% | 0% |

All 23 trials stayed upright by the study's 45° criterion; the observed maximum
was 4.1°. No command clipped at the physical speed cap. Saturation is brief
startup/transient motor saturation, not a persistent command-limit problem.
Therefore increasing limits is not supported as the next control improvement.

### Startup versus established cycles

The open-loop startup cycle moved 0.056 m and changed yaw +45.2°; later cycles
moved 0.054, 0.067, and 0.065 m with yaw changes -10.4°, -3.2°, and -0.6°.
The body nevertheless continued an approximately 84° peak-to-peak yaw swing each
later cycle. Thus final/net yaw conceals large periodic body yaw, while net
translation settles quickly.

With fixed -15 rad/s, startup traveled at +166.2° and changed yaw +50.6°, but the
three later cycles traveled -160.5°, -160.6°, and -159.9°. This is strong evidence
that this small negative bias has useful **established-cycle** authority near the
-160° target, but it is not an immediate startup steering response.

The unchanged 3 lb-tuned instantaneous controller reached mean biases of -10.3,
-4.3, -3.8, and -3.8 rad/s over its four cycles for the -160° request. It improved
alignment over open loop, but less than fixed -15 and with about 12% less speed.
It is effective in this narrow case, not ineffective, but the result does not
validate its gain for 9 lb or other headings.

### Control sign and gait phase

The sign response is nonlinear and asymmetric:

* At the -160° test point, fixed -15 rad/s changed travel from +176.2° to -167.1°;
  -30 overshot to -146.7°. Positive +15 barely changed the displacement heading,
  while +30 moved it strongly to +146.1°.
* The existing feedback controller gave errors of -13.2° for request -160° and
  +9.8° for request +160°. Those two points establish local bidirectional effect,
  not monotonic 360° controllability.
* Under open loop, positive- and negative-sine half cycles displaced in almost
  the same direction (176.5° and 175.9°), even though their aggregated body-yaw
  changes were -286° and +317°. With fixed -15, half-cycle displacement headings
  were -160.0° and -173.3°. This directly distinguishes oscillatory body yaw from
  actual displacement and shows that useful bias authority is distributed across
  both waveform halves rather than being a fake yaw impulse.

### Slower/lower-amplitude candidates

Reduced gaits suppress yaw oscillation but sacrifice too much translation in this
small sweep:

| Candidate | distance | speed | later yaw p-p | change versus open-loop speed |
|---|---:|---:|---:|---:|
| gentle: 0.75 amplitude, 0.80 frequency | 0.069 m | 0.011 m/s | 42.9° | -77% |
| slow: 0.75 amplitude, 0.65 frequency | 0.032 m | 0.005 m/s | 30.1° | -90% |
| low: 0.60 amplitude, 0.80 frequency | 0.021 m | 0.003 m/s | 21.7° | -93% |

Cycle averaging did not rescue the gentle gait. For -160°, it reduced error only
from -35.3° to -28.3° and did not reduce yaw oscillation; instantaneous feedback
reached -25.5° but reduced distance further. For +160°, cycle averaging ended at
164.2° (4.2° error), but gentle open loop is already 164.7°, so that is not useful
causal evidence for feedback.

The half-dt repeat is unusually consistent for these selected trials: gentle open
was 0.069 m at 164.7° at both timesteps; gentle cycle-average was 0.066 m at 171.7°
versus 171.8°. Later yaw p-p differed by about 0.02°. This supports the comparison
for these candidates only; it does not establish global contact-model convergence.

The parent independently repeated the **default** open-loop and fixed −15 bias
cases at half timestep. Open loop gave 0.243 m at +173.62°; fixed −15 gave
0.237 m at −163.93°. The local bias benefit survives the smaller timestep, but
the exact heading shifts by a few degrees; it remains a model-dependent result.

## Ranked next improvements

1. **Next experiment: cycle-synchronous 9 lb displacement feedback, retaining default gait magnitude.**
   This displacement-feedback controller has not yet been implemented or proven;
   the tested cycle-average controller used body yaw, not displacement.
   The best narrow result is a small negative bias after startup: fixed -15 gives
   7.1° error with essentially unchanged speed. Estimate bias from completed-cycle
   displacement direction (or phase-binned contact/displacement), not raw
   instantaneous body yaw, because body yaw swings roughly 84° each cycle.
2. **Add explicit startup handling and a deadband/hold.** The first cycle behaves
   materially differently. Hold zero or a conservative preselected bias through
   one complete cycle, then update once per cycle from displacement evidence.
   Do not infer sign from the startup yaw transient.
3. **Map local bias response by requested sector before changing gain.** The
   -15/-30 and +15/+30 results are not linear or symmetric. A bounded follow-up
   should repeat nearby headings and initial phases, and should include opposite
   quadrants before claiming more than local controllability.
4. **Do not adopt the reduced gait candidates solely for control.** They buy lower
   body-yaw oscillation at a 77–93% speed penalty, and steering remains weak.
   A smaller reduction between 0.75 and 1.0 amplitude may be worth a later sweep
   only if reduced visual/mechanical oscillation is itself a requirement.

## Unknowns and limits

There are no disturbance, friction, initial gait-phase, parameter-uncertainty, or
random-seed repeats; determinism here is not robustness. Only two requested
headings were feedback-tested. The model lacks measured 9 lb hardware properties,
motor/electrical dynamics, foot compliance validation, impacts, and terrain.
Truth yaw and truth velocity would also require practical sensing/estimation on a
robot. Consequently, the scoped finding is only that **small gait bias has useful
local steering authority in the 9 lb simulation, while instantaneous yaw feedback
and simply slowing the gait are inferior in the tested cases**.
