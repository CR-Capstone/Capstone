# Droopy — twin-rotor physics sandbox

A separate desktop simulator inspired by Tommy Wong's **Droopy**, alongside the
existing Meltybrain project. Nothing in the original simulator is replaced.
Droopy is an exploratory dynamics model, **not a validated digital twin** of the
real robot. Dimensions, inertia, motor response, feet, and gait are illustrative.

## Run

On this computer:

```sh
cd /home/Sam/src/Capstone/droopy
./run_sim.sh
```

The launcher works from any directory. It uses `droopy/.venv` if present, otherwise
the existing `Capstone/.venv`. It requires no native rebuild or new dependencies
in that existing environment.

For a standalone environment (Python 3.10+):

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
./run_sim.sh
```

### Try it

1. Select **Enable motors**, then **Start**.
2. The default **Alternating gait** applies a sinusoidal speed perturbation to
   counterrotating weapons. Watch body tilt, floor contacts, and the path.
3. Slow **Simulation speed** to 0.05–0.1 to inspect the motion. **Pause** and
   **STEP** (one physics tick) advance the same fixed-step physics, not an animation.
4. Compare **Constant counterspin** with **Alternating gait**, or use signed
   **Manual rotors** targets. Off means active braking, not instant rotor stop.
5. Open **Settings**, change rotor tilt or floor friction, then **Apply**.
   Try zero friction: rotor actuation cannot create horizontal centre-of-mass
   propulsion without an external force from the floor.
6. Use **Plot channels** → **Speed tracking**, **Left servo**, **Right servo**, or
   **Torque only**. Individual channels include raw setpoints, clamped requests,
   actual rotor speeds, servo errors, and torque. Select a 1/5/10-second history
   window and hover for numerical values. Speed, error [rad/s], and torque [N·m]
   have separate plot lanes; error autoscales independently. Dashed limit lines
   mark configured motor limits. Speed tracking defaults to the clamped requests,
   so an unreachable raw demand is not mistaken for servo error. The setpoint explainer distinguishes
   the immutable consumed tick from the current UI demand and expands the gait's
   base ± sine-wave + bias, clamp, servo gain, torque cap, and braking math.
7. **Record CSV** saves telemetry plus a JSON sidecar with configuration and
   accepted command changes. A recording started mid-run is a telemetry segment,
   not an exact restart/replay checkpoint. Reset/apply/close stops recording.

Speed targets are radians/second; telemetry also displays signed RPM. Positive
and negative rates denote opposite rotation about the displayed rotor axes.
The demo uses an illustrative narrow 5 × 5 cm support footprint, 0.8 N·m
motor torque limit, and 220 ± 180 rad/s counterspin modulation at 1 Hz. These
parameters were selected to demonstrate contact-driven motion, not fitted to
Droopy. In a five-second run the default model travels roughly 0.21 m, mostly
in its initial −X direction.

The gait is an experimental open-loop waveform: controls **do not guarantee
forward travel or calibrated steering**. Symmetric geometry may rock, turn, or
shuffle rather than trace a straight path. There is no hidden position control.

## What is simulated?

- Three-dimensional chassis translation and rotation.
- Two independently actuated, outward-tilted rotor angular momenta.
- Bounded motor torque and speed regulation, including braking reaction torque.
- Internal angular-momentum exchange and gyroscopic coupling.
- Gravity and compliant, unilateral floor contacts with dissipative friction.
- A fixed integration timestep independent of rendering.

The yellow points/arrows indicate loaded contacts and contact forces; the green
line indicates chassis forward (+X). Click a contact-force or velocity arrow (or
use the accessible explainer selector) for live equations populated from the exact
immutable pre-step inputs consumed by the latest tick. The detail pane shows units,
substituted numbers, and recomputation residuals for motor torque, Euler/gyroscopic
rotation, contact/friction, translation, and energy. Reset clears that evidence.
The floor grid is 10 cm. The camera follows
horizontal position; the telemetry reports world position. The plot's coloured
legend identifies the selected motor channels. Rotor outlines depict swept regions, not collision geometry. Fast
rotor bars may appear stationary or reverse at display frame rates (aliasing).

There is **no direct motor thrust**, scripted stepping, aerodynamic lift, opponent,
weapon-floor collision, impact model, flexible chassis, electrical/battery model,
or validated embedded controller. The arena is an infinite plane without walls. Mass and rotational inertia are
independent model inputs: changing mass in the interface does not resize the
chassis or automatically rescale its inertia. Advanced geometry/inertia changes
can be explored with `Config` in `droopy/physics.py`.
The rotor model represents balanced, axisymmetric spinning inertia rather than
resolving the instantaneous inertia tensor of each bar. A tipped-over chassis is
not accurately represented by feet-only collision. Stay near the upright regime;
large tilt experiments can leave the model's physically meaningful range.

### Physics interpretation

For a rotor, accelerating its spin transfers equal and opposite angular momentum
to the chassis. Its already-spinning angular momentum must also change direction
when the chassis tilts, producing gyroscopic coupling. These are distinct terms
in the same mechanics, not two independent propulsion forces. Floor friction
provides the external force that can turn rocking and changing contacts into
translation. The detailed derivation and numerical assumptions are in
`droopy/physics.py`. Contact-driven heading is timestep-sensitive: a four-second
comparison at 0.5 ms versus 0.25 ms yielded similar longitudinal displacement
(about −0.167 m) but different yaw (36° versus 27°). Do not treat the plotted
trajectory as a quantitatively converged prediction.

The real robot's exact gait and geometry are not published in the sources used:

- [NHRL Droopy entry](https://wiki.nhrl.io/wiki/index.php?title=Droopy)
- [NHRL explanation, from 2:06](https://www.youtube.com/watch?v=SAQ7xzr948A&t=126s)
- [Combat Robot Hall of Fame](https://runamok.tech/hall_of_fame.html), describing
  Droopy as a torque-reaction shuffle-walker.

## Headless runs and checks

Use `.venv/bin/python` instead of `../.venv/bin/python` below if you created a
standalone environment. Commands run from this directory.

```sh
# Deterministic open-loop experiment (JSON summary; optional CSV + metadata)
../.venv/bin/python -m droopy.cli --seconds 5 --csv /tmp/droopy.csv
../.venv/bin/python -m droopy.cli --seconds 2 --mode off
../.venv/bin/python -m droopy.cli --seconds 2 --friction 0

# Unit tests plus offscreen UI lifecycle test
QT_QPA_PLATFORM=offscreen ../.venv/bin/python -m pytest -q tests

# Render /tmp/droopy-ui.png and exit without opening a desktop window
QT_QPA_PLATFORM=offscreen ./run_sim.sh --smoke
```

Current validation covers conservation, immutable diagnostic formula checks, contact-force
bounds, deterministic runs, the default gait, frictionless-floor behavior, and
UI controls/rendering/recording/shutdown. The offscreen launcher also works when
invoked outside this directory.

Plot history freezes while paused and clears on reset or settings apply; stepping
adds accepted-tick samples. The plot uses consumption time t₀, with actual speed
measured after that tick. It retains up to 10 simulated seconds (capped at 200,000
samples at small timesteps) and decimates only for painting. The GUI CSV records
every completed tick with post-step `t` and pre-step `consumed_at_s`, plus raw targets, clamped requests,
pre-step errors and post-step actual speeds. The CLI CSV remains a lower-rate
state-telemetry export, not the GUI control-signal format.

The **Physics inspector** tab contains live calculations and named pass/fail
recomputation checks. Arrow clicks and the flow buttons select a topic; the
**Controls & telemetry** tab keeps pending commands distinct from consumed ones.
The flow summary is textual, not a full graphical block diagram. Plots offer
hover values but do not select historical ticks in the physics inspector.
 The desktop limits work per timer callback and discards excess backlog under
heavy load to keep controls responsive. Requested playback speed may exceed what
the computer can sustain; the toolbar shows measured actual playback rate while
running. Simulated time remains the source of truth.
