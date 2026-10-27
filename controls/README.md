# Controls — serial controller for a two-motor combat robot

A PySide6 desktop app that drives two ESCs over a serial link. One page per
stage of bring-up, one engine, one wire format.

```sh
./run_controls.sh
./run_controls.sh --port /dev/ttyACM0
```

This app commands motors. It does not establish that any robot is safe to
power up.

## The hardware path

```text
this app ──USB serial──▶ sender ESP32 ──ESP-NOW──▶ receiver ESP32 ──DShot600──▶ ESCs
```

Both ESP32s are Seeed XIAO ESP32-S3 boards running
[remote-dshot](https://github.com/CR-Capstone/remote-dshot). They are a relay
and nothing more: the sender forwards each valid line over ESP-NOW, and the
receiver turns the two values into DShot frames and repeats them in hardware
until the next packet. **Everything about the ESC protocol — arming, neutral,
commands, ramps — lives in this app.** `remote-dshot/ramp_test.py` is the
known-good reference that this app follows.

## The wire contract

One text line per update, two bare DShot values in decimal:

```text
"<motor 1 value> <motor 2 value>\n"        e.g.  b"1548 548\n"
```

| value | meaning |
| --- | --- |
| `0` | stop. Disarmed, and the first phase of arming |
| `1..47` | ESC command (beep, spin direction, 3-D mode, save) |
| `48..1047` | 3-D reverse, `48` slowest |
| `1048` | 3-D neutral |
| `1049..2047` | 3-D forward, `1049` slowest |

The receiver builds the 16-bit frame itself (telemetry bit 0, normal
checksum). The sender rejects anything that is not exactly two unsigned
decimals in `0..2047` and logs `Bad line` on the same port, which shows up in
the link page console. `controls/protocol.py` is the only place that builds or
reads a line.

### The receiver's rules

`remote-dshot/main/receiver.c`, mirrored in `tools/firmware_reference.py`:

| state | behaviour |
| --- | --- |
| IDLE | No output. The first valid packet starts output. |
| ACTIVE | Each packet replaces the value being repeated. A gap over **500 ms** stops output. |
| LOCKOUT | No output and **every packet ignored for 10 s**, then back to IDLE. |

So the app streams continuously, armed or not:

* Send a line the moment the values change; if nothing changes, resend the
  held line every **keepalive** (50 ms, as `ramp_test.py` does).
* While disarmed, stream `0 0` so the receiver stays ACTIVE and the ESCs see
  a live stop signal.
* If the app itself goes quiet past 500 ms while armed (port lost, app
  stalled), the receiver has cut the ESCs and locked out. The engine disarms
  rather than resuming throttle into an ESC that lost its arming.

### Arming is a sequence

Pressing ARM starts `disarmed → arming → armed`:

1. **Lockout wait.** If the link has had a gap over 500 ms in the last 10 s
   (including at start-up, when the app can't know), keep sending `0 0` until
   any lockout has certainly ended. The status bar counts it down.
2. **`0 0` for 1.6 s** so the ESCs arm.
3. **Neutral (`1048 1048`) for 0.4 s.**
4. The page takes over, with its clock starting at zero, so a gait's spin-up
   ramp starts when the motors can actually move.

The banner shows the phase. All four numbers are on the link page. Disarm
(STOP, space, escape, page change, focus loss) sends `0 0` eight times and
keeps streaming it.

### Signed demand to DShot value

Pages produce signed demand in `[-1, 1]`. A `MotorMap` per motor turns that
into a value:

| setting | effect |
| --- | --- |
| `three_d` | the 3-D ranges above, zero demand is neutral. The default, because the gait counter-rotates |
| plain | 48–2047 magnitude only, zero demand is `0`; a negative demand leaves as forward throttle |
| `invert` | flips the sign before mapping |
| `deadband` | demand at or below this sends idle (neutral in 3-D) |
| `limit` | hard ceiling on the motor |

With `three_d`, demand ±0.5 is exactly `1548` / `548`, the values
`ramp_test.py` uses for 50%.

## Pages

| Stage | Page | What it does |
| --- | --- | --- |
| 0 | **Link** | Port, timing, receiver lockout and arming numbers, per-motor mapping, raw console, manual line entry. Sends no motor output. |
| 1 | **Bench** | Direct throttle per motor, canned bring-up tests including `ramp_test.py`, DShot configuration commands. |
| 2 | **Gait** | Continuous waveform: counter-rotating base plus a common-mode periodic term, with the simulator's gaits as presets. Shows the literal packets. |
| 3 | **Drive** | The droopy gait from the keyboard: hold `W` to walk, `A` / `D` to steer by bias. |

## Stage 1 — bring-up tests

The bench page ships the checks a new ESC pairing needs, in the order they are
usually needed. Pick one, read what to watch for, arm, and press Run; the
sequence drives both motors on a clock and returns to zero when it ends.
Pressing Run again, switching pages or hitting space stops it.

| Test | Watch for |
| --- | --- |
| Arm check | The arming sequence runs; both ESCs play their armed tone and nothing turns. |
| Ramp test (ramp_test.py) | The known-good script: both motors forward to 50% and back, neutral, reverse to 50% and back. 102 s. Run this first on new hardware. |
| Direction — motor 1 | Which way motor 1 turns at a low forward command. |
| Direction — motor 2 | Same for motor 2. |
| Counter-rotate check | The pair turning opposite ways. If they match, the 3-D mapping or the ESC direction is wrong. |
| Deadband sweep | Where each motor first moves, on a 20 s ramp to 20%. |
| Step response | How fast each reaches speed, and whether the ESC desyncs on a hard step. |
| Slow ramp to 50% | Current draw and vibration through the range, with time to abort. |
| Reverse check | Reverse on both motors. Needs the 3-D range. |
| Custom ramp | Whatever the target and time fields below the list say. |

The tests live in `controls/sequences.py` as plain data — a name, what to watch
for, and a list of timed steps that each hold or ramp to a pair of demands.
Adding one is adding a `Sequence` to `TESTS`.

The sliders stay live whenever no test is running, so the page still works for
poking at a single value.

## Stage 2 — the droopy gait

Stage 2 mirrors `droopy.control.Command` in the simulator, so a setting tried
in `droopy/` means the same thing here (with demand as a fraction of full
throttle instead of rad/s):

```python
wave1 = amplitude * shape(2*pi*frequency*t + phase)
wave2 = amplitude * shape(2*pi*frequency*t + phase + offset)
motor1 =  spin + wave1 + bias
motor2 = -spin + wave2 + bias      # counter-rotate on
```

One rotor's magnitude rises while the other's falls. Everything is scaled by a
spin-up ramp from zero over `ramp_s` seconds after arming.

* **Spin base** — steady counter-rotating magnitude.
* **Amplitude** — size of the common-mode term.
* **Frequency** — cycles per second. For a melty-style robot this is the spin rate.
* **Phase** — where in each rotation the term peaks. This is the translation heading.
* **Motor 2 offset** — phase shift on motor 2 only. Leave at `0` to match the simulator.
* **Bias** — constant added to both motors. This is what steers a droopy
  robot; see stage 3.

**Presets.** The simulator's gait normalised by its max rotor speed: a 0.49
base with a ±0.40 swing at 0.83 Hz (9 lb) or 1.0 Hz (3 lb), plus the control
study's gentler candidate and a constant counter-spin. They are a starting
shape, not a calibrated speed: real full throttle will not be the sim's
375 rad/s. Neither rotor ever reverses in these gaits. The status line warns
when a setting makes a rotor change direction every cycle, because that
drags a loaded rotor through 3-D neutral twice per cycle.

The upper left plot is the steady-state waveform with the ramp removed. The
upper right one is what actually went down the wire. When they disagree, a
clamp or the motor mapping is the reason; the peak readout turns red when the
demand saturates.

### Seeing the packets

The lower panel shows the lines themselves. The top block breaks down the
most recent one: the value that went on the wire, and the frame the receiver
builds from it, the way the ESC reads it:

```text
M1   demand +0.381   dshot 1429  throttle
     payload 0xB2A   value 0b10110010101   telem 0   crc 0x3 ok
     ESC frame 0xB2A3   (built by the receiver)
M2   demand -0.319   dshot  367  throttle
     payload 0x2DE   value 0b00101101111   telem 0   crc 0x1 ok
     ESC frame 0x2DE1   (built by the receiver)

wire  b'1429 367\n'
```

Below it is a log of the raw lines, decimated because a 100 Hz stream is not
readable at full rate:

```text
   t (s)  line on the wire        dshot values       why
   2.990  b'1385 410\n'            M1 1385  M2  410  change
   3.095  b'1510 285\n'            M1 1510  M2  285  change
   3.145  b'1510 285\n'            M1 1510  M2  285  keepalive
```

The `why` column is the reason the line left: `change`, `keepalive`, `arm`,
`command` or `disarm`. Frames the engine held back because nothing moved never
appear here — they are counted in the status bar.

* **Freeze** stops the panel updating. Transmission continues.
* **lines/s** sets the log rate, 1 to 200.
* **Capture burst** logs every frame for one gait cycle with no decimation —
  the burst length tracks the frequency and the transmit rate.
* **Copy** puts the whole log on the clipboard.

## Stage 3 — walking

The drive page runs the same gait from the keyboard, set up for walking
rather than for a melty:

* **Hold W / ↑** to engage the walk amplitude. Release it and the robot keeps
  counter-spinning in place.
* **Hold A / ← or D / →** to add ∓/± the steer bias to both rotors. The
  droopy control study (`droopy/docs/control-study.md`) found a bias of about
  0.04 of full speed moved the 9 lb model's travel heading by around 20° over
  established cycles without costing speed, and that startup cycles don't
  steer. The sign comes from the sim; **Invert steering** fixes it on the floor.
* Walk amplitude and bias **slew** at a set rate instead of stepping, because
  a step on a heavy rotor is a current spike and a likely desync.

There is no heading dial. The gait is open loop, so its phase is only a time
offset and cannot point the robot anywhere. Which way it walks comes from the
feet and has to be found on the floor.

## Safety

* Arming is the single gate between the UI and the motors. It needs an open
  port and a page that produces output.
* **Space** or **Escape** disarms from anywhere, as does the STOP button.
* Switching pages, losing the port, losing window focus, or a page raising an
  exception all disarm.
* While disarmed the engine streams value 0 on the keepalive, so the
  receiver stays ACTIVE and the ESCs see a live stop signal.
* Losing the link while armed disarms, and re-arming runs the whole sequence,
  lockout wait included.
* Arming asks for confirmation unless you turn that off in the toolbar.

Failsafe on loss of link belongs in the receiver firmware, which is why its
timeouts are the firmware's numbers and not this app's.
`tools/firmware_reference.py` is a model of the receiver's state machine for
checking the app's timing without the radio link; the test suite drives the
real engine through it for 30 s and requires zero failsafe trips.

```text
FORWARD M1 1548 throttle 1548  M2  548 throttle 548   +0.050s
FAILSAFE  gap of 0.550 s — output stopped, locked out for 10 s
IGNORE  M1 1048 neutral        M2 1048 neutral        (lockout, 9.4 s left)
```

## Layout

```text
controls/
├── controls/
│   ├── dshot.py        # DShot values, frame display, MotorMap
│   ├── protocol.py     # the line format
│   ├── link.py         # serial port, reader thread
│   ├── engine.py       # change-driven transmit, arming sequence, command queue
│   ├── waveform.py     # gait maths and sim presets, no Qt
│   ├── sequences.py    # canned bench tests, no Qt
│   ├── scope.py        # rolling display of transmitted frames
│   ├── packets.py      # literal frame breakdown and raw line log
│   ├── widgets.py      # value slider
│   ├── theme.py        # stylesheet
│   ├── app.py          # window, toolbar, page stack
│   └── pages/          # link, bench, gait, drive
├── tools/
│   └── firmware_reference.py   # model of the remote-dshot receiver
└── tests/
```

## Setup

Python 3.10+.

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

The launcher checks `controls/.venv` first, then the repository's shared
`.venv`.

## Validate

```sh
QT_QPA_PLATFORM=offscreen ../.venv/bin/python -m pytest -q tests
QT_QPA_PLATFORM=offscreen ./run_controls.sh --smoke
```
