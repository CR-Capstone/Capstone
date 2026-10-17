# Controls — serial controller for a two-motor combat robot

A PySide6 desktop app that drives two ESCs over a serial link. One page per
stage of bring-up, one engine, one wire format.

```sh
./run_controls.sh
./run_controls.sh --port /dev/cu.usbmodem1101 --baud 921600
```

This app commands motors. It does not establish that any robot is safe to
power up.

## The wire contract

One text line per control frame:

```text
"<motor 1 frame> <motor 2 frame>\n"        e.g.  b"49541 17544\n"
```

Each number is the complete **16-bit DShot frame**, not the bare throttle:

| bits | field |
| --- | --- |
| 15..5 | 11-bit value |
| 4 | telemetry request |
| 3..0 | checksum |

The value is `0` for disarmed, `1..47` for ESC commands and `48..2047` for
throttle. The checksum is the XOR of the three nibbles of `value << 1 \| telemetry`;
bidirectional DShot inverts it. Hexadecimal (`0xC185 0x4488`) is accepted on
both sides and is switched on from the link page.

`controls/protocol.py` is the only place that builds or reads a line, and
`controls/dshot.py` the only place that frames one. `tools/firmware_reference.py`
is a working receiver showing what the other end has to do.

### Timing: hold, change, keepalive

The link is change-driven, not a stream. The two ends agree on this:

| Side | Rule |
| --- | --- |
| Firmware | Drive the ESCs from the last accepted packet. Keep driving it until a new one arrives or **2 s** pass with no packet, then fail safe to value 0. |
| Laptop | Evaluate the active page at the evaluation rate. Send a packet the moment the values change. If nothing changes, resend the held packet every **0.5 s**. |

Both numbers are set on the link page. The app does not enforce the firmware's
timeout — it only checks the keepalive has margin against it, and warns when
one lost packet would be enough to trip the failsafe. The rule of thumb is at
least three keepalives inside the timeout.

A frame always goes out, whatever else is true, when:

* a motor starts or stops — a transition to or from value 0 ignores the change
  threshold;
* the port opens — the app does not know what the firmware is holding;
* arming or disarming — a disarm sends eight frames back to back;
* a DShot command is queued — ten identical command frames must all arrive.

### What that costs on the wire

Measured against the reference receiver, 200 Hz evaluation, 0.5 s keepalive,
over a five-second run:

| Output | Packets sent | Held back | Longest gap | Bytes/s |
| --- | --- | --- | --- | --- |
| Slider held at 40% | 10 | 990 | 0.500 s | 24 |
| 1 Hz sine, threshold 1 | 960 | 40 | 0.025 s | 2237 |
| 1 Hz sine, threshold 10 | 350 | 650 | 0.095 s | 819 |
| 1 Hz sine, threshold 50 | 60 | 940 | 0.275 s | 141 |

A held command costs almost nothing. **A continuous waveform always changes**,
so stage 2 sends on nearly every evaluation unless the change threshold is
raised — that is the knob that trades waveform resolution for line traffic.
The gait page shows the expected send rate for the current settings, and the
status bar shows what is really happening: evaluation rate, send rate, frames
sent, frames held.

### Signed demand to DShot value

Pages produce signed demand in `[-1, 1]`. A `MotorMap` per motor turns that
into a value:

| setting | effect |
| --- | --- |
| `three_d` | 48–1047 reverse, 1048–2047 forward — the default, because the gait counter-rotates |
| plain | 48–2047 magnitude only; a negative demand leaves as forward throttle |
| `invert` | flips the sign before mapping |
| `deadband` | demand at or below this sends value 0 |
| `limit` | hard ceiling on the motor |

Without `three_d` the sign is lost. That is right when the ESC itself is
configured reversed, and wrong otherwise, so the gait and drive pages say so
in their status line when it applies.

## Pages

| Stage | Page | What it does |
| --- | --- | --- |
| 0 | **Link** | Port, baud, timing, wire format, per-motor mapping, raw console, manual line entry. Sends no motor output. |
| 1 | **Bench** | Direct throttle per motor, a list of canned bring-up tests, DShot configuration commands. |
| 2 | **Gait** | Continuous waveform: counter-rotating base plus a common-mode periodic term. Sine, triangle, square or sawtooth. Shows the literal packets. |
| 3 | **Drive** | The same gait steered from a heading dial or `W A S D` / arrows. |

## Stage 1 — bring-up tests

The bench page ships the checks a new ESC pairing needs, in the order they are
usually needed. Pick one, read what to watch for, arm, and press Run; the
sequence drives both motors on a clock and returns to zero when it ends.
Pressing Run again, switching pages or hitting space stops it.

| Test | Watch for |
| --- | --- |
| Arm check | Both ESCs beep their arming tone and nothing turns. |
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
in `droopy/` means the same thing here:

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
* **Bias** — constant added to both motors.

The upper left plot is the steady-state waveform with the ramp removed. The
upper right one is what actually went down the wire. When they disagree, a
clamp or the motor mapping is the reason; the peak readout turns red when the
demand saturates.

### Seeing the packets

The lower panel shows the frames themselves. The top block breaks down the
most recent one the way the ESC reads it:

```text
M1   demand +0.381   dshot 1429  throttle
     payload 0xB2A   value 0b10110010101   telem 0   crc 0x3 ok
     frame   0xB2A3   45731
M2   demand -0.319   dshot  367  throttle
     payload 0x2DE   value 0b00101101111   telem 0   crc 0x1 ok
     frame   0x2DE1   11745

wire  b'45731 11745\n'
```

Below it is a log of the raw lines, decimated because a 200 Hz stream is not
readable at full rate:

```text
   t (s)  line on the wire        dshot values       why
   2.990  b'44325 13124\n'         M1 1385  M2  410  change
   3.095  b'48331 9131\n'          M1 1510  M2  285  change
   3.595  b'48331 9131\n'          M1 1510  M2  285  keepalive
```

The `why` column is the reason the frame left: `change`, `keepalive`,
`command` or `disarm`. Frames the engine held back because nothing moved never
appear here — they are counted in the status bar.

* **Freeze** stops the panel updating. Transmission continues.
* **lines/s** sets the log rate, 1 to 200.
* **Capture burst** logs every frame for one gait cycle with no decimation —
  the burst length tracks the frequency and the transmit rate.
* **Copy** puts the whole log on the clipboard.

Heading is a phase, not a surveyed direction. Zero degrees is wherever the
wiring and contact behaviour put it — find the offset on the floor and keep it.

## Safety

* Arming is the single gate between the UI and the motors. It needs an open
  port and a page that produces output.
* **Space** or **Escape** disarms from anywhere, as does the STOP button.
* Switching pages, losing the port, losing window focus, or a page raising an
  exception all disarm.
* While disarmed the engine holds value 0 and resends it every keepalive, so
  the firmware can tell a live link from a dead one.
* Arming asks for confirmation unless you turn that off in the toolbar.

Failsafe on loss of link belongs in the firmware, which is why the hold
timeout is the firmware's number and not this app's. `tools/firmware_reference.py`
implements the receiving half and can be pointed at the port to check it:

```sh
python tools/firmware_reference.py /dev/cu.usbmodem1101 --hold 2.0
```

```text
HOLD    M1 0xC185 throttle 1548  telem=0 crc=ok  M2 0x4488 throttle 548   telem=0 crc=ok
HOLD    M1 0xC185 throttle 1548  telem=0 crc=ok  M2 0x4488 throttle 548   telem=0 crc=ok  +0.500s
FAILSAFE  no packet for 2.00 s — outputs forced to value 0
```

## Layout

```text
controls/
├── controls/
│   ├── dshot.py        # frame construction, MotorMap
│   ├── protocol.py     # the line format
│   ├── link.py         # serial port, reader thread
│   ├── engine.py       # fixed-rate transmit, arming, command queue
│   ├── waveform.py     # gait maths, no Qt
│   ├── sequences.py    # canned bench tests, no Qt
│   ├── scope.py        # rolling display of transmitted frames
│   ├── packets.py      # literal frame breakdown and raw line log
│   ├── widgets.py      # slider and heading dial
│   ├── theme.py        # stylesheet
│   ├── app.py          # window, toolbar, page stack
│   └── pages/          # link, bench, gait, drive
├── tools/
│   └── firmware_reference.py
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
