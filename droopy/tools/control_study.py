#!/usr/bin/env python3
"""Bounded, reproducible 9 lb gait/steering study.

This is deliberately separate from the application controller.  It applies only
rotor speed commands to Simulator; it never edits pose, velocity, or forces.
Default run: 23 deterministic trials (four complete gait cycles each, including four
half-dt repeats) and JSON output suitable for docs/control-study.json.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from pathlib import Path
import json
import math
import sys

import numpy as np

# Permit `python tools/control_study.py` from Capstone/droopy.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from droopy.control import Command
from droopy.physics import Simulator
from droopy.presets import robot_preset
from droopy.steering import DirectionController


def wrap_deg(x: float) -> float:
    return (x + 180.0) % 360.0 - 180.0


def angle_deg(x: float, y: float) -> float | None:
    return math.degrees(math.atan2(y, x)) if math.hypot(x, y) > 1e-12 else None


class CycleAverageController:
    """Experimental held correction from mean yaw of the completed cycle.

    This uses the same gain and cap as DirectionController, but deliberately
    ignores within-cycle yaw.  At each boundary it circular-averages sampled yaw
    and slews by at most 80 rad/s per elapsed cycle.  The first cycle is unbiased.
    """
    def __init__(self, requested_deg: float, frequency: float):
        self.requested = requested_deg
        self.frequency = frequency
        self.bias = 0.0
        self.yaws: list[float] = []
        self.cycle = 0

    def update(self, t: float, yaw: float) -> float:
        now_cycle = int(math.floor((t + 1e-10) * self.frequency))
        if now_cycle > self.cycle and self.yaws:
            radians = np.radians(self.yaws)
            mean_yaw = math.degrees(math.atan2(float(np.mean(np.sin(radians))),
                                               float(np.mean(np.cos(radians)))))
            error = wrap_deg(self.requested - wrap_deg(mean_yaw + 180.0))
            target = max(-45.0, min(45.0, -0.40 * error))
            slew = 80.0 / self.frequency
            self.bias += max(-slew, min(slew, target - self.bias))
            self.cycle = now_cycle
            self.yaws.clear()
        self.yaws.append(yaw)
        return self.bias


def trial(name: str, *, amp_scale=1.0, freq_scale=1.0, fixed_bias=0.0,
          feedback: str = "none", requested_deg=-160.0, dt_scale=1.0,
          cycles=4) -> dict:
    config, base = robot_preset("9lb")
    config = replace(config, dt=config.dt * dt_scale)
    command = replace(base, enabled=True, amplitude=base.amplitude * amp_scale,
                      frequency=base.frequency * freq_scale, bias=fixed_bias)
    sim = Simulator(config)
    instant = DirectionController() if feedback == "instant" else None
    averaged = CycleAverageController(requested_deg, command.frequency) if feedback == "cycle_average" else None
    duration = cycles / command.frequency
    steps = int(math.ceil(duration / config.dt))

    # Tick samples are retained only as scalar arrays; this is also the metric
    # audit seam used to derive cycle and clipping/saturation evidence.
    ts=[]; xs=[]; ys=[]; yaws=[]; tilts=[]; biases=[]; torques=[]; clips=[]
    for _ in range(steps):
        snap = sim.snapshot()
        yaw = float(snap["yaw_deg"])
        if instant:
            applied = instant.update(sim, command, requested_deg, 1.0)
        elif averaged:
            applied = replace(command, bias=fixed_bias + averaged.update(sim.time, yaw))
        else:
            applied = command
        raw = applied.raw_targets(sim.time)
        targets = applied.targets(sim.time, config.max_speed)
        sim.step(targets)
        r = sim.rotation
        ts.append(sim.time); xs.append(float(sim.position[0])); ys.append(float(sim.position[1]))
        yaws.append(float(sim.snapshot()["yaw_deg"]))
        tilts.append(math.degrees(math.acos(float(np.clip(r[2, 2], -1, 1)))))
        biases.append(float(applied.bias))
        torques.extend(abs(float(v)) for v in sim.last_torques)
        clips.extend(abs(raw[i]) > config.max_speed + 1e-12 for i in range(2))

    t=np.asarray(ts); x=np.asarray(xs); y=np.asarray(ys)
    yaw_unwrap=np.degrees(np.unwrap(np.radians(yaws)))
    cycle_rows=[]
    for c in range(cycles):
        mask=(t > c/command.frequency) & (t <= (c+1)/command.frequency + config.dt)
        inds=np.flatnonzero(mask)
        if not len(inds): continue
        i0=max(0, inds[0]-1); i1=inds[-1]
        dx=float(x[i1]-x[i0]); dy=float(y[i1]-y[i0])
        cycle_rows.append({
            "cycle":c+1, "dx_m":dx, "dy_m":dy,
            "distance_m":math.hypot(dx,dy), "travel_angle_deg":angle_deg(dx,dy),
            "yaw_change_deg":float(yaw_unwrap[i1]-yaw_unwrap[i0]),
            "yaw_peak_to_peak_deg":float(np.ptp(yaw_unwrap[inds])),
            "mean_bias_rad_s":float(np.mean(np.asarray(biases)[inds])),
            "max_tilt_deg":float(np.max(np.asarray(tilts)[inds])),
        })
    dx=float(x[-1]); dy=float(y[-1]); travel=angle_deg(dx,dy)
    later=cycle_rows[1:] if len(cycle_rows)>1 else cycle_rows
    # Aggregate increments by waveform half-cycle. This does not confuse the
    # large within-cycle body-yaw swing with net displacement direction.
    phase_positive=np.sin(2*math.pi*command.frequency*(t-config.dt)) >= 0
    step_dx=np.diff(np.r_[0.0,x]); step_dy=np.diff(np.r_[0.0,y])
    step_yaw=np.diff(np.r_[yaw_unwrap[0],yaw_unwrap])
    phase_rows={}
    for label,mask in (("sine_nonnegative",phase_positive),("sine_negative",~phase_positive)):
        pdx=float(np.sum(step_dx[mask])); pdy=float(np.sum(step_dy[mask]))
        phase_rows[label]={"dx_m":pdx,"dy_m":pdy,"distance_vector_m":math.hypot(pdx,pdy),
                           "travel_angle_deg":angle_deg(pdx,pdy),
                           "net_body_yaw_deg":float(np.sum(step_yaw[mask]))}
    # Lateral/longitudinal components in the requested direction frame.
    a=math.radians(requested_deg); longitudinal=dx*math.cos(a)+dy*math.sin(a)
    lateral=-dx*math.sin(a)+dy*math.cos(a)
    return {
        "name":name,
        "setup":{"robot":"9lb", "cycles":cycles, "duration_s":float(sim.time),
                 "dt_s":config.dt, "amplitude_scale":amp_scale,
                 "frequency_scale":freq_scale, "fixed_bias_rad_s":fixed_bias,
                 "feedback":feedback, "requested_angle_deg":requested_deg,
                 "config":asdict(config), "command":asdict(command)},
        "metrics":{"dx_m":dx,"dy_m":dy,"distance_m":math.hypot(dx,dy),
                   "travel_angle_deg":travel,
                   "requested_longitudinal_m":longitudinal,
                   "requested_lateral_m":lateral,
                   "heading_error_deg":None if travel is None else wrap_deg(travel-requested_deg),
                   "mean_speed_mps":math.hypot(dx,dy)/sim.time,
                   "final_yaw_deg":float(yaws[-1]),
                   "net_unwrapped_yaw_deg":float(yaw_unwrap[-1]-yaw_unwrap[0]),
                   "max_tilt_deg":float(max(tilts)),
                   "tipped_over":bool(max(tilts)>45.0),
                   "torque_saturation_fraction":float(np.mean(np.asarray(torques) >= config.max_torque-1e-9)),
                   "command_clipping_fraction":float(np.mean(clips)),
                   "startup_cycle":cycle_rows[0],
                   "waveform_phase_aggregate":phase_rows,
                   "later_cycle_mean_distance_m":float(np.mean([z["distance_m"] for z in later])),
                   "later_cycle_mean_yaw_p2p_deg":float(np.mean([z["yaw_peak_to_peak_deg"] for z in later])),
                   "cycles":cycle_rows},
    }


def run_study() -> dict:
    specs=[]
    for b in (0, -15, 15, -30, 30):
        specs.append((f"default_fixed_{b:+g}", dict(fixed_bias=b)))
    specs += [
        ("default_instant_request_-160",dict(feedback="instant",requested_deg=-160)),
        ("default_instant_request_+160",dict(feedback="instant",requested_deg=160)),
    ]
    candidates=[("gentle",.75,.80),("slow",.75,.65),("low",.60,.80)]
    for label,a,f in candidates:
        specs.append((f"{label}_open",dict(amp_scale=a,freq_scale=f)))
        specs.append((f"{label}_fixed_-15",dict(amp_scale=a,freq_scale=f,fixed_bias=-15)))
        specs.append((f"{label}_fixed_+15",dict(amp_scale=a,freq_scale=f,fixed_bias=15)))
    # The gentle candidate preserves more speed than the more reduced variants;
    # test feedback forms symmetrically without pretending this is optimization.
    specs += [
        ("gentle_instant_request_-160",dict(amp_scale=.75,freq_scale=.80,feedback="instant",requested_deg=-160)),
        ("gentle_cycleavg_request_-160",dict(amp_scale=.75,freq_scale=.80,feedback="cycle_average",requested_deg=-160)),
        ("gentle_cycleavg_request_+160",dict(amp_scale=.75,freq_scale=.80,feedback="cycle_average",requested_deg=160)),
        ("default_open_dt_half",dict(dt_scale=.5)),
        ("default_fixed_-15_dt_half",dict(fixed_bias=-15,dt_scale=.5)),
        ("gentle_open_dt_half",dict(amp_scale=.75,freq_scale=.80,dt_scale=.5)),
        ("gentle_cycleavg_-160_dt_half",dict(amp_scale=.75,freq_scale=.80,feedback="cycle_average",requested_deg=-160,dt_scale=.5)),
    ]
    results=[trial(name,**kw) for name,kw in specs]
    return {
        "schema":"droopy-control-study-v1",
        "purpose":"bounded comparative evidence; not controller calibration",
        "model_warning":"illustrative simulator, not a validated digital twin",
        "metric_notes":"mean_speed_mps is net displacement/duration, not path length/time. Cycle windows include a boundary tolerance of one integration tick.",
        "reproduce":"/home/Sam/src/Capstone/.venv/bin/python tools/control_study.py --output docs/control-study.json",
        "trial_count":len(results),
        "invariants":{"no_direct_yaw_force":True,"no_scripted_propulsion":True,
                      "feedback_reads_sim_truth":True,"production_files_changed":False},
        "trials":results,
    }


def main() -> None:
    p=argparse.ArgumentParser()
    p.add_argument("--output",type=Path)
    args=p.parse_args()
    result=run_study()
    text=json.dumps(result,indent=2,allow_nan=False)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text)
    else:
        print(text,end="")

if __name__ == "__main__":
    main()
