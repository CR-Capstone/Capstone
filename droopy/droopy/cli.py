"""Headless deterministic simulation and CSV export."""
import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

from .control import Command
from .physics import Config, Simulator


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=positive, default=5.0)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--mode", choices=("gait", "constant", "off"), default="gait")
    parser.add_argument("--tilt", type=float, default=15.0)
    parser.add_argument("--friction", type=float, default=0.7)
    args = parser.parse_args()
    config = Config(tilt_deg=args.tilt, friction=args.friction)
    try:
        config.validate()
    except ValueError as exc:
        parser.error(str(exc))
    sim = Simulator(config)
    command = Command(enabled=args.mode != "off", mode=("Constant counterspin" if args.mode == "constant" else "Alternating gait"))
    steps = math.ceil(args.seconds / config.dt)
    if steps > 2_000_000:
        parser.error("run exceeds 2,000,000 physics steps; shorten --seconds")
    handle = args.csv.open("w", newline="") if args.csv else None
    try:
        writer = csv.DictWriter(handle, fieldnames=sim.snapshot().keys()) if handle else None
        if writer:
            writer.writeheader()
            writer.writerow(sim.snapshot())
        stride = max(1, round(0.02 / config.dt))
        for i in range(steps):
            sim.step(command.targets(sim.time, config.max_speed))
            if writer and ((i + 1) % stride == 0 or i == steps - 1):
                writer.writerow(sim.snapshot())
    finally:
        if handle:
            handle.close()
    if args.csv:
        args.csv.with_suffix(".json").write_text(json.dumps({
            "schema": "droopy-exploratory-v1", "config": asdict(config),
            "command": asdict(command), "duration": sim.time,
            "final": sim.snapshot(),
        }, indent=2) + "\n")
    print(json.dumps(sim.snapshot(), indent=2))


if __name__ == "__main__":
    main()
