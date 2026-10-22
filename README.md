# Capstone — combat-robot physics simulators and controller

Three independent desktop tools, in one repository:

| Project | Mechanism | Start |
| --- | --- | --- |
| [Melty](melty/README.md) | Spinning wheeled Meltybrain robot, native C++ physics and reference firmware | `./melty/run_sim.sh` |
| [Droopy](droopy/README.md) | Twin angled rotors, reaction torque, gyroscopic coupling and contact-driven shuffling | `./droopy/run_sim.sh` |
| [Controls](controls/README.md) | Serial controller for the real two-motor robot, through the remote-dshot ESP32 link | `./controls/run_controls.sh` |

The two simulators offer interactive physics visualization and telemetry.
Droopy is an **exploratory model**, not a calibrated replica of the real
competition robot. Read each project's model limits before interpreting
results.

## Layout

```text
Capstone/
├── melty/                 # Original simulator, C++ firmware, docs and tests
│   ├── sim/
│   ├── firmware/
│   └── run_sim.sh
├── droopy/                # Python 3-D dynamics, UI, explainers and tests
│   ├── droopy/
│   └── run_sim.sh
├── controls/              # Qt serial controller for the physical robot
│   ├── controls/
│   └── run_controls.sh
└── .venv/                 # Optional shared environment (not in Git)
```

The original top-level Melty files now live in `melty/`. The earlier `Drupi/`
prototype is renamed `droopy/`, including its Python package. The Git root
remains `Capstone/`.

## Setup

Python 3.10+ is required. Melty additionally needs a C++17 compiler and CMake 3.20+.
From this repository root:

```sh
python3 -m venv .venv
.venv/bin/pip install -r melty/requirements.txt -r droopy/requirements.txt
cmake -S melty -B melty/build -DPython_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build melty/build -j
```

Each launcher first checks its project's `.venv`, then the shared root `.venv`.
The launchers work from any directory. Do not move an existing virtual environment
or reuse a CMake cache from before the directory reorganization; create/reconfigure
instead.

## Run

```sh
./melty/run_sim.sh
./droopy/run_sim.sh
./controls/run_controls.sh
```

Only `controls/` talks to hardware. The simulators do not.

See the project READMEs for live controls, detailed physics assumptions,
recordings, CLI experiments, and test commands.

## Validate

From the repository root with the shared environment:

```sh
ctest --test-dir melty/build --output-on-failure
(cd melty && QT_QPA_PLATFORM=offscreen PYTHONPATH=build/python ../.venv/bin/python -m pytest -q tests)
(cd droopy && QT_QPA_PLATFORM=offscreen ../.venv/bin/python -m pytest -q tests)
(cd controls && QT_QPA_PLATFORM=offscreen ../.venv/bin/python -m pytest -q tests)
QT_QPA_PLATFORM=offscreen ./melty/run_sim.sh --smoke
QT_QPA_PLATFORM=offscreen ./droopy/run_sim.sh --smoke
QT_QPA_PLATFORM=offscreen ./controls/run_controls.sh --smoke
```

Generated build files, virtual environments, and Python caches remain excluded
from Git. Neither simulator establishes the safety of a physical combat robot.
