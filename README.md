# Coppelia_RevereseObjectFollower

A Pioneer P3DX robot simulation that can be driven with the keyboard or run in reverse object-following mode in CoppeliaSim.

## How it works

The Python program connects to CoppeliaSim's **legacy Remote API** at `127.0.0.1:1998`. `m_p3dx.py` reads the robot's 16 ultrasonic sensors and sends wheel-speed commands to the left and right motors. The scene and Python code must use the expected object paths: `/PioneerP3DX/ultrasonicSensor[0]` through `[15]`, `/PioneerP3DX/leftMotor`, and `/PioneerP3DX/rightMotor`.

`main.py` has two modes:

- **Keyboard mode** is the default. `W` and `S` drive forward and backward; `A` and `D` turn.
- **Reverse object-following mode** is toggled with `C`. The controller uses the smaller reading from front sensors 3 and 4 as distance `d`, and the difference between sensor readings 5 and 2 as bearing estimate `q`. When an object is closer than the 0.5 m reference, it commands reverse motion; when farther away, it stops rather than approaching. It also steers away from the estimated bearing. `Q` exits and sends a zero-velocity command.

The controller smooths commands before applying them. `m_p3dx.py` converts linear/angular velocity to left/right wheel angular velocity using the Pioneer P3DX wheel radius (0.0975 m) and wheel spacing (0.28 m). A sensor with no detection is reported as 2.0 m, the sensor's maximum range in this project.

## Requirements

- Python 3.10 or newer
- CoppeliaSim with a **legacy Remote API server** listening on port `1998`
- The matching legacy Remote API client library for your operating system and architecture
- The Pioneer P3DX scene in `simscene.ttt`

This project uses the legacy API exposed by `sim.py`; CoppeliaSim's newer ZeroMQ Remote API is not a drop-in replacement. Some CoppeliaSim releases do not include or enable the legacy server by default. Install/enable a compatible legacy Remote API server and configure it to accept connections on port `1998`. The scene or server must be running before starting the Python program.

The included `remoteApi.so` is a 64-bit Linux x86-64 library. On Windows or macOS, obtain the matching legacy Remote API client library from a compatible CoppeliaSim distribution and place it beside `sim.py` as `remoteApi.dll` or `remoteApi.dylib`, respectively. `sim.py` loads the library for the current platform automatically.

## Setup and run

1. Install CoppeliaSim and make sure its legacy Remote API server is available on `127.0.0.1:1998`.
2. Clone this repository:

   ```bash
   git clone https://github.com/khrisnanova/Coppelia_RevereseObjectFollower.git
   cd Coppelia_RevereseObjectFollower
   ```

3. Create and activate a virtual environment, then install the Python dependencies:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install -r requirements.txt
   ```

   On Windows PowerShell, activate with `.venv\Scripts\Activate.ps1` instead.

4. Open `simscene.ttt` in CoppeliaSim and start the simulation. Confirm the legacy Remote API server is listening on port `1998` and the scene object names match those listed above.
5. From the repository directory, run:

   ```bash
   python main.py
   ```

On Linux, the `keyboard` package may require elevated input-device permissions to read global key presses. Run it only in an environment where you trust the code; do not run the entire program as root unless you understand the security implications.

## Controls

| Key | Action |
| --- | --- |
| `W` | Move forward (keyboard mode) |
| `S` | Move backward (keyboard mode) |
| `A` | Turn left (keyboard mode) |
| `D` | Turn right (keyboard mode) |
| `C` | Toggle keyboard / reverse object-following mode |
| `Q` | Stop and quit |

## Project files

| File | Purpose |
| --- | --- |
| `main.py` | Connects to the simulator, reads keyboard input, runs the controller, and sends motion commands |
| `m_p3dx.py` | Reads the 16 proximity sensors and converts robot velocities to wheel velocities |
| `mysim.py` | Separate extended keyboard-control example |
| `sim.py`, `simConst.py` | Legacy Remote API Python wrapper and constants |
| `remoteApi.so` | Included Linux x86-64 legacy Remote API client library |
| `simscene.ttt` | CoppeliaSim scene |
| `requirements.txt` | Python package dependencies |

## Publish your own GitHub copy

After installing and authenticating the GitHub CLI (`gh auth login`), run these commands from the project directory:

```bash
git init -b main
git add .
git commit -m "Initial project"
gh repo create Coppelia_RevereseObjectFollower --public --source=. --remote=origin --push
```

Choose `--private` instead of `--public` if you do not want the repository to be publicly visible. GitHub repository visibility can also be selected in the GitHub web interface.
