# AirSwarm Drone Navigation

This project implements a drone navigation system for the AirSwarm simulator (https://game.1t.ru/AirSwarm.html). The system controls a fleet of five drones to extinguish fires at specified coordinates by dropping fire extinguishers. Each drone navigates to its target, avoids obstacles using LIDAR data, drops an extinguisher, and returns to the origin (`x=0, z=0`) for recharging before continuing the mission.

## Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Running the Project](#running-the-project)
- [Simulator Details](#simulator-details)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Project Overview
The AirSwarm Drone Navigation project is designed to solve a fire-extinguishing task in a simulated environment. The simulator provides a 3D space with obstacles and fire locations. Drones must:
- Navigate to predefined fire coordinates (`x, z`).
- Avoid obstacles using LIDAR sensors (`f`, `b`, `r`, `l` directions).
- Drop fire extinguishers at precise locations.
- Return to the origin (`x=0, z=0`) for recharging.
- Repeat until all fires are extinguished.

The project uses Python with WebSocket communication to interact with the simulator, PID controllers for precise drone movement, and a navigation algorithm to handle obstacle avoidance and path planning.

## Features
- **Precise Navigation**: Drones align to target coordinates with a tolerance of 0.2 units using a two-stage precision alignment (x-axis, then z-axis).
- **Obstacle Avoidance**: LIDAR-based obstacle detection with a threshold of 0.5 units and safe direction selection (>1.5 units).
- **Return and Recharge**: Drones return to the origin after dropping an extinguisher, reset their state, and resume the mission.
- **PID Control**: Optimized PID parameters (`hor_kp=0.5`, `hor_kd=0.05`) for stable and accurate drone movement.
- **Logging**: Detailed logs for debugging, including drone positions, LIDAR values, distances, and mission status.
- **Asynchronous WebSocket**: Communication with the simulator via WebSocket for real-time data exchange.

## Project Structure
```
AirSwarm/
├── main.py                 # Entry point, initializes WebSocket and runs the mission
├── algorithm/
│   ├── fly.py             # Main navigation logic (aka drone_navigation.py, version v11)
│   ├── PID.py             # PID controller for drone movement
├── connection/
│   ├── SocketConnection.py # WebSocket communication with the simulator
├── config.py              # Configuration file (e.g., LOG_LEVEL)
├── services/
│   ├── logger.py          # Logging utilities
├── README.md              # This file
```

- **`main.py`**: Sets up the WebSocket connection, retrieves fire positions, and starts the mission.
- **`fly.py`**: Implements drone navigation, including path planning, obstacle avoidance, precision alignment, and return logic.
- **`PID.py`**: Manages drone movement with PID controllers for x, y, and z axes.
- **`SocketConnection.py`**: Handles WebSocket communication with the simulator.
- **`config.py` and `logger.py`**: Configuration and logging utilities (may be provided by the project environment).

## Requirements
- Python 3.8+
- Required packages:
  - `websockets` (for WebSocket communication)
- Access to the AirSwarm simulator (https://game.1t.ru/AirSwarm.html or equivalent WebSocket server)

## Installation
1. **Clone the repository** (or set up the project directory):
   ```bash
   mkdir AirSwarm
   cd AirSwarm
   ```
   Copy the provided files (`main.py`, `fly.py`, `PID.py`, `SocketConnection.py`) into the appropriate directories.

2. **Create the directory structure**:
   ```
   mkdir algorithm connection services
   mv fly.py algorithm/
   mv PID.py algorithm/
   mv SocketConnection.py connection/
   mv main.py .
   ```
   Ensure `config.py` and `services/logger.py` are available or comment out the logger import in `main.py` if not needed.

3. **Install dependencies**:
   ```bash
   pip install websockets
   ```

4. **Verify the simulator**:
   - Ensure the simulator is accessible at `ws://game.1t.ru` or update the WebSocket URL in `connection/SocketConnection.py` if different.

## Running the Project
1. **Navigate to the project directory**:
   ```bash
   cd AirSwarm
   ```

2. **Run the main script**:
   ```bash
   python main.py
   ```

3. **Expected output**:
   - Initial connection: `WebSocket connection established`
   - Mission start: `Using code v11`
   - Drone logs:
     ```
     Drone 0: Near target (dist=...), switching to precision
     Drone 0: Aligning x, moving l, dx=-0.17
     Drone 0: Reached target at x=-34.17, z=100.55, dx=..., dz=..., dropping extinguisher
     Drone 0: Reached origin, resetting for recharge
     ```
   - Successful completion: All five fires extinguished (`fires = [1, 1, 1, 1, 1]`).

4. **Monitor the simulator**:
   - Open https://game.1t.ru/AirSwarm.html in a browser to visualize drone movements and fire extinguishing.

## Simulator Details
- **URL**: https://game.1t.ru/AirSwarm.html
- **WebSocket**: `ws://game.1t.ru` (default, may vary)
- **Fire Positions**: The simulator provides a list of fire coordinates. The project targets five specific fires:
  - Fire 14: `x=-34.17, z=100.55` (Drone 0)
  - Fire 13: `x=-24.3, z=99.83` (Drone 1)
  - Fire 1: `x=-30.8, z=48` (Drone 2)
  - Fire 2: `x=-60.32, z=113.95` (Drone 3)
  - Fire 10: `x=-70.85, z=100.86` (Drone 4)
- **Drone Data**:
  - `droneVector`: Current position (`x, y, z`)
  - `lidarInfo`: LIDAR distances (`f`, `b`, `r`, `l`; `-1` means no obstacle)
  - `droneAxisRotation`: Current rotation angles
  - `id`: Drone identifier
- **Control**: Drones move in four directions (`f`, `b`, `r`, `l`) with specified angles and maintain a fixed height (`y=8`).

## Troubleshooting
- **Error: `NameError: name 'SocketConnection' is not defined`**:
  - Ensure `connection/SocketConnection.py` exists and is correctly imported in `fly.py`.
  - Verify the project structure and file paths.
  - Create `SocketConnection.py` as provided if missing.

- **WebSocket Connection Failure**:
  - Check if `ws://game.1t.ru` is accessible. Update the URL in `SocketConnection.py` if needed.
  - Ensure the simulator server is running.

- **Drones Miss Targets**:
  - Check logs for `dx` and `dz` values in `precision_align`:
    ```
    Drone 0: Aligning x, moving l, dx=-0.17
    ```
  - Adjust `PRECISION_ANGLE` (e.g., to `0.3`) or `equal` tolerance (e.g., to `0.25`) in `fly.py`.
  - Verify PID parameters in `PID.py` (`hor_kp=0.5`, `hor_kd=0.05`).

- **Drones Crash**:
  - Inspect LIDAR logs:
    ```
    Drone 2: Direction f blocked (lidar=0.4)
    Drone 2: Crashed, last lidars=..., pos=x=...,z=...
    ```
  - Increase `SAFE_LIDAR_THRESHOLD` (e.g., to `2.0`) or decrease `DANGER_LIDAR_THRESHOLD` (e.g., to `0.6`) in `fly.py`.
  - Reduce `DANGER_COUNT_THRESHOLD` to `1` for faster route reset.

- **Fires Not Extinguished**:
  - Add debug logs in `fly.py` to verify drop coordinates:
    ```python
    if equal(drone["droneVector"]["x"], targets[i]["x"]) and equal(drone["droneVector"]["z"], targets[i]["z"]) and not DROPPED[i]:
        print(f"Drone {i}: Drop coordinates, pos_x={drone['droneVector']['x']:.4f}, pos_z={drone['droneVector']['z']:.4f}, target_x={targets[i]['x']:.4f}, target_z={targets[i]['z']:.4f}")
    ```
  - Ensure `fires` array returns `[1, 1, 1, 1, 1]`.

- **Logs Missing**:
  - If `config.py` or `logger.py` cause issues, comment out:
    ```python
    # set_logger_config(LOG_LEVEL)
    ```
  - Verify logging configuration or provide these files.

## Contributing
Contributions are welcome! To contribute:
1. Fork the repository (if hosted).
2. Create a feature branch (`git checkout -b feature-name`).
3. Commit changes (`git commit -m "Add feature"`).
4. Push to the branch (`git push origin feature-name`).
5. Open a pull request with a description of changes.

Please include tests or logs demonstrating improvements, especially for navigation or obstacle avoidance.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details (create one if needed).

---

**Contact**: For issues or questions, provide logs (`dx`, `dz`, LIDAR values, coordinates) and a description of the problem to facilitate debugging.
