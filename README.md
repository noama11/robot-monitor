# Real Time Robot Monitor

A web based monitoring dashboard for ROS 2 robots. The system provides remote operators with real time visibility into robot position, velocity, and camera feeds through a customizable interface.


<img width="1573" height="845" alt="image" src="https://github.com/user-attachments/assets/324ee94a-fa7d-479b-a639-c18ac2cd50d2" />



---

## Design Decisions

### What to Display

The interface shows three core elements:

**Map View** - GPS position with historical path trail for spatial context and movement patterns.

**Velocity Metrics** - Real time speed display to confirm normal operation at a glance.

**Camera Feed** - Throttled to 2 fps for visual context without overwhelming network bandwidth.

I deliberately excluded raw coordinate readouts, detailed telemetry panels, and excessive status indicators. These elements add visual noise without supporting real time operational decisions. The focus is clarity under live monitoring conditions.

### Customizable Layout

Every widget can be resized and repositioned. Layouts persist across sessions with a one click reset to defaults. Different missions require different information hierarchies, so operators control their workspace rather than adapting to a fixed layout.

---

## Technical Architecture

### Backend: Modular Node System

Each sensor type (GPS, Odometry, Camera) inherits from a `BaseRobotNode` base class. This makes the system extensible—adding new sensors means implementing one new class without restructuring existing code.

Each node runs in its own thread for isolation. All data flows through a central `DataManager` that coordinates updates and prevents race conditions through thread-safe operations.

### Real-Time Communication

The system uses WebSockets rather than HTTP polling to minimize latency. Operators need immediate awareness of state changes during live missions.

Data throttling is configurable throughout the pipeline:

- Camera: publishes at 13 Hz, dashboard receives 2 fps
- GPS: updates at 4 Hz, map redraws at 10 Hz
- Odometry: publishes at 28 Hz, processed at 10 Hz

These rates balance responsiveness with network efficiency and UI performance.

### Configuration System

All settings are externalized to configuration files:

**Backend** (`config/config.yaml`) - ROS topic mappings, buffer sizes, throttling rates, WebSocket parameters

**Frontend** (`.env`) - WebSocket connection URL for different deployment environments

**Frontend** (`constants.js`) - Widget layouts, map settings, data validation thresholds, UI update intervals

This enables deployment to different robot configurations without code changes.

### Frontend Architecture

React with context-based state management separates WebSocket logic from UI components. This isolation simplifies testing and keeps components focused on presentation logic.

Unit tests cover backend threading behavior and data management where concurrency creates defect risk.

---

## Getting Started

### Prerequisites

- ROS 2 Humble
- Python 3.8+
- Node.js 18+

### Running the System

### Terminal 1 – Backend
```bash
# Load ROS environment
source /opt/ros/humble/setup.bash

# Create a venv that can see ROS packages (only once)
cd backend
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt

# Run backend server
python3 main.py
```

**Terminal 2 - Frontend:**

```bash
cd robot-monitor-frontend
npm install
npm run dev
```

Open browser to `http://localhost:5173`

**Terminal 3 - Data Playback:**

```bash
source /opt/ros/humble/setup.bash
ros2 bag play path/to/robot_data.mcap 
```

---

## Configuration

### Backend Settings

Edit `backend/config/config.yaml`:

```yaml
ros:
  topics:
    gps: "/ublox_gps_node/fix"
    odom: "/robot_odom"
    image: "/camera/image_raw"

data:
  path_buffer_size: 100
  image_throttle: 2

websocket:
  host: "0.0.0.0"
  port: 8765
  update_rate_hz: 10
```

### Frontend Settings

**Connection** - Edit `robot-monitor-frontend/.env`:

```
VITE_WS_URL="ws://localhost:8765"
```

**Behavior** - Edit `robot-monitor-frontend/src/config/constants.js`:

```javascript
export const WEBSOCKET_CONFIG = {
  url: import.meta.env.VITE_WS_URL || "ws://localhost:8765",
  reconnectDelay: 2000,
  maxReconnectAttempts: 5,
};

export const MAP_CONFIG = {
  defaultCenter: [31.7683, 35.2137],
  defaultZoom: 15,
  pathColor: "#2196F3",
};
```

---

## Testing

Run backend unit tests:

```bash
cd backend
source /opt/ros/humble/setup.bash
source .venv/bin/activate
PYTHONPATH=./src:$PYTHONPATH python -m pytest -v
```

---




