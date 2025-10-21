# Robot Monitor WebSocket API

## Connection

Connect to: `ws://localhost:8765`

The server sends data at 10Hz (configurable in `config.yaml`).

## Data Format

All messages are JSON with the following structure:

### Root Object
```json
{
  "gps": GPS_Object | null,
  "odom": Odometry_Object | null,
  "image": Image_Object | null,
  "path": [PathPoint, ...],
  "status": "active" | "waiting" | "no_data",
  "timestamp": "2025-10-21T19:52:17.482000"
}
```

### GPS_Object
```json
{
  "latitude": 31.663881,      // Degrees (float)
  "longitude": 35.005230,     // Degrees (float)
  "altitude": 45.23,          // Meters (float)
  "timestamp": 1754815489.413 // Unix timestamp (float)
}
```

**Null when:** No GPS data received yet.

---

### Odometry_Object
```json
{
  "speed": 2.56,              // m/s, total velocity (float)
  "position_x": 123.45,       // Meters (float)
  "position_y": -67.89,       // Meters (float)
  "orientation_z": 0.234,     // Quaternion Z component (float)
  "orientation_w": 0.972,     // Quaternion W component (float)
  "timestamp": 1754815489.413 // Unix timestamp (float)
}
```

**Null when:** No odometry data received yet.

**Note:** Speed is calculated as `sqrt(vx² + vy²)` from linear velocities.

---

### Image_Object (Optional)
```json
{
  "format": "jpeg",           // Image format (string)
  "data": "base64string...",  // Base64 encoded image (string)
  "timestamp": 1754815489.413 // Unix timestamp (float)
}
```

**Null when:** Image processing disabled (default) or no image received.

**Note:** Images are heavy. Enable only if needed via `config.yaml`.

---

### PathPoint
```json
{
  "lat": 31.663881,           // Latitude (float)
  "lng": 35.005230,           // Longitude (float)
  "timestamp": 1754815489.413 // Unix timestamp (float)
}
```

**Array:** Contains last N GPS points (default: 100, configurable).

---

### Status Field

- `"active"`: Both GPS and Odometry data available
- `"waiting"`: Only one data source available
- `"no_data"`: No data received yet

---

## Example Full Message
```json
{
  "gps": {
    "latitude": 31.663881,
    "longitude": 35.005230,
    "altitude": 45.23,
    "timestamp": 1754815489.413
  },
  "odom": {
    "speed": 2.56,
    "position_x": 123.45,
    "position_y": -67.89,
    "orientation_z": 0.234,
    "orientation_w": 0.972,
    "timestamp": 1754815489.413
  },
  "image": null,
  "path": [
    {"lat": 31.663881, "lng": 35.005230, "timestamp": 1754815489.413},
    {"lat": 31.663877, "lng": 35.005236, "timestamp": 1754815490.163}
  ],
  "status": "active",
  "timestamp": "2025-10-21T19:52:17.482000"
}
```

## Error Handling

- **Connection lost**: Client should reconnect automatically
- **Malformed JSON**: Check console for parsing errors
- **Missing fields**: All top-level fields always present, nested objects may be `null`