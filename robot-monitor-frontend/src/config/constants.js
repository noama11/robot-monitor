export const AVAILABLE_WIDGETS = [
  {
    id: "map",
    name: "Map",
    defaultLayout: { x: 0, y: 0, w: 7, h: 24 },
  },
  {
    id: "camera",
    name: "Camera",
    defaultLayout: { x: 7, y: 0, w: 5, h: 14 },
  },

  {
    id: "speed",
    name: "Velocity",
    defaultLayout: { x: 7, y: 8, w: 5, h: 10 },
  },
];

// Backend connection settings
export const WEBSOCKET_CONFIG = {
  url: import.meta.env.VITE_WS_URL || "ws://localhost:8765",
  reconnectDelay: 2000,
  maxReconnectAttempts: 5,
  heartbeatInterval: 5000,
};

// Map settings
export const MAP_CONFIG = {
  defaultCenter: [31.7683, 35.2137], // Jerusalem area as fallback
  defaultZoom: 15,
  maxZoom: 19,
  minZoom: 10,
  pathColor: "#2196F3",
  pathWeight: 5,
  robotMarkerColor: "#22c55e",
};

// Data thresholds and validation
export const DATA_VALIDATION = {
  maxSpeedMps: 20, // Max reasonable speed in m/s
  minLatitude: -90,
  maxLatitude: 90,
  minLongitude: -180,
  maxLongitude: 180,
  maxAltitude: 9000, // meters
  minAltitude: -500,
  staleDataThreshold: 5000, // ms - consider data stale after 5s
};

// UI update rates
export const UI_CONFIG = {
  metricsUpdateInterval: 100, // ms - how often to update displayed metrics
  mapUpdateInterval: 100,
  statusCheckInterval: 1000,
};

// Status messages
export const STATUS_MESSAGES = {
  connected: "Connected",
  connecting: "Connecting...",
  disconnected: "Disconnected",
  error: "Connection Error",
  active: "Robot Active",
  waiting: "Waiting for Sensors",
  no_data: "No Data",
};

// Color scheme
export const COLORS = {
  success: "#22c55e",
  warning: "#f59e0b",
  error: "#ef4444",
  info: "#3b82f6",
  neutral: "#64748b",
};
