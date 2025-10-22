import { DATA_VALIDATION } from "../config/constants";

// Validate GPS data

export function validateGPS(gps) {
  if (!gps || typeof gps !== "object") return null;

  const { latitude, longitude, altitude, timestamp } = gps;

  // Check required fields exist
  if (latitude === undefined || longitude === undefined) return null;

  // Validate latitude range
  if (
    typeof latitude !== "number" ||
    latitude < DATA_VALIDATION.minLatitude ||
    latitude > DATA_VALIDATION.maxLatitude
  ) {
    console.warn("Invalid latitude:", latitude);
    return null;
  }

  // Validate longitude range
  if (
    typeof longitude !== "number" ||
    longitude < DATA_VALIDATION.minLongitude ||
    longitude > DATA_VALIDATION.maxLongitude
  ) {
    console.warn("Invalid longitude:", longitude);
    return null;
  }

  // Validate altitude if present (optional field)
  if (altitude !== undefined && altitude !== null) {
    if (
      typeof altitude !== "number" ||
      altitude < DATA_VALIDATION.minAltitude ||
      altitude > DATA_VALIDATION.maxAltitude
    ) {
      console.warn("Invalid altitude:", altitude);
      // Return GPS but with altitude set to null
      return { ...gps, altitude: null };
    }
  }

  // Validate timestamp if present
  if (
    timestamp !== undefined &&
    (typeof timestamp !== "number" || timestamp < 0)
  ) {
    console.warn("Invalid timestamp:", timestamp);
    // Use current time as fallback
    return { ...gps, timestamp: Date.now() / 1000 };
  }

  return gps;
}

// Validate odometry data

export function validateOdometry(odom) {
  if (!odom || typeof odom !== "object") return null;

  const { speed, position_x, position_y, orientation_z, orientation_w } = odom;

  // Check speed is valid (required field)
  if (speed === undefined || typeof speed !== "number" || speed < 0) {
    console.warn("Invalid speed:", speed);
    return null;
  }

  if (speed > DATA_VALIDATION.maxSpeedMps) {
    console.warn("Speed exceeds maximum:", speed);
    return { ...odom, speed: DATA_VALIDATION.maxSpeedMps }; // max instead of rejecting
  }

  // Validate position if present
  if (position_x !== undefined && typeof position_x !== "number") {
    console.warn("Invalid position_x:", position_x);
    return null;
  }

  if (position_y !== undefined && typeof position_y !== "number") {
    console.warn("Invalid position_y:", position_y);
    return null;
  }

  // Val quaternion if present
  if (orientation_z !== undefined && typeof orientation_z !== "number") {
    console.warn("Invalid orientation_z:", orientation_z);
  }

  if (orientation_w !== undefined && typeof orientation_w !== "number") {
    console.warn("Invalid orientation_w:", orientation_w);
  }

  return odom;
}

// Validate single path point
export function validatePathPoint(point) {
  if (!point || typeof point !== "object") return null;

  const { lat, lng } = point;

  // Validate coordinates
  if (
    typeof lat !== "number" ||
    lat < DATA_VALIDATION.minLatitude ||
    lat > DATA_VALIDATION.maxLatitude
  ) {
    return null;
  }

  if (
    typeof lng !== "number" ||
    lng < DATA_VALIDATION.minLongitude ||
    lng > DATA_VALIDATION.maxLongitude
  ) {
    return null;
  }

  return point;
}

// Validate complete robot data payload from backend

export function validateRobotData(data) {
  if (!data || typeof data !== "object") {
    console.error("Invalid data payload");
    return null;
  }

  const validated = {
    gps: validateGPS(data.gps),
    odom: validateOdometry(data.odom),
    image: data.image || null,
    path: [],
    status: data.status || "no_data",
    timestamp: data.timestamp || new Date().toISOString(),
  };

  // Validate each path point, filter out bad ones
  if (Array.isArray(data.path)) {
    validated.path = data.path
      .map(validatePathPoint)
      .filter((point) => point !== null);
  }

  return validated;
}

// checks if timestamp is older than 5 seconds
export function isDataStale(timestamp) {
  if (!timestamp) return true;

  const now = Date.now();
  let dataTime;

  if (typeof timestamp === "number") {
    // Unix timestamp (seconds or milliseconds)
    dataTime = timestamp < 10000000000 ? timestamp * 1000 : timestamp;
  } else if (typeof timestamp === "string") {
    dataTime = new Date(timestamp).getTime();
  } else {
    return true;
  }

  return now - dataTime > DATA_VALIDATION.staleDataThreshold;
}

// Safe number parsing - ensure we always get a number

export function safeParseFloat(value, defaultValue = 0) {
  const parsed = parseFloat(value);
  return isNaN(parsed) ? defaultValue : parsed;
}

export function safeParseInt(value, defaultValue = 0) {
  const parsed = parseInt(value, 10);
  return isNaN(parsed) ? defaultValue : parsed;
}
