export function formatSpeed(speed) {
  if (speed == null || typeof speed !== "number" || isNaN(speed)) {
    return "N/A";
  }
  return `${speed.toFixed(2)} m/s`;
}

// Format coordinate (lat/lng)
export function formatCoordinate(value, decimals = 6) {
  if (value == null || typeof value !== "number" || isNaN(value)) {
    return "N/A";
  }
  return value.toFixed(decimals);
}

export function formatAltitude(altitude) {
  if (altitude == null || typeof altitude !== "number" || isNaN(altitude)) {
    return "N/A";
  }

  return `${altitude.toFixed(1)} m`;
}

// Convert quaternion (z, w) to heading angle in degrees
export function quaternionToHeading(qz, qw) {
  const isQzInvalid = qz == null || typeof qz !== "number" || isNaN(qz);
  const isQwInvalid = qw == null || typeof qw !== "number" || isNaN(qw);
  if (isQzInvalid || isQwInvalid) {
    return null;
  }

  // Calculate yaw from quaternion
  const yaw = Math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz);

  // Convert radians to degrees
  let degrees = yaw * (180.0 / Math.PI);

  // Normalize to 0-360 range
  degrees = ((degrees % 360) + 360) % 360;

  return degrees;
}

export function formatHeading(qz, qw) {
  const heading = quaternionToHeading(qz, qw);

  if (heading === null) return "N/A";

  return `${heading.toFixed(1)}°`;
}

export function getCompassDirection(qz, qw) {
  const heading = quaternionToHeading(qz, qw);

  if (heading === null) return "N/A";

  const directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
  const index = Math.round(heading / 45) % 8;

  return directions[index];
}

export function formatTime(timestamp) {
  if (!timestamp) return "N/A";

  try {
    let date;

    if (typeof timestamp === "number") {
      // Handle both seconds and milliseconds Unix timestamps
      const ms = timestamp < 10000000000 ? timestamp * 1000 : timestamp;
      date = new Date(ms);
    } else if (typeof timestamp === "string") {
      date = new Date(timestamp);
    } else {
      return "N/A";
    }

    if (isNaN(date.getTime())) return "N/A";

    return date.toLocaleTimeString();
  } catch (error) {
    console.error("Error formatting time:", error);
    return "N/A";
  }
}

export function formatRelativeTime(timestamp) {
  if (!timestamp) return "N/A";

  try {
    let dataTime;

    if (typeof timestamp === "number") {
      dataTime = timestamp < 10000000000 ? timestamp * 1000 : timestamp;
    } else if (typeof timestamp === "string") {
      dataTime = new Date(timestamp).getTime();
    } else {
      return "N/A";
    }

    if (isNaN(dataTime)) return "N/A";

    const secondsAgo = Math.floor((Date.now() - dataTime) / 1000);

    if (secondsAgo < 1) return "Just now";
    if (secondsAgo < 60) return `${secondsAgo}s ago`;
    if (secondsAgo < 3600) return `${Math.floor(secondsAgo / 60)}m ago`;
    return `${Math.floor(secondsAgo / 3600)}h ago`;
  } catch (error) {
    console.error("Error formatting relative time:", error);
    return "N/A";
  }
}

/**
 * Calculate distance between two GPS coordinates using Haversine formula
 * Returns distance in meters
 */
export function calculateDistance(lat1, lon1, lat2, lon2) {
  // Validate all inputs
  if (
    [lat1, lon1, lat2, lon2].some(
      (v) => v === null || v === undefined || typeof v !== "number" || isNaN(v)
    )
  ) {
    return 0;
  }

  const R = 6371000; // Earth's radius in meters
  const dLat = toRadians(lat2 - lat1);
  const dLon = toRadians(lon2 - lon1);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(toRadians(lat1)) *
      Math.cos(toRadians(lat2)) *
      Math.sin(dLon / 2) *
      Math.sin(dLon / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c;
}

/**
 * Calculate total distance traveled from path array
 */
export function calculatePathDistance(path) {
  if (!Array.isArray(path) || path.length < 2) return 0;

  let totalDistance = 0;

  for (let i = 1; i < path.length; i++) {
    const prev = path[i - 1];
    const curr = path[i];

    // Skip invalid points
    if (!prev?.lat || !prev?.lng || !curr?.lat || !curr?.lng) continue;

    const distance = calculateDistance(prev.lat, prev.lng, curr.lat, curr.lng);

    totalDistance += distance;
  }

  return totalDistance;
}

/**
 * Format distance with appropriate units (m or km)
 */
export function formatDistance(meters) {
  if (meters === null || meters === undefined || typeof meters !== "number") {
    return "N/A";
  }

  if (isNaN(meters)) return "N/A";

  if (meters < 1000) {
    return `${meters.toFixed(1)} m`;
  }

  return `${(meters / 1000).toFixed(2)} km`;
}

/**
 * Convert degrees to radians
 */
function toRadians(degrees) {
  return degrees * (Math.PI / 180);
}

/**
 * Clamp value between min and max
 */
export function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}
