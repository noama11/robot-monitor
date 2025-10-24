export function formatSpeed(speed) {
  if (speed == null || typeof speed !== "number" || isNaN(speed)) {
    return "N/A";
  }
  return `${speed.toFixed(2)} m/s`;
}

// Calculate distance between two GPS coordinates using Haversine formula
// Returns distance in meters

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

// Calculate total distance traveled from path array

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

//  Format distance with appropriate units (m or km)

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

//  Convert degrees to radians

function toRadians(degrees) {
  return degrees * (Math.PI / 180);
}

export function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}
