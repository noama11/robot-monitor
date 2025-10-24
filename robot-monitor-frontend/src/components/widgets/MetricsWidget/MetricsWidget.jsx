import {
  formatSpeed,
  formatCoordinate,
  formatAltitude,
  formatHeading,
  getCompassDirection,
  calculatePathDistance,
  formatDistance,
} from "../../../utils/formatters";

import "./MetricsWidget.css";
export function MetricsWidget({ gps, odom, path }) {
  const distTraveled = calculatePathDistance(path);

  return (
    <div className="metrics-widget">
      <div className="metrics-widget__header widget-header">
        <h3 className="metrics-widget__title">📊 Metrics</h3>
      </div>

      <div className="metrics-widget__content">
        <MetricItem
          label="Speed"
          value={formatSpeed(odom?.speed)}
          highlight={odom && odom.speed > 0}
        />

        <MetricItem
          label="Heading"
          value={formatHeading(odom?.orientation_z, odom?.orientation_w)}
          subtitle={getCompassDirection(
            odom?.orientation_z,
            odom?.orientation_w
          )}
        />

        <MetricItem label="Latitude" value={formatCoordinate(gps?.latitude)} />

        <MetricItem
          label="Longitude"
          value={formatCoordinate(gps?.longitude)}
        />

        <MetricItem label="Altitude" value={formatAltitude(gps?.altitude)} />

        <MetricItem label="Distance" value={formatDistance(distTraveled)} />
      </div>
    </div>
  );
}

function MetricItem({ icon, label, value, subtitle, highlight }) {
  return (
    <div className={`metric-item ${highlight ? "metric-item--highlight" : ""}`}>
      <div className="metric-item__icon">{icon}</div>
      <div className="metric-item__content">
        <div className="metric-item__label">{label}</div>
        <div className="metric-item__value">{value}</div>
        {subtitle && <div className="metric-item__subtitle">{subtitle}</div>}
      </div>
    </div>
  );
}
