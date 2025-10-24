import { formatSpeed, clamp } from "../../../utils/formatters";
import "./SpeedWidget.css";
import { MetricCard } from "../MetricCard/MetricCard";
import "../MetricCard/MetricCard.css";
export function SpeedWidget({ odom }) {
  const linearSpeed = odom?.speed || 0; // m/s

  return (
    <div className="metric-card">
      <MetricCard label="Speed" value={linearSpeed.toFixed(2)} unit="m/s" />
    </div>
  );
}
