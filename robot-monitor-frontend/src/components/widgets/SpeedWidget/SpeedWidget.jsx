import { formatSpeed, clamp } from "../../../utils/formatters";
import "./SpeedWidget.css";
import { MetricCard } from "../MetricCard/MetricCard";
import "../MetricCard/MetricCard.css";
export function SpeedWidget({ odom }) {
  const linearSpeed = odom?.speed || 0; // m/s

  // const linearPercentage = clamp((linearSpeed / 5) * 100, 0, 100);

  return (
    // <div className="speed-widget">
    //   <div className="speed-widget-header widget-header">
    //     <h3 className="speed-widget-title">Velocity</h3>
    //   </div>

    //   <div className="speed-widget-content">
    //     <div className="velocity-card">
    //       <div className="velocity-card-data">
    //         <div className="velocity-card-label">Linear</div>
    //         <div className="velocity-card-value">{linearSpeed.toFixed(2)}</div>
    //         <div className="velocity-card-unit">m/s</div>
    //       </div>
    //     </div>
    //   </div>
    // </div>
    <div className="metric-card">
      <MetricCard label="Speed" value={linearSpeed.toFixed(2)} unit="m/s" />
    </div>
  );
}
