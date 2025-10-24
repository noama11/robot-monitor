import { formatSpeed, clamp } from "../../../utils/formatters";
import "./SpeedWidget.css";
export function SpeedWidget({ odom }) {
  const linearSpeed = odom?.speed || 0; // m/s
  const angularSpeed = odom?.angular_z || 0; // rad/s

  const linearPercentage = clamp((linearSpeed / 5) * 100, 0, 100);
  const angularPercentage = clamp((Math.abs(angularSpeed) / 2) * 100, 0, 100);

  return (
    // <div className="speed-widget">
    //   <div className="speed-widget-header widget-header">
    //     <h3 className="speed-widget-title"> Velocity</h3>
    //   </div>

    //   <div className="speed-widget-content">
    //     {/* Linear Speed */}
    //     <div className="speed-widget-primary">
    //       <div className="speed-widget-primary-label">Linear</div>
    //       <div className="speed-widget-primary-value">
    //         {linearSpeed.toFixed(2)}
    //       </div>
    //       <div className="speed-widget-primary-unit">m/s</div>

    //     </div>

    //     {/* Angular Speed */}
    //     <div className="speed-widget-secondary">
    //       <div className="speed-widget-secondary-label">Angular</div>
    //       <div className="speed-widget-secondary-value">
    //         {angularSpeed.toFixed(3)}{" "}
    //         <span className="speed-widget-secondary-unit">rad/s</span>
    //       </div>
    //     </div>
    //   </div>
    // </div>
    <div className="speed-widget">
      <div className="speed-widget-header widget-header">
        <h3 className="speed-widget-title">Velocity</h3>
      </div>

      <div className="speed-widget-content">
        <div className="velocity-card">
          <div className="velocity-card-icon">🚀</div>
          <div className="velocity-card-data">
            <div className="velocity-card-label">Linear</div>
            <div className="velocity-card-value">{linearSpeed.toFixed(2)}</div>
            <div className="velocity-card-unit">m/s</div>
          </div>
        </div>
      </div>
    </div>
  );
}
