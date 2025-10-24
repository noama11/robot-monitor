import "./VelocityWidget.css";
export function VelocityWidget({
  type, // linear or angular
  value,
  icon,
  unit, // unit of measurement
  maxValue, // max value for bar visualization
  color, // color for the bar
}) {
  const percentage = Math.min((Math.abs(value) / maxValue) * 100, 100);

  return (
    <div className="velocity-widget">
      <div className="widget-header velocity-widget-header">
        <h3 className="velocity-widget-title">
          {icon} {type}
        </h3>
      </div>

      <div className="velocity-widget-content">
        <div className="value-display">
          <div className="value-display-value">{value.toFixed(2)}</div>
          <div className="value-display-unit">{unit}</div>
        </div>

        <div className="bar-container">
          <div
            className="bar-progress"
            style={{
              width: `${percentage}%`,
              background:
                value !== 0
                  ? `linear-gradient(90deg, ${color} 0%, ${color}dd 100%)`
                  : "#334155",
            }}
          />
        </div>
      </div>
    </div>
  );
}
