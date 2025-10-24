// src/components/widgets/MetricCard/MetricCard.jsx
import "./MetricCard.css";

export function MetricCard({ icon, label, value, unit }) {
  return (
    // <div className="metric-card">
    //   <div className="metric-card__icon">{icon}</div>
    //   <div className="metric-card__data">
    //     <div className="metric-card__label">{label}</div>
    //     <div className="metric-card__value">{value}</div>
    //     <div className="metric-card__unit">{unit}</div>
    //   </div>
    // </div>

    <div className="metric-card">
      <div className="metric-card-header widget-header">
        <h3 className="metric-card-title">{label}</h3>
      </div>

      <div className="metric-card-content">
        <div className="metric-card-value">{value}</div>
        <div className="metric-card-unit">{unit}</div>
      </div>
    </div>
  );
}
