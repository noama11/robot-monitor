import { COLORS } from "../../config/constants";
import { useRobotContext } from "../../context/useRobotContext";
import "./StatusBar.css";

export const StatusBar = ({
  stale,
  connectionStatus,
  robotStatus,
  // isStale,
  onResetLayout,
}) => {
  const { isConnected } = useRobotContext();
  const getStatusInfo = () => {
    if (!isConnected) {
      return {
        color: COLORS.error,
        icon: "🔴",
        text: "Disconnected",
      };
    }
    if (stale) {
      return {
        color: COLORS.warning,
        icon: "🟡",
        text: "Data Stale",
      };
    }
    if (isConnected) {
      return {
        color: COLORS.success,
        icon: "🟢",
        text: "Active",
      };
    }
    return {
      color: COLORS.neutral,
      icon: "⚪",
      text: "Waiting for Data",
    };
  };

  const getStatusColor = () => {
    if (!isConnected) return "#ef4444"; // Red
    if (stale) return "#f59e0b"; // Amber/Orange
    if (data.status === "active") return "#22c55e"; // Green
    return "#6b7280"; // Gray
  };

  const getStatusText = () => {
    if (!isConnected) return "Disconnected";
    if (stale) return "Data Stale"; // ✅ הצג stale
    if (data.status === "active") return "Active";
    // if (data.status === "active") return "Waiting";
    return "No Data";
  };

  const status = getStatusInfo();
  return (
    <header className="status-bar">
      <h1 className="status-bar-title">Monitor</h1>

      <div className="status-bar-right-section">
        {/* Reset Layout Button */}
        {isConnected && (
          <button
            className="status-bar-button reset-button"
            onClick={() => {
              onResetLayout();
              // Trigger dashboard reset event
              window.dispatchEvent(new Event("dashboard-reset"));
            }}
            title="Reset to Default Layout"
          >
            ↺ Reset View
          </button>
        )}
        {stale && (
          <span className="status-bar__warning">
            ⚠️ No updates for 5+ seconds
          </span>
        )}{" "}
        {/* Status Badge */}
        <div
          className="status-badge"
          style={{ "--status-color": status.color }}
        >
          <span className="status-badge-icon">{status.icon}</span>
          <span className="status-badge-text">{status.text}</span>
        </div>
      </div>
    </header>
  );
};
