import { COLORS } from "../../config/constants";
import "./StatusBar.css";
export const StatusBar = ({
  connectionStatus,
  robotStatus,
  isStale,

  onResetLayout,
  isConnected,
}) => {
  const getStatusInfo = () => {
    if (connectionStatus !== "connected") {
      return {
        color: COLORS.error,
        icon: "🔴",
        text: "Disconnected",
      };
    }
    if (isStale) {
      return {
        color: COLORS.warning,
        icon: "🟡",
        text: "Data Stale",
      };
    }
    if (robotStatus === "active") {
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

  const getStatusText = () => {
    if (!isConnected) return "Disconnected";
    if (isStale) return "Data Stale"; // ✅ הצג stale
    if (robotStatus === "active") return "Active";
    if (robotStatus === "waiting") return "Waiting";
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
        {isStale && (
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
