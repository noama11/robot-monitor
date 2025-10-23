import { COLORS } from "../../config/constants";
import "./StatusBar.css";
export const StatusBar = ({ connectionStatus, robotStatus, isStale }) => {
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
  const status = getStatusInfo();
  return (
    <header className="robot-header">
      <h1 className="robot-header__title">Monitor</h1>

      {/* Pass status color once as a CSS variable */}
      <div className="robot-status" style={{ "--status-color": status.color }}>
        <span className="robot-status__icon">{status.icon}</span>
        <span className="robot-status__text">{status.text}</span>
      </div>
    </header>
  );
};
