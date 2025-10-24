import "./App.css";
import { useState } from "react";

// import { useRobotData } from "./context/RobotDataContext";
import { useRobotData } from "./hooks/useRobotData";
import { StatusBar } from "./components/StatusBar/StatusBar";
import { Dashboard } from "./pages/Dashboard/Dashboard";
import { WEBSOCKET_CONFIG } from "./config/constants";
function App() {
  const { data, connectionStatus, isConnected, hasData, isStale } =
    useRobotData();

  // reset everything to default
  const handleResetAll = () => {
    window.dispatchEvent(new Event("dashboard-reset"));
  };
  return (
    <div className="app">
      <StatusBar
        connectionStatus={connectionStatus}
        robotStatus={data.status}
        isStale={isStale}
        onResetLayout={handleResetAll}
        isConnected={isConnected}
      />

      {!isConnected ? (
        <div className="message">
          <div className="warning">
            <p>Not connected to backend</p>
            <p>Make sure backend is running on {WEBSOCKET_CONFIG.url}</p>
          </div>
        </div>
      ) : !hasData ? (
        <div className="app__message">
          <div className="warning">
            <p>⏳ Waiting for robot data...</p>
          </div>
        </div>
      ) : (
        <Dashboard />
      )}
    </div>
  );
}

export default App;
