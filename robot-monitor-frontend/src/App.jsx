import { useState } from "react";
import reactLogo from "./assets/react.svg";
import viteLogo from "/vite.svg";
import "./App.css";
// import { useRobotData } from "./context/RobotDataContext";
import { useRobotData } from "./hooks/useRobotData";

function App() {
  console.log("HHHHHHHHHH");
  const { data, connectionStatus, isConnected, hasData } = useRobotData();
  return (
    <div className="app">
      <header className="header">
        <h1>Robot Monitor</h1>
        <div className={`status-badge ${connectionStatus}`}>
          {isConnected ? "🟢" : "🔴"} {connectionStatus.toUpperCase()}
        </div>
      </header>

      <main className="content">
        {!isConnected && (
          <div className="warning">
            <p>⚠️ Not connected to backend</p>
            <p>Make sure backend is running on ws://localhost:8765</p>
          </div>
        )}

        {isConnected && !hasData && (
          <div className="warning">
            <p>⏳ Connected, waiting for robot data...</p>
            <p>Is the ROS bag playing?</p>
          </div>
        )}

        {hasData && (
          <div className="data-grid">
            {/* GPS Section */}
            <section className="data-card">
              <h2>📍 GPS</h2>
              {data.gps ? (
                <div className="data-content">
                  <p>
                    <strong>Lat:</strong>{" "}
                    {data.gps.latitude?.toFixed(6) || "N/A"}
                  </p>
                  <p>
                    <strong>Lng:</strong>{" "}
                    {data.gps.longitude?.toFixed(6) || "N/A"}
                  </p>
                  <p>
                    <strong>Alt:</strong>{" "}
                    {data.gps.altitude?.toFixed(1) || "N/A"} m
                  </p>
                </div>
              ) : (
                <p className="no-data">No GPS data</p>
              )}
            </section>

            {/* Odometry Section */}
            <section className="data-card">
              <h2>🚗 Odometry</h2>
              {data.odom ? (
                <div className="data-content">
                  <p>
                    <strong>Speed:</strong>{" "}
                    {data.odom.speed?.toFixed(2) || "N/A"} m/s
                  </p>
                  <p>
                    <strong>Pos X:</strong>{" "}
                    {data.odom.position_x?.toFixed(2) || "N/A"}
                  </p>
                  <p>
                    <strong>Pos Y:</strong>{" "}
                    {data.odom.position_y?.toFixed(2) || "N/A"}
                  </p>
                </div>
              ) : (
                <p className="no-data">No odometry data</p>
              )}
            </section>

            {/* Path Section */}
            <section className="data-card">
              <h2>🛤️ Path</h2>
              <div className="data-content">
                <p>
                  <strong>Points:</strong> {data.path?.length || 0}
                </p>
                {data.path && data.path.length > 0 && (
                  <p className="small">
                    Latest: {data.path[data.path.length - 1].lat.toFixed(6)},{" "}
                    {data.path[data.path.length - 1].lng.toFixed(6)}
                  </p>
                )}
              </div>
            </section>

            {/* Status Section */}
            <section className="data-card">
              <h2>⚙️ System</h2>
              <div className="data-content">
                <p>
                  <strong>Status:</strong> {data.status}
                </p>
                <p>
                  <strong>Last Update:</strong>{" "}
                  {new Date(data.timestamp).toLocaleTimeString()}
                </p>
              </div>
            </section>
          </div>
        )}
      </main>
    </div>
  );
}

export default App;
