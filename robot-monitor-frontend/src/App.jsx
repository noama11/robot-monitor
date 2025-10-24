import "./App.css";
import { useEffect } from "react";

import { websocketService } from "./services/websocket";
import { useRobotData } from "./hooks/useRobotData";
import { StatusBar } from "./components/StatusBar/StatusBar";
import { Dashboard } from "./pages/Dashboard/Dashboard";
import { WEBSOCKET_CONFIG } from "./config/constants";
import { useRobotContext } from "./context/useRobotContext";

function App() {
  const { data, connectionStatus, isConnected, hasData, isStale } =
    useRobotData();
  const { stale } = useRobotContext();

  // reset everything to default
  const handleResetAll = () => {
    window.dispatchEvent(new Event("dashboard-reset"));
  };

  useEffect(() => {
    console.log("App mounted. Connecting WebSocket service...");
    websocketService.connect();

    return () => {
      console.log("App unmounting. Disconnecting WebSocket service...");
      websocketService.disconnect();
    };
  }, []);
  return (
    <div className="app">
      <StatusBar
        connectionStatus={connectionStatus}
        robotStatus={data.status}
        isStale={isStale}
        onResetLayout={handleResetAll}
        isConnected={isConnected}
      />
      <main className="app-main">
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
      </main>
      <div className="bottom-bumper" aria-hidden="true" />{" "}
    </div>
  );
}

export default App;
