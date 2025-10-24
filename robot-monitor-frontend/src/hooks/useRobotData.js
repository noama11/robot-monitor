// import { useContext } from "react";
// import { RobotDataContext } from "../context/RobotDataContext";

// export function useRobotData() {
//   // console.log("call useRobotHook");
//   const context = useContext(RobotDataContext);

//   if (!context) {
//     throw new Error("useRobotData must be used within RobotDataProvider");
//   }

//   return context;
// }

// src/hooks/useRobotData.js

import { useState, useEffect, useCallback, useRef } from "react";
import { websocketService } from "../services/websocket";
import { validateRobotData, isDataStale } from "../utils/validation";
// import { useRobotContext } from "../context/useRobotContext";

// global flag to ensure single initialization
const STALE_THRESHOLD_MS = 5000;

export function useRobotData() {
  const [data, setData] = useState({
    gps: null,
    odom: null,
    image: null,
    path: [],
    status: "no_data",
    timestamp: null,
  });
  // const { stale, handleSetStale } = useRobotContext();
  const [connectionStatus, setConnectionStatus] = useState("disconnected");
  const [isStale, setIsStale] = useState(true);

  const prevConnectionStatusRef = useRef("disconnected");
  const lastUpdateTime = useRef(null);

  //  incoming data
  const handleData = useCallback((rawData) => {
    const val = validateRobotData(rawData);
    if (!val || !val.timestamp) return;

    setData((prevData) => {
      if (val.timestamp === prevData.timestamp) {
        return prevData; // no change - avoid re-render
      }

      // we received new data
      lastUpdateTime.current = Date.now();
      return val;
    });
  }, []);

  const handleStatusChange = useCallback((status) => {
    setConnectionStatus(status);
    // console.log("WebSocket connection status:", status);
  }, []);

  useEffect(() => {
    const unsubscribeData = websocketService.subscribe(handleData);
    const unsubscribeStatus =
      websocketService.subscribeToStatus(handleStatusChange);

    return () => {
      unsubscribeData();
      unsubscribeStatus();
    };
  }, [handleData, handleStatusChange]);

  useEffect(() => {
    const interval = setInterval(() => {
      if (!lastUpdateTime.current) {
      } else {
        const timeSinceUpdate = Date.now() - lastUpdateTime.current;
        const staleStatus = timeSinceUpdate > STALE_THRESHOLD_MS;
        // console.log("Hook Timer Check -> Should be stale:", staleStatus);
      }
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    const wasConnected = prevConnectionStatusRef.current === "connected";
    const isNowDisconnected = connectionStatus === "disconnected";

    if (wasConnected && isNowDisconnected) {
      console.log("Connection lost - clearing path");
      setData((prev) => ({
        ...prev,
        path: [],
        gps: null,
        odom: null,
      }));
    }

    prevConnectionStatusRef.current = connectionStatus;
  }, [connectionStatus]);

  return {
    data, // gps, odom, path, images
    connectionStatus, // 'connected' | 'disconnected' | 'error'
    isStale, // if true- data is too old
    isConnected: connectionStatus === "connected",
    hasData: data.gps !== null || data.odom !== null,
  };
}
