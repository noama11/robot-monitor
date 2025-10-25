import { useState, useEffect, useCallback, useRef } from "react";
import { websocketService } from "../services/websocket";
import { validateRobotData } from "../utils/validation";

export function useRobotData() {
  const [data, setData] = useState({
    gps: null,
    odom: null,
    image: null,
    path: [],
    status: "no_data",
    timestamp: null,
  });
  const [connectionStatus, setConnectionStatus] = useState("disconnected");
  const [isStale, setIsStale] = useState(true);

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

  // Staleness check interval
  // useEffect(() => {
  //   const interval = setInterval(() => {
  //     if (!lastUpdateTime.current) {
  //     } else {
  //       const timeSinceUpdate = Date.now() - lastUpdateTime.current;
  //       const staleStatus = timeSinceUpdate > STALE_THRESHOLD_MS;
  //     }
  //   }, 1000);

  //   return () => clearInterval(interval);
  // }, []);

  return {
    data, // gps, odom, path, images
    connectionStatus, // 'connected' | 'disconnected' | 'error'
    isStale, // if true- data is too old
    isConnected: connectionStatus === "connected",
    hasData: data.gps !== null || data.odom !== null,
  };
}
