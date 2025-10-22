import {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
} from "react";
import { websocketService } from "../services/websocket";
import { validateRobotData, isDataStale } from "../utils/validation";

export const RobotDataContext = createContext(null);

export function RobotDataProvider({ children }) {
  const [data, setData] = useState({
    gps: null,
    odom: null,
    image: null,
    path: [],
    status: "no_data",
    timestamp: null,
  });

  const [connectionStatus, setConnectionStatus] = useState("disconnected");
  const [isStale, setIsStale] = useState(false);

  //  incoming data
  const handleData = useCallback((rawData) => {
    const val = validateRobotData(rawData);
    if (val) {
      setData(val);
      setIsStale(false);
    }
  }, []);

  const handleStatusChange = useCallback((status) => {
    setConnectionStatus(status);
  }, []);

  useEffect(() => {
    const unsubDataFunc = websocketService.subscribe(handleData);
    const unubeStatusFunc =
      websocketService.subscribeToStatus(handleStatusChange);

    websocketService.connect();

    return () => {
      unsubDataFunc();
      unubeStatusFunc();
      websocketService.disconnect();
    };
  }, [handleData, handleStatusChange]);

  // check if data is stale
  useEffect(() => {
    const interval = setInterval(() => {
      if (data.timestamp) {
        setIsStale(isDataStale(data.timestamp));
      }
    }, 1000); // check every sec

    return () => clearInterval(interval);
  }, [data.timestamp]);

  const value = {
    data, // gps, odom, path, images
    connectionStatus, // 'connected' | 'disconnected' | 'error'
    isStale, // if true- data is too old
    isConnected: connectionStatus === "connected",
    hasData: data.gps !== null || data.odom !== null,
  };

  return (
    <RobotDataContext.Provider value={value}>
      {children}
    </RobotDataContext.Provider>
  );
}
