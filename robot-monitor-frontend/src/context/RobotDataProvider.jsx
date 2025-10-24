import { useState } from "react";
import { useRobotData } from "../hooks/useRobotData";
import { RobotDataContext } from "./RobotContext.js";
// export const RobotDataContext = createContext(null);

export function RobotDataProvider({ children }) {
  const robotData = useRobotData();
  const [resetCount, setResetCount] = useState(0);
  const [stale, setStale] = useState(true);
  const handleSetStale = (value) => {
    setStale(value);
  };
  const resetLayout = () => {
    setResetCount((prev) => prev + 1);
  };
  const value = {
    ...robotData,
    resetCount,
    resetLayout,
    handleSetStale,
    stale,
  };
  return (
    <RobotDataContext.Provider value={value}>
      {children}
    </RobotDataContext.Provider>
  );
}
