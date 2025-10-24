import { useContext } from "react";
import { RobotDataContext } from "./RobotContext";

export function useRobotContext() {
  const context = useContext(RobotDataContext);

  if (context === null) {
    throw new Error("useRobotContext must be used within a RobotDataProvider");
  }

  return context;
}
