import { useContext } from "react";
import { RobotDataContext } from "../context/RobotDataContext";

export function useRobotData() {
  console.log("call useRobotHook");
  const context = useContext(RobotDataContext);

  if (!context) {
    throw new Error("useRobotData must be used within RobotDataProvider");
  }

  return context;
}
