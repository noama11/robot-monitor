import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { RobotDataProvider } from "./context/RobotDataContext.jsx";
import App from "./App.jsx";
import "./index.css";

createRoot(document.getElementById("root")).render(
  <StrictMode>
    <RobotDataProvider>
      <App />
    </RobotDataProvider>
  </StrictMode>
);
