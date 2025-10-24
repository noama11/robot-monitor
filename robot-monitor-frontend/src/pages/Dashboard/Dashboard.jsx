// import GridLayout from "react-grid-layout";
import RGL, { WidthProvider } from "react-grid-layout";
const ReactGridLayout = WidthProvider(RGL);
import { useEffect, useRef, useState } from "react";

import "react-grid-layout/css/styles.css";
import "react-resizable/css/styles.css";
import "../../styles/rgl-overrides.css";
import "./Dashboard.css";

import { useRobotData } from "../../hooks/useRobotData";
import { useDashboardLayout } from "../../hooks/useDashboardLayout";
import { MapView } from "../../components/widgets/MapView/MapView";
import { AVAILABLE_WIDGETS } from "../../config/constants";
import { SpeedWidget } from "../../components/widgets/SpeedWidget/SpeedWidget";
import { CameraWidget } from "../../components/widgets/CameraWidget/CameraWidget";
export function Dashboard() {
  const { data } = useRobotData();

  const { layout, saveLayout, resetLayout } =
    useDashboardLayout(AVAILABLE_WIDGETS);

  useEffect(() => {
    const handleReset = () => resetLayout();
    window.addEventListener("dashboard-reset", handleReset);
    return () => window.removeEventListener("dashboard-reset", handleReset);
  }, [resetLayout]);

  // render widget by ID

  const renderWidget = (widgetId) => {
    switch (widgetId) {
      case "map":
        return <MapView gps={data.gps} path={data.path} odom={data.odom} />;

      case "camera":
        return <CameraWidget image={data.image} />;

      case "speed":
        return <SpeedWidget odom={data.odom} />;

      default:
        return null;
    }
  };

  return (
    <div className="dashboard">
      <ReactGridLayout
        className="dashboard-grid"
        layout={layout}
        onLayoutChange={saveLayout}
        cols={12}
        rowHeight={Math.floor((window.innerHeight - 60 - 8 - 32) / 40)}
        isResizable={true}
        isDraggable={true}
        isBounded={true}
        resizeHandles={["se", "sw", "ne", "nw"]}
        compactType="vertical"
        preventCollision={true}
        draggableHandle=".widget-header"
        draggableCancel=".map-widget, .leaflet-container, .map-interactive"
      >
        {AVAILABLE_WIDGETS.map((widget) => (
          <div
            className="dashboard-widget"
            key={widget.id}
            data-grid={layout.find((l) => l.i === widget.id)}
          >
            {renderWidget(widget.id)}
          </div>
        ))}
      </ReactGridLayout>
    </div>
  );
}
