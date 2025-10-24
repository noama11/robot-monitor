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
import { MetricsWidget } from "../../components/widgets/MetricsWidget/MetricsWidget";
import { MapView } from "../../components/widgets/MapView/MapView";
import { AVAILABLE_WIDGETS } from "../../config/constants";
import { VelocityWidget } from "../../components/widgets/VelocityWidget/VelocityWidget";
import { SpeedWidget } from "../../components/widgets/SpeedWidget/SpeedWidget";
import { CameraWidget } from "../../components/widgets/CameraWidget/CameraWidget";
export function Dashboard() {
  const { data } = useRobotData();

  // const defaultLayout = [
  //   { i: "map", x: 0, y: 0, w: 8, h: 10, minW: 4, minH: 6 },
  //   { i: "metrics", x: 8, y: 0, w: 4, h: 10, minW: 3, minH: 6 },
  // ];

  const { layout, saveLayout, resetLayout } =
    useDashboardLayout(AVAILABLE_WIDGETS);

  // const containerRef = useRef(null);
  // const [containerWidth, setContainerWidth] = useState(1200);

  // Measure container width for responsive grid
  // useEffect(() => {
  //   if (!containerRef.current) return;

  //   const updateWidth = () => {
  //     setContainerWidth(containerRef.current.offsetWidth);
  //   };

  //   updateWidth();
  //   window.addEventListener("resize", updateWidth);
  //   return () => window.removeEventListener("resize", updateWidth);
  // }, []);

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
        // rowHeight={30}
        rowHeight={Math.floor((window.innerHeight - 60 - 8 - 32) / 40)}
        isResizable={true}
        isDraggable={true}
        // width={containerWidth}
        isBounded={true}
        resizeHandles={["se", "sw", "ne", "nw"]}
        // draggableCancel=".no-drag"
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
