import GridLayout from "react-grid-layout";
import "react-grid-layout/css/styles.css";
import "react-resizable/css/styles.css";
import "./Dashboard.css";
import { useRobotData } from "../../hooks/useRobotData";
import { useDashboardLayout } from "../../hooks/useDashboardLayout";
import { MetricsWidget } from "../../components/widgets/MetricsWidget/MetricsWidget";
import { MapWidget } from "../../components/widgets/MapWidget/MapWidget";

export function Dashboard() {
  const { data } = useRobotData();
  const defaultLayout = [
    { i: "map", x: 0, y: 0, w: 8, h: 10, minW: 4, minH: 6 },
    { i: "metrics", x: 8, y: 0, w: 4, h: 10, minW: 3, minH: 6 },
  ];
  const [layout, handleLayoutChange] = useDashboardLayout(defaultLayout);
  return (
    <div className="dashboard">
      <GridLayout
        className="dashboard__grid"
        layout={layout}
        onLayoutChange={handleLayoutChange}
        cols={12}
        rowHeight={30}
        width={1200}
        draggableHandle=".widget-header"
      >
        <div key="map" className="dashboard__widget">
          <MapWidget gps={data.gps} path={data.path} />
        </div>

        <div key="metrics" className="dashboard__widget">
          <MetricsWidget gps={data.gps} odom={data.odom} path={data.path} />
        </div>
      </GridLayout>
    </div>
  );
}
