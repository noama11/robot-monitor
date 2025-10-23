import { useState, useEffect } from "react";

export function useDashboardLayout(defaultLayout) {
  const [layout, setLayout] = useState(() => {
    const savedLayout = localStorage.getItem("dashboardLayout");
    return savedLayout ? JSON.parse(savedLayout) : defaultLayout;
  });

  useEffect(() => {
    localStorage.setItem("dashboard-layout", JSON.stringify(layout));
  }, [layout]);
  const handleLayoutChange = (newLayout) => {
    setLayout(newLayout);
  };

  return [layout, handleLayoutChange];
}
