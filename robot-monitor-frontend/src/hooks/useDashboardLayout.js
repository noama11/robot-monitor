import { useState, useEffect, useRef, useCallback } from "react";

const STORAGE_KEY = "dashboard-layout";

// default layout config
// const DEFAULT_LAYOUTS = {
//   map: { x: 0, y: 0, w: 8, h: 20, minW: 4, minH: 10 },
//   camera: { x: 8, y: 0, w: 4, h: 10, minW: 3, minH: 6 },
//   linear_velocity: { x: 8, y: 10, w: 4, h: 5, minW: 3, minH: 4 },
//   angular_velocity: { x: 8, y: 15, w: 4, h: 5, minW: 3, minH: 4 },
// };

export function useDashboardLayout(widgets) {
  const [layout, setLayout] = useState(() => {
    try {
      const saved = localStorage.getItem(STORAGE_KEY);
      if (saved) {
        return JSON.parse(saved);
      }
    } catch (error) {
      console.error("Failed to load layout:", error);
    }

    // default layout for visible widgets
    return generateLayout(widgets);
  });

  // save layout to localStorage
  const saveLayout = useCallback((newLayout) => {
    setLayout(newLayout);

    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(newLayout));
    } catch (error) {
      console.error("Failed to save layout:", error);
    }
  }, []);

  // Reset layout to defaults
  const resetLayout = useCallback(() => {
    const defaultLayout = generateLayout(widgets);
    setLayout(defaultLayout);

    try {
      localStorage.removeItem(STORAGE_KEY);
    } catch (error) {
      console.error("Failed to reset layout:", error);
    }
  }, [widgets]);

  const handleLayoutChange = (newLayout) => {
    setLayout(newLayout);
    localStorage.setItem("dashboard-layout", JSON.stringify(newLayout));
  };

  return {
    layout,
    // handleLayoutChange,
    // containerRef,
    // containerWidth,
    saveLayout,
    resetLayout,
  };
}

function generateLayout(widgets) {
  return widgets.map((widget) => ({
    i: widget.id,
    x: widget.defaultLayout.x,
    y: widget.defaultLayout.y,
    w: widget.defaultLayout.w,
    h: widget.defaultLayout.h,
    minW: widget.defaultLayout.minW,
    minH: widget.defaultLayout.minH,
  }));
}
