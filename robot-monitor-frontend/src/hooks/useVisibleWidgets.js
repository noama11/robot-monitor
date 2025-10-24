import { useState, useCallback } from "react";
import { AVAILABLE_WIDGETS } from "../config/constants";

const STORAGE_KEY = "visible-widgets";
const DEFAULT_VISIBLE_WIDGETS = [
  "map",
  "camera",
  "linear_velocity",
  "angular_velocity",
];

/**
 * Hook for managing which widgets are visible
 */

export function useVisibleWidgets() {
  // load from localStorage or use defaults
  const [visibleWidgets, setVisibleWidgets] = useState(() => {
    try {
      const saved = localStorage.getItem(STORAGE_KEY);
      return saved ? JSON.parse(saved) : DEFAULT_VISIBLE_WIDGETS;
    } catch (error) {
      console.error("Failed to load visible widgets:", error);
      return DEFAULT_VISIBLE_WIDGETS;
    }
  });

  // reset to default
  const resetToDefaults = useCallback(() => {
    setVisibleWidgets(DEFAULT_VISIBLE_WIDGETS);

    try {
      localStorage.setItem(
        STORAGE_KEY,
        JSON.stringify(DEFAULT_VISIBLE_WIDGETS)
      );
    } catch (error) {
      console.error("Failed to reset visible widgets:", error);
    }
  }, []);

  return {
    visibleWidgets,
    resetToDefaults,
  };
}
