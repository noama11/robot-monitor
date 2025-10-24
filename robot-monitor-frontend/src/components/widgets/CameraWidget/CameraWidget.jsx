import React from "react";
import "./CameraWidget.css";

export function CameraWidget({ image }) {
  return (
    <div className="camera-widget">
      <div className="camera-widget-header widget-header">
        <h3 className="camera-widget-title">Camera</h3>
      </div>

      <div className="camera-widget-content">
        {image ? (
          <img
            src={`data:image/${image.format};base64,${image.data}`}
            alt="Robot camera view"
            className="camera-widget-image"
          />
        ) : (
          <div className="camera-widget-placeholder">
            <div className="camera-widget-placeholder-icon">📷</div>
            <p className="camera-widget-placeholder-text">
              Camera disabled or no image received
            </p>
          </div>
        )}
      </div>

      {image && (
        <div className="camera-widget-footer">
          Format: {image.format} | Updated:{" "}
          {new Date(image.timestamp * 1000).toLocaleTimeString()}
        </div>
      )}
    </div>
  );
}
