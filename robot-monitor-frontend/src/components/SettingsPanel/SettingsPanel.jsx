import "./SettingsPanel.css";

export function SettingsPanel({
  availableWidgets,
  visibleWidgets,
  onToggleWidget,
  onClose,
}) {
  return (
    <div className="settings-overlay" onClick={onClose}>
      <div className="settings-panel" onClick={(e) => e.stopPropagation()}>
        <div className="settings-header">
          <h2 className="settings-title">⚙️ Dashboard Settings</h2>
          <button className="settings-close" onClick={onClose}>
            ✕
          </button>
        </div>

        <div className="settings-content">
          <p className="settings-desc">
            Choose which components to display on your dashboard:
          </p>

          <div className="settings-list">
            {availableWidgets.map((widget) => (
              <label key={widget.id} className="settings-item">
                <input
                  type="checkbox"
                  checked={visibleWidgets.includes(widget.id)}
                  onChange={() => onToggleWidget(widget.id)}
                  className="settings-checkbox"
                />
                <span className="settings-icon">{widget.icon}</span>
                <div className="settings-info">
                  <div className="settings-name">{widget.name}</div>
                  <div className="settings-desc-small">
                    {widget.description}
                  </div>
                </div>
              </label>
            ))}
          </div>

          <div className="settings-footer">
            <button
              className="settings-btn settings-btn--secondary"
              onClick={() => {
                availableWidgets.forEach((w) => {
                  if (!visibleWidgets.includes(w.id)) onToggleWidget(w.id);
                });
              }}
            >
              Show All
            </button>
            <button
              className="settings-btn settings-btn--primary"
              onClick={onClose}
            >
              Done
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
