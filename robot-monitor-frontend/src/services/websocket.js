import { WEBSOCKET_CONFIG } from "../config/constants";
/**
 * WebSocket service for robot data connection
 * Handles connection lifecycle, reconnection, and error recovery
 */
class WebSocketService {
  constructor() {
    this.ws = null; // only one instance of WebSocket
    this.reconnectAttempts = 0;
    this.reconnectTimeout = null; //timout ID (setTimout returns ID for the timer it created)
    this.listeners = new Set(); // data listeners
    this.statusListeners = new Set();
    this.isIntentionalClose = false;
    this.isConnecting = false;
  }
  connect() {
    // prevent duplicate connections
    if (this.ws?.readyState === WebSocket.OPEN) {
      console.warn("WebSocket already connected");
      return;
    }
    if (this.isConnecting) {
      console.warn("WebSocket connection already in progress");
      return;
    }
    this.isConnecting = true;
    this.isIntentionalClose = false;

    try {
      this.ws = new WebSocket(WEBSOCKET_CONFIG.url);
      this._setupEventHandlers();
    } catch (error) {
      console.error("Failed to create WebSocket:", error);
      this.isConnecting = false;
      this._notifyStatus("error");
      this._scheduleReconnect();
    }
  }

  _setupEventHandlers() {
    if (!this.ws) return;

    this.ws.onopen = () => {
      console.log("WebSocket connected to", WEBSOCKET_CONFIG.url);
      this.reconnectAttempts = 0; // reset counter on success
      this.isConnecting = false;
      this._notifyStatus("connected");
    };

    // data received
    this.ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        // console.log("Data received from backend:", receivedData);
        this._notifyListeners(data);
      } catch (error) {
        console.error("Failed to parse WebSocket message:", error);
        // don't crash- just skip
      }
    };

    this.ws.onerror = (error) => {
      console.error("WebSocket error:", error);
      this.isConnecting = false;
      this._notifyStatus("error");
    };

    // when the connection is closed: There are two possibilities:
    // 1- closed intentionally or 2- closed suddenly and then try to reconnect
    this.ws.onclose = (event) => {
      console.log(
        "WebSocket closed. Code:",
        event.code,
        "Reason:",
        event.reason
      );
      this.ws = null;
      this.isConnecting = false;
      this._notifyStatus("disconnected");

      // Only reconnect if not intentionally closed by user
      if (!this.isIntentionalClose) {
        this._scheduleReconnect();
      }
    };
  }

  _scheduleReconnect() {
    // stop if exceeded max
    if (this.reconnectAttempts >= WEBSOCKET_CONFIG.maxReconnectAttempts) {
      console.error("Max reconnection attempts reached. Giving up.");
      this._notifyStatus("error");
      return;
    }

    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout); // clear existingtimeout
    }

    this.reconnectAttempts++;

    // exponential backoff
    const delay = WEBSOCKET_CONFIG.reconnectDelay * this.reconnectAttempts;

    console.log(
      `Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts}/${WEBSOCKET_CONFIG.maxReconnectAttempts})`
    );

    this.reconnectTimeout = setTimeout(() => {
      console.log("Attempting to reconnect...");
      this.connect();
    }, delay);
  }

  // broadcasts to all subscribers
  _notifyListeners(data) {
    this.listeners.forEach((listener) => {
      try {
        listener(data);
      } catch (error) {
        console.error("Listener callback error:", error);

        this.listeners.delete(listener); // if failed remove this listener
      }
    });
  }

  // broadcasts to all subscribers
  _notifyStatus(status) {
    this.statusListeners.forEach((listener) => {
      try {
        listener(status);
      } catch (error) {
        console.error("Status listener callback error:", error);
        this.statusListeners.delete(listener);
      }
    });
  }

  // callback - called with data when received
  subscribe(callback) {
    if (typeof callback !== "function") {
      throw new Error("Callback must be a function");
    }

    this.listeners.add(callback);

    return () => this.listeners.delete(callback);
  }

  // callback -called with status  when received
  subscribeToStatus(callback) {
    if (typeof callback !== "function") {
      throw new Error("Callback must be a function");
    }

    this.statusListeners.add(callback);

    return () => this.statusListeners.delete(callback);
  }

  disconnect() {
    this.isIntentionalClose = true; // set to intentionally disconnecting
    this.isConnecting = false;
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout); // cancel future reconnect timers
      this.reconnectTimeout = null; // sets flag to prevent auto reconnect
    }

    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    this._notifyStatus("disconnected");
  }

  // get current connection status
  getStatus() {
    if (!this.ws) return "disconnected";

    switch (this.ws.readyState) {
      case WebSocket.CONNECTING:
        return "connecting";
      case WebSocket.OPEN:
        return "connected";
      case WebSocket.CLOSING:
      case WebSocket.CLOSED:
        return "disconnected";
      default:
        return "error";
    }
  }
}

export const websocketService = new WebSocketService();
