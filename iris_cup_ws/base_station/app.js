/**
 * IRIS Base Station Application
 * Main application logic for receiving and displaying lane detection data
 */

// Resolve ROSLIB in both browser (global) and Node environments
let ROSLIB;
if (typeof window !== "undefined" && window.ROSLIB) {
  ROSLIB = window.ROSLIB;
} else if (typeof require === "function") {
  ROSLIB = require("roslib");
} else {
  throw new Error(
    "ROSLIB library not found. Make sure to load roslib.js before app.js"
  );
}

class BaseStation {
  constructor() {
    // Connection state
    this.connected = false;
    this.connectionType = "websocket";
    this.ws = null;
    this.ros = null;

    // Canvas contexts
    this.rawCanvas = document.getElementById("rawCanvas");
    this.processedCanvas = document.getElementById("processedCanvas");
    this.bevCanvas = document.getElementById("bevCanvas");
    this.rawCtx = this.rawCanvas.getContext("2d");
    this.processedCtx = this.processedCanvas.getContext("2d");
    this.bevCtx = this.bevCanvas.getContext("2d");

    // FPS tracking
    this.rawFpsCounter = new FPSCounter("rawFps");
    this.processedFpsCounter = new FPSCounter("processedFps");
    this.bevFpsCounter = new FPSCounter("bevFps");

    this.telemetryData = {
      steering_angle: 0,
      laneStatus: "Unknown",
      speed: 0,
      jarakTempuh: 0,
      laneWidth: 0,
      deviation: 0,
      batteryVoltage: 0.0
    };

    this.currentMetric = "angle";

    this.initializeEventListeners();
    this.log("System initialized. Ready to connect.", "info");
  }

  /**
   * Initialize all event listeners
   */
  initializeEventListeners() {
    // Connection type change
    document
      .getElementById("connectionType")
      .addEventListener("change", (e) => {
        this.connectionType = e.target.value;
        const rosSettings = document.getElementById("rosSettings");

        if (this.connectionType === "ros") {
          rosSettings.classList.remove("hidden");
          document.getElementById("serverUrl").placeholder =
            "ws://localhost:9090";
          document.getElementById("serverUrl").value = "ws://localhost:9090";
        } else {
          rosSettings.classList.add("hidden");
          document.getElementById("serverUrl").placeholder =
            "ws://localhost:8080";
          document.getElementById("serverUrl").value = "ws://localhost:8080";
        }
      });

    // Connect button
    document.getElementById("connectBtn").addEventListener("click", () => {
      this.connect();
    });

    // Disconnect button
    document.getElementById("disconnectBtn").addEventListener("click", () => {
      this.disconnect();
    });

    // Clear logs button
    document.getElementById("clearLogsBtn").addEventListener("click", () => {
      this.clearLogs();
    });
  }

  /**
   * Connect to server based on connection type
   */
  connect() {
    const serverUrl = document.getElementById("serverUrl").value;

    if (!serverUrl) {
      this.log("Please enter a server URL", "error");
      return;
    }

    if (this.connectionType === "ros") {
      this.connectROS(serverUrl);
    } else {
      this.connectWebSocket(serverUrl);
    }
  }

  /**
   * Connect using standard WebSocket
   */
  connectWebSocket(url) {
    this.log(`Connecting to WebSocket server: ${url}`, "info");

    try {
      this.ws = new WebSocket(url);

      this.ws.onopen = () => {
        this.connected = true;
        this.updateConnectionStatus(true);
        this.log("WebSocket connected successfully", "success");
      };

      this.ws.onmessage = (event) => {
        this.handleWebSocketMessage(event.data);
      };

      this.ws.onerror = (error) => {
        this.log(
          `WebSocket error: ${error.message || "Connection failed"}`,
          "error"
        );
      };

      this.ws.onclose = () => {
        this.connected = false;
        this.updateConnectionStatus(false);
        this.log("WebSocket disconnected", "warning");
      };
    } catch (error) {
      this.log(`Failed to connect: ${error.message}`, "error");
    }
  }

  /**
   * Connect using ROS (roslibjs)
   */
  connectROS(url) {
    this.log(`Connecting to ROS bridge: ${url}`, "info");

    try {
      this.ros = new ROSLIB.Ros({
        url: url,
      });

      this.ros.on("connection", () => {
        this.connected = true;
        this.updateConnectionStatus(true);
        this.log("ROS bridge connected successfully", "success");
        this.subscribeToROSTopics();
      });

      this.ros.on("error", (error) => {
        this.log(`ROS error: ${error}`, "error");
      });

      this.ros.on("close", () => {
        this.connected = false;
        this.updateConnectionStatus(false);
        this.log("ROS bridge disconnected", "warning");
      });
    } catch (error) {
      this.log(`Failed to connect to ROS: ${error.message}`, "error");
    }
  }

  /**
   * Subscribe to ROS topics
   */
  subscribeToROSTopics() {
    const rawTopic = document.getElementById("rosTopicRaw").value;
    const processedTopic = document.getElementById("rosTopicProcessed").value;
    const bevTopic = document.getElementById("rosTopicBev").value;
    const speedTopic = document.getElementById("rosTopicSteering").value;
    const batteryTopic = document.getElementById("rosTopicBattery") ? document.getElementById("rosTopicBattery").value : "/vehicle/battery";
    const actualSpeedTopic = document.getElementById("rosTopicActualSpeed") ? document.getElementById("rosTopicActualSpeed").value : "/vehicle/actual_speed";

    // Subscribe to raw image topic
    const rawImageListener = new ROSLIB.Topic({
      ros: this.ros,
      name: rawTopic,
      messageType: "sensor_msgs/CompressedImage",
    });

    rawImageListener.subscribe((message) => {
      this.displayImage(message.data, "raw");
      this.rawFpsCounter.tick();
    });

    // Subscribe to processed image topic
    const processedImageListener = new ROSLIB.Topic({
      ros: this.ros,
      name: processedTopic,
      messageType: "sensor_msgs/CompressedImage",
    });

    processedImageListener.subscribe((message) => {
      this.displayImage(message.data, "processed");
      this.processedFpsCounter.tick();
    });

    // Subscribe to BEV image topic
    const bevImageListener = new ROSLIB.Topic({
      ros: this.ros,
      name: bevTopic,
      messageType: "sensor_msgs/CompressedImage",
    });

    bevImageListener.subscribe((message) => {
      this.displayImage(message.data, "bev");
      this.bevFpsCounter.tick();
    });

    // Subscribe to steering angle topic
    const steeringListener = new ROSLIB.Topic({
      ros: this.ros,
      name: speedTopic,
      messageType: "std_msgs/Float32",
    });

    steeringListener.subscribe((message) => {
      this.updateTelemetry({ steering_angle: message.data });
    });

    // Subscribe to speed topic
    const speedListener = new ROSLIB.Topic({
      ros: this.ros,
      name: speedTopic,
      messageType: "std_msgs/Float32",
    });

    speedListener.subscribe((message) => {
      this.updateTelemetry({ speed: message.data });
    });

    // Subscribe to Battery Topic
    const batteryListener = new ROSLIB.Topic({
      ros: this.ros,
      name: batteryTopic,
      messageType: "std_msgs/Float32",
    });

    batteryListener.subscribe((message) => {
      this.updateTelemetry({ batteryVoltage: message.data });
    });

    // Subscribe to Actual Speed
    const actualSpeedListener = new ROSLIB.Topic({
      ros: this.ros,
      name: actualSpeedTopic,
      messageType: "std_msgs/Float32",
    });
    actualSpeedListener.subscribe((message) => {
      this.updateTelemetry({ actualSpeed: message.data });
    });

    // Subscribe to RPM
    const rpmTopic = document.getElementById("rosTopicRpm") ? document.getElementById("rosTopicRpm").value : "/vehicle/rpm";
    const rpmListener = new ROSLIB.Topic({
      ros: this.ros,
      name: rpmTopic,
      messageType: "std_msgs/Float32",
    });
    rpmListener.subscribe((message) => {
      this.updateTelemetry({ rpm: message.data });
    });

    this.log("Subscribed to ROS topics", "success");
  }

  /**
   * Handle incoming WebSocket messages
   */
  handleWebSocketMessage(data) {
    try {
      const message = JSON.parse(data);

      switch (message.type) {
        case "image_raw":
          this.displayImage(message.data, "raw");
          this.rawFpsCounter.tick();
          if (message.width && message.height) {
            document.getElementById(
              "rawResolution"
            ).textContent = `${message.width}x${message.height}`;
          }
          break;

        case "image_processed":
          this.displayImage(message.data, "processed");
          this.processedFpsCounter.tick();
          if (message.width && message.height) {
            document.getElementById(
              "processedResolution"
            ).textContent = `${message.width}x${message.height}`;
          }
          break;

        case "telemetry":
          this.updateTelemetry(message.data);
          break;

        case "steering_angle":
          this.updateTelemetry({ steering_angle: message.value });
          break;

        case "obstacle":
          this.updateTelemetry({
            obstacleDetected: message.detected,
            obstacleDistance: message.distance,
            obstaclePosition: message.position,
          });
          break;

        default:
          this.log(`Unknown message type: ${message.type}`, "warning");
      }
    } catch (error) {
      this.log(`Error parsing message: ${error.message}`, "error");
    }
  }

  /**
   * Display image on canvas
   */
  displayImage(imageData, type) {
    let canvas, ctx, placeholder;

    if (type === "raw") {
      canvas = this.rawCanvas;
      ctx = this.rawCtx;
      placeholder = document.getElementById("rawPlaceholder");
    } else if (type === "processed") {
      canvas = this.processedCanvas;
      ctx = this.processedCtx;
      placeholder = document.getElementById("processedPlaceholder");
    } else if (type === "bev") {
      canvas = this.bevCanvas;
      ctx = this.bevCtx;
      placeholder = document.getElementById("bevPlaceholder");
    }

    if (placeholder) {
      placeholder.style.display = "none";
      canvas.style.display = "block";
    }

    // Create image from base64 data
    const img = new Image();
    img.onload = () => {
      // Set canvas size to match image
      canvas.width = img.width;
      canvas.height = img.height;

      // Draw image
      ctx.drawImage(img, 0, 0);
    };

    // Handle both base64 with and without prefix
    if (imageData.startsWith("data:image")) {
      img.src = imageData;
    } else {
      img.src = `data:image/jpeg;base64,${imageData}`;
    }
  }

  /**
   * Update telemetry data and UI
   */
  updateTelemetry(data) {
    // Update internal state
    Object.assign(this.telemetryData, data);

    if (data.steering_angle !== undefined) {
      const angle = data.steering_angle;
      document.getElementById("steeringValue").textContent = `${angle.toFixed(
        1
      )}°`;
      this.updateSteeringGauge(angle);
    }

    // Update Actual Speed
    if (data.actualSpeed !== undefined) {
      document.getElementById("actualSpeedValue").textContent = `${parseFloat(data.actualSpeed).toFixed(1)} cm/s`;
    }

    // Update RPM
    if (data.rpm !== undefined) {
      // RPM biasanya angka bulat, jadi toFixed(0)
      document.getElementById("rpmValue").textContent = `${parseFloat(data.rpm).toFixed(0)}`;
    }

    // Update Jarak Tempuh
    if (data.jarakTempuh !== undefined) {
      document.getElementById("jarakTempuhValue").textContent = `${parseFloat(data.jarakTempuh).toFixed(2)} m`;
    }

    // Update Battery Detection
    if (data.batteryVoltage !== undefined) {
      const voltage = parseFloat(data.batteryVoltage);
      document.getElementById("batteryVoltage").textContent = `${voltage.toFixed(2)} V`;

      const batteryBadge = document.getElementById("batteryStatus");
      
      // Logika peringatan untuk Baterai LiPo 3S (11.1V - 12.6V)
      if (voltage >= 11.4) {
        batteryBadge.textContent = "Safe";
        batteryBadge.className = "badge badge-success"; // Warna Hijau
      } else if (voltage < 11.4 && voltage >= 10.5) {
        batteryBadge.textContent = "Low Battery";
        batteryBadge.className = "badge badge-warning"; // Warna Kuning
      } else if (voltage < 10.5 && voltage > 1.0) {
        batteryBadge.textContent = "CRITICAL DROP!";
        batteryBadge.className = "badge badge-error"; // Warna Merah (Segera Cabut!)
      } else {
        batteryBadge.textContent = "Unknown";
        batteryBadge.className = "badge badge-warning";
      }
    }

    // Update obstacle detection
    // if (data.obstacleDetected !== undefined) {
    //   const obstacleBadge = document.getElementById("obstacleStatus");
    //   obstacleBadge.textContent = data.obstacleDetected ? "Detected" : "None";
    //   obstacleBadge.className =
    //     "badge " + (data.obstacleDetected ? "badge-error" : "badge-success");
    // }

    // if (data.obstacleDistance !== undefined) {
    //   const distanceCm = Number(data.obstacleDistance) || 0;
    //   document.getElementById("obstacleDistance").textContent =
    //     distanceCm > 0 ? `${distanceCm.toFixed(0)} cm` : "- cm";
    // }

    // if (data.obstaclePosition !== undefined) {
    //   document.getElementById("obstaclePosition").textContent =
    //     data.obstaclePosition || "-";
    // }
  }

  /**
   * Update steering gauge visualization
   */
  updateSteeringGauge(angle) {
    // Clamp angle between -90 and 90
    const clampedAngle = Math.max(-90, Math.min(90, angle));

    // Calculate gauge value (0 to 251.2, where 125.6 is center)
    const gaugeValue = 125.6 - (clampedAngle / 90) * 125.6;
    document.getElementById("gaugeValue").style.strokeDashoffset = gaugeValue;

    // Rotate needle
    const needleRotation = clampedAngle;
    document.getElementById(
      "gaugeNeedle"
    ).style.transform = `rotate(${needleRotation}deg)`;
  }

  /**
   * Disconnect from server
   */
  disconnect() {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }

    this.connected = false;
    this.updateConnectionStatus(false);
    this.log("Disconnected from server", "info");
  }

  /**
   * Update connection status UI
   */
  updateConnectionStatus(connected) {
    const statusIndicator = document.getElementById("connectionStatus");
    const statusText = document.getElementById("connectionText");
    const connectBtn = document.getElementById("connectBtn");
    const disconnectBtn = document.getElementById("disconnectBtn");

    if (connected) {
      statusIndicator.className = "status-indicator status-connected";
      statusText.textContent = "Connected";
      connectBtn.disabled = true;
      disconnectBtn.disabled = false;
    } else {
      statusIndicator.className = "status-indicator status-disconnected";
      statusText.textContent = "Disconnected";
      connectBtn.disabled = false;
      disconnectBtn.disabled = true;
    }
  }

  /**
   * Log message to console and UI
   */
  log(message, type = "info") {
    const timestamp = new Date().toLocaleTimeString();
    const logContainer = document.getElementById("logContainer");

    const logEntry = document.createElement("div");
    logEntry.className = "log-entry";
    logEntry.innerHTML = `
            <span class="log-timestamp">[${timestamp}]</span>
            <span class="log-${type}">${message}</span>
        `;

    logContainer.appendChild(logEntry);
    logContainer.scrollTop = logContainer.scrollHeight;

    // Also log to console
    console.log(`[${timestamp}] ${message}`);
  }

  /**
   * Clear all logs
   */
  clearLogs() {
    const logContainer = document.getElementById("logContainer");
    logContainer.innerHTML = "";
    this.log("Logs cleared", "info");
  }
}

/**
 * FPS Counter utility class
 */
class FPSCounter {
  constructor(elementId) {
    this.elementId = elementId;
    this.frames = 0;
    this.lastTime = Date.now();
    this.fps = 0;

    // Update FPS display every second
    setInterval(() => {
      const now = Date.now();
      const elapsed = (now - this.lastTime) / 1000;
      this.fps = Math.round(this.frames / elapsed);
      this.frames = 0;
      this.lastTime = now;

      document.getElementById(this.elementId).textContent = this.fps;
    }, 1000);
  }

  tick() {
    this.frames++;
  }
}

// Initialize application when DOM is ready
document.addEventListener("DOMContentLoaded", () => {
  window.baseStation = new BaseStation();
});
