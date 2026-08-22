const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];

const clamp = (value, min, max) => Math.min(Math.max(value, min), max);
const lerp = (a, b, amount) => a + (b - a) * amount;
const formatSigned = (value) => `${value >= 0 ? "+" : ""}${value.toFixed(2)}`;

const quaternionFromEuler = (roll, pitch, yaw) => {
  const cr = Math.cos(roll / 2); const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2); const sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2); const sy = Math.sin(yaw / 2);
  return {
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
    w: cr * cp * cy + sr * sp * sy,
  };
};

const mockImu = (sensorId, role, moduleIndex) => ({
  sensorId,
  displayId: `0x${sensorId}`,
  role,
  moduleIndex,
  online: true,
  ageSec: 0,
  quaternion: { x: 0, y: 0, z: 0, w: 1 },
  eulerDeg: { roll: 0, pitch: 0, yaw: 0 },
});

class MissionStore extends EventTarget {
  constructor() {
    super();
    this.state = {
      source: "simulation",
      connected: true,
      missionSeconds: 0,
      pose: { x: 0, y: 0, z: 0, yaw: 0 },
      start: { x: 0, y: 0, z: 0, yaw: 0 },
      path: [{ x: 0, y: 0, z: 0 }],
      exploredCells: new Set(),
      occupiedCells: new Set(),
      mapCellSize: 0.42,
      cloudPoints: [],
      cloudPointCount: 0,
      cloudSessionId: null,
      segmentPoses: [],
      cameraFrame: null,
      distanceTraveled: 0,
      distanceTraveled3d: 0,
      distanceSlamCorrected: 0,
      distanceFromStart: 0,
      mapNodes: 1,
      mapRate: 2,
      battery: 87,
      latency: 24,
      rates: {
        RGB: { value: 30, expected: 30, ok: true },
        DEPTH: { value: 30, expected: 30, ok: true },
        VO: { value: 29.6, expected: 30, ok: true },
        SLAM: { value: 2, expected: 2, ok: true },
        IMU50: { value: 45, expected: 45, ok: true },
        IMU51: { value: 45, expected: 45, ok: true },
        IMU52: { value: 45, expected: 45, ok: true },
        EKF: { value: 45, expected: 45, ok: true },
      },
      imus: {
        50: mockImu("50", "HEAD", 1),
        51: mockImu("51", "MIDDLE", 2),
        52: mockImu("52", "TAIL", 3),
      },
      snakeModel: {
        moduleCount: 7,
        modules: [
          { moduleIndex: 1, active: true, sensorId: "50" },
          { moduleIndex: 2, active: true, sensorId: "51" },
          { moduleIndex: 3, active: true, sensorId: "52" },
          { moduleIndex: 4, active: false, sensorId: null },
          { moduleIndex: 5, active: false, sensorId: null },
          { moduleIndex: 6, active: false, sensorId: null },
          { moduleIndex: 7, active: false, sensorId: null },
        ],
        activeModules: [
          { sensorId: "50", displayId: "0x50", role: "HEAD", moduleIndex: 1, translationKnown: true, translation: { x: -0.05, y: 0, z: 0.02 }, visualLength: 0.28 },
          { sensorId: "51", displayId: "0x51", role: "MIDDLE", moduleIndex: 2, translationKnown: false, translation: null, visualLength: 0.28 },
          { sensorId: "52", displayId: "0x52", role: "TAIL", moduleIndex: 3, translationKnown: false, translation: null, visualLength: 0.28 },
        ],
      },
      personDetection: {
        detected: false,
        count: 0,
        candidates: [],
        modelReady: false,
        inferenceMs: 0,
      },
      target: null,
      events: [],
    };
  }

  update(patch) {
    Object.assign(this.state, patch);
    this.dispatchEvent(new CustomEvent("update", { detail: this.state }));
  }

  addEvent(message, level = "info") {
    this.state.events.unshift({
      time: new Date(),
      message,
      level,
    });
    this.state.events = this.state.events.slice(0, 24);
  }
}

class MockTelemetrySource {
  constructor(store) {
    this.store = store;
    this.route = [
      [0, 0], [0.8, 0.1], [1.6, 0.25], [2.4, 0.2], [3.1, -0.35],
      [3.7, -1.15], [4.5, -1.5], [5.3, -1.15], [5.9, -0.25],
      [5.8, 0.8], [5.15, 1.6], [4.25, 2.05], [3.1, 2.25],
      [2.0, 2.15], [1.1, 2.7], [0.15, 3.05], [-0.85, 2.8],
      [-1.55, 2.1], [-1.85, 1.15], [-1.4, 0.35], [-0.55, 0.05],
    ];
    this.segment = 0;
    this.progress = 0;
    this.lastTick = performance.now();
    this.lastPathPoint = { x: 0, y: 0, z: 0 };
    this.lastMapNodeTime = 0;
    this.elapsed = 0;
  }

  start() {
    this.store.addEvent("온라인 SLAM 세션을 시작했습니다.");
    this.store.addEvent("D435 RGB-D 스트림 연결 완료");
    this.timer = window.setInterval(() => this.tick(), 100);
  }

  stop() {
    window.clearInterval(this.timer);
  }

  tick() {
    const now = performance.now();
    const dt = Math.min((now - this.lastTick) / 1000, 0.25);
    this.lastTick = now;
    this.elapsed += dt;
    this.progress += dt * 0.17;

    if (this.progress >= 1) {
      this.progress -= 1;
      this.segment = (this.segment + 1) % this.route.length;
    }

    const current = this.route[this.segment];
    const next = this.route[(this.segment + 1) % this.route.length];
    const x = lerp(current[0], next[0], this.progress);
    const y = lerp(current[1], next[1], this.progress);
    const yaw = Math.atan2(next[1] - current[1], next[0] - current[0]);
    const pose = { x, y, z: 0.08 + Math.sin(this.elapsed * 2.1) * 0.015, yaw };
    const step = Math.hypot(x - this.lastPathPoint.x, y - this.lastPathPoint.y);

    if (step > 0.045) {
      this.store.state.path.push({ ...pose });
      this.store.state.distanceTraveled += step;
      this.store.state.distanceTraveled3d += Math.hypot(
        x - this.lastPathPoint.x,
        y - this.lastPathPoint.y,
        pose.z - this.lastPathPoint.z,
      );
      this.store.state.distanceSlamCorrected = this.store.state.distanceTraveled;
      this.lastPathPoint = { ...pose };
    }

    this.revealAround(x, y, 1.65);
    const distanceFromStart = Math.hypot(
      x - this.store.state.start.x,
      y - this.store.state.start.y,
    );

    if (this.elapsed - this.lastMapNodeTime > 1 / 3) {
      this.store.state.mapNodes += 1;
      this.lastMapNodeTime = this.elapsed;
    }

    const jitter = (base, spread) => Math.max(0, base + Math.sin(this.elapsed * 1.7 + base) * spread);
    const rates = {
      RGB: { value: jitter(29.9, 0.35), expected: 30, ok: true },
      DEPTH: { value: jitter(29.8, 0.45), expected: 30, ok: true },
      VO: { value: jitter(29.4, 0.75), expected: 30, ok: true },
      SLAM: { value: jitter(2.0, 0.08), expected: 2, ok: true },
      IMU50: { value: jitter(44.2, 0.55), expected: 45, ok: true },
      IMU51: { value: jitter(44.1, 0.65), expected: 45, ok: true },
      IMU52: { value: jitter(44.2, 0.6), expected: 45, ok: true },
      EKF: { value: jitter(45.0, 0.4), expected: 45, ok: true },
    };
    const imuAngles = [
      [Math.sin(this.elapsed * 1.1) * 8, Math.sin(this.elapsed * 0.7) * 5, yaw * 180 / Math.PI],
      [Math.sin(this.elapsed * 1.1 + 0.8) * 13, Math.sin(this.elapsed * 0.8 + 0.4) * 8, yaw * 180 / Math.PI + Math.sin(this.elapsed) * 18],
      [Math.sin(this.elapsed * 1.1 + 1.6) * 18, Math.sin(this.elapsed * 0.8 + 0.9) * 10, yaw * 180 / Math.PI + Math.sin(this.elapsed + 0.7) * 28],
    ];
    const imus = { ...this.store.state.imus };
    ["50", "51", "52"].forEach((sensorId, index) => {
      const [roll, pitch, imuYaw] = imuAngles[index];
      imus[sensorId] = {
        ...imus[sensorId],
        eulerDeg: { roll, pitch, yaw: imuYaw },
        quaternion: quaternionFromEuler(
          roll * Math.PI / 180,
          pitch * Math.PI / 180,
          imuYaw * Math.PI / 180,
        ),
      };
    });

    let target = this.store.state.target;
    if (!target && this.elapsed > 46) {
      target = { id: "A-01", x: 4.8, y: 1.75, confidence: 0.82 };
      this.store.addEvent("사람 후보 A-01을 감지했습니다.", "warning");
    }

    this.store.update({
      missionSeconds: this.elapsed,
      pose,
      distanceFromStart,
      rates,
      imus,
      mapRate: rates.SLAM.value,
      battery: clamp(87 - this.elapsed / 1100, 15, 100),
      latency: Math.round(jitter(24, 4)),
      target,
    });
  }

  revealAround(x, y, radius) {
    const size = 0.42;
    const cellRadius = Math.ceil(radius / size);
    const centerX = Math.round(x / size);
    const centerY = Math.round(y / size);
    for (let ix = -cellRadius; ix <= cellRadius; ix += 1) {
      for (let iy = -cellRadius; iy <= cellRadius; iy += 1) {
        if (Math.hypot(ix, iy) * size <= radius) {
          this.store.state.exploredCells.add(`${centerX + ix},${centerY + iy}`);
        }
      }
    }
  }
}

class WebSocketTelemetrySource {
  constructor(store, url) {
    this.store = store;
    this.url = url;
    this.retryDelay = 1000;
    this.sparseCloudStreak = 0;
    this.sessionId = null;
  }

  start() {
    this.connect();
  }

  connect() {
    this.socket = new WebSocket(this.url);
    this.socket.binaryType = "arraybuffer";
    this.store.update({ source: "websocket", connected: false });
    this.socket.addEventListener("open", () => {
      this.retryDelay = 1000;
      this.store.addEvent("로봇 gateway에 연결했습니다.");
      this.store.update({ connected: true });
      this.pingTimer = window.setInterval(() => {
        if (this.socket.readyState === WebSocket.OPEN) {
          this.socket.send(JSON.stringify({ type: "ping", sent: performance.now() }));
        }
      }, 2000);
    });
    this.socket.addEventListener("message", (event) => {
      if (event.data instanceof ArrayBuffer) {
        this.applyBinaryMessage(event.data);
        return;
      }
      try {
        this.applyMessage(JSON.parse(event.data));
      } catch (error) {
        console.warn("Invalid gateway message", error);
      }
    });
    this.socket.addEventListener("close", () => {
      window.clearInterval(this.pingTimer);
      this.store.update({ connected: false });
      window.setTimeout(() => this.connect(), this.retryDelay);
      this.retryDelay = Math.min(this.retryDelay * 1.7, 10000);
    });
  }

  applyBinaryMessage(buffer) {
    if (buffer.byteLength < 8) return;
    const bytes = new Uint8Array(buffer, 0, 4);
    if (String.fromCharCode(...bytes) !== "SPC1") return;
    const view = new DataView(buffer);
    const declaredCount = view.getUint32(4, true);
    const count = Math.min(declaredCount, Math.floor((buffer.byteLength - 8) / 16));
    if (count <= 0) return;
    const previousCount = this.store.state.cloudPoints?.count
      ?? this.store.state.cloudPointCount
      ?? 0;
    const suspiciouslySparse = previousCount >= 500 && count < previousCount * 0.12;
    if (suspiciouslySparse) {
      this.sparseCloudStreak += 1;
      // The server already keeps a persistent voxel cache. This second guard
      // protects clients connected to an older or mixed-version gateway.
      return;
    }
    this.sparseCloudStreak = 0;
    const positions = new Float32Array(count * 3);
    const colors = new Float32Array(count * 3);
    for (let index = 0; index < count; index += 1) {
      const source = 8 + index * 16;
      const target = index * 3;
      positions[target] = view.getFloat32(source, true);
      positions[target + 1] = view.getFloat32(source + 4, true);
      positions[target + 2] = view.getFloat32(source + 8, true);
      colors[target] = view.getUint8(source + 12) / 255;
      colors[target + 1] = view.getUint8(source + 13) / 255;
      colors[target + 2] = view.getUint8(source + 14) / 255;
    }
    this.store.update({
      cloudPoints: { positions, colors, count },
      cloudPointCount: count,
    });
  }

  applyMessage(message) {
    if (message.type === "snapshot") {
      const map = message.map ?? {};
      const nextSessionId = message.sessionId ?? null;
      const sessionChanged = Boolean(
        this.sessionId && nextSessionId && this.sessionId !== nextSessionId
      );
      if (nextSessionId) this.sessionId = nextSessionId;
      const exploredCells = new Set((map.known ?? []).map((cell) => `${cell[0]},${cell[1]}`));
      const occupiedCells = new Set((map.occupied ?? []).map((cell) => `${cell[0]},${cell[1]}`));
      this.store.update({
        ...message.data,
        rates: { ...this.store.state.rates, ...(message.data.rates ?? {}) },
        exploredCells,
        occupiedCells,
        mapCellSize: map.cellSize ?? this.store.state.mapCellSize,
        // The current gateway sends the cloud immediately after this JSON as
        // an SPC1 binary frame. Preserve the last good cloud across reconnects
        // instead of replacing it with the empty compatibility field.
        cloudPoints: sessionChanged
          ? []
          : ((message.cloudPoints?.length ?? 0) > 0
          ? message.cloudPoints
          : this.store.state.cloudPoints),
        cloudPointCount: sessionChanged
          ? 0
          : (message.data.cloudPointCount ?? this.store.state.cloudPointCount),
        cloudSessionId: nextSessionId ?? this.store.state.cloudSessionId,
      });
    } else if (message.type === "pose") {
      this.store.update({
        pose: message.pose,
        distanceTraveled: message.distanceTraveled,
        distanceTraveled3d: message.distanceTraveled3d ?? this.store.state.distanceTraveled3d,
        distanceSlamCorrected: message.distanceSlamCorrected ?? this.store.state.distanceSlamCorrected,
        distanceFromStart: message.distanceFromStart,
        missionSeconds: message.missionSeconds ?? this.store.state.missionSeconds,
      });
    } else if (message.type === "start") {
      this.store.update({ start: message.pose });
    } else if (message.type === "path") {
      this.store.update({ path: message.points });
    } else if (message.type === "map_cells") {
      message.cells.forEach((cell) => this.store.state.exploredCells.add(`${cell[0]},${cell[1]}`));
      this.store.update({ mapNodes: message.mapNodes ?? this.store.state.mapNodes });
    } else if (message.type === "map") {
      const map = message.map ?? {};
      this.store.update({
        exploredCells: new Set((map.known ?? []).map((cell) => `${cell[0]},${cell[1]}`)),
        occupiedCells: new Set((map.occupied ?? []).map((cell) => `${cell[0]},${cell[1]}`)),
        mapCellSize: map.cellSize ?? this.store.state.mapCellSize,
        mapNodes: message.mapNodes ?? this.store.state.mapNodes,
      });
    } else if (message.type === "cloud") {
      // Backward compatibility with older gateways. New gateways use SPC1.
      const points = message.points ?? [];
      if (points.length > 0) this.store.update({ cloudPoints: points, cloudPointCount: points.length });
    } else if (message.type === "cloud_reset") {
      // Legacy gateways emitted false-positive resets while RTAB-Map optimized
      // its graph. A session change in the snapshot is the only valid reset.
      console.info("Ignoring legacy cloud_reset; persistent cache retained");
    } else if (message.type === "rates") {
      this.store.update({
        rates: { ...this.store.state.rates, ...message.rates },
        mapRate: message.rates.SLAM?.value ?? this.store.state.mapRate,
      });
    } else if (message.type === "event") {
      this.store.addEvent(message.message, message.level);
      this.store.update({});
    } else if (message.type === "camera") {
      if (message.mime === "image/jpeg") {
        this.store.update({ cameraFrame: `data:${message.mime};base64,${message.data}` });
      }
    } else if (message.type === "segment_poses") {
      this.store.update({ segmentPoses: message.poses ?? [] });
    } else if (message.type === "imu") {
      const imu = message.imu ?? {};
      if (imu.sensorId) {
        this.store.update({ imus: { ...this.store.state.imus, [imu.sensorId]: imu } });
      }
    } else if (message.type === "person_detection") {
      const detection = message.data ?? {};
      const wasDetected = Boolean(this.store.state.personDetection?.detected);
      if (detection.detected && !wasDetected) {
        const candidate = detection.candidates?.[0];
        const extremities = candidate?.extremities?.map((item) => item.name) ?? [];
        const detail = extremities.length > 0 ? ` (${extremities.join(", ")})` : "";
        this.store.addEvent(`사람 후보를 감지했습니다${detail}.`, "warning");
      }
      const candidate = detection.detected ? detection.candidates?.[0] : null;
      const mapPosition = candidate?.mapPosition;
      const localizedTarget = [mapPosition?.x, mapPosition?.y, mapPosition?.z]
        .every(Number.isFinite)
        ? {
          id: candidate.id ?? "P-01",
          x: mapPosition.x,
          y: mapPosition.y,
          z: mapPosition.z,
          confidence: candidate.confidence ?? 0,
          distanceMeters: candidate.distanceMeters,
          localizedAt: detection.stamp,
        }
        : this.store.state.target;
      this.store.update({ personDetection: detection, target: localizedTarget });
    } else if (message.type === "status") {
      this.store.update({
        missionSeconds: message.missionSeconds,
        imus: message.imus ?? this.store.state.imus,
      });
    } else if (message.type === "pong" && Number.isFinite(message.sent)) {
      this.store.update({ latency: Math.round(performance.now() - message.sent) });
    }
  }
}

class MapRenderer {
  constructor(canvas, store) {
    this.canvas = canvas;
    this.context = canvas.getContext("2d");
    this.store = store;
    this.mode = "2d";
    this.enabled = true;
    this.follow = true;
    this.scale = 62;
    this.offset = { x: 0, y: 0 };
    this.dragStart = null;
    this.frames = [];
    this.seed = 9381;
    this.walls = [
      [-2.8, -2.3, 6.8, -2.3], [-2.8, 3.8, 6.8, 3.8],
      [-2.8, -2.3, -2.8, 3.8], [6.8, -2.3, 6.8, 3.8],
      [-0.6, -2.3, -0.6, -0.6], [-0.6, 0.7, -0.6, 2.0],
      [1.8, -0.8, 1.8, 1.6], [1.8, 1.6, 3.7, 1.6],
      [3.7, -2.3, 3.7, -0.6], [3.7, 0.4, 3.7, 1.6],
      [5.1, -0.5, 6.8, -0.5], [4.8, 2.5, 6.8, 2.5],
    ];
    this.rubble = this.createRubble();
    this.bindEvents();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas.parentElement);
    this.resize();
    requestAnimationFrame((time) => this.draw(time));
  }

  random() {
    this.seed = (this.seed * 1664525 + 1013904223) % 4294967296;
    return this.seed / 4294967296;
  }

  createRubble() {
    const points = [];
    const clusters = [[0.7, 1.1], [2.9, -1.1], [4.5, 0.75], [5.7, 3.0], [-1.5, 2.7]];
    clusters.forEach(([cx, cy], clusterIndex) => {
      for (let index = 0; index < 55; index += 1) {
        points.push({
          x: cx + (this.random() - 0.5) * 1.15,
          y: cy + (this.random() - 0.5) * 0.85,
          z: this.random() * (0.22 + clusterIndex * 0.04),
          tone: this.random(),
        });
      }
    });
    return points;
  }

  bindEvents() {
    this.canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.follow = false;
      this.scale = clamp(this.scale * (event.deltaY > 0 ? 0.9 : 1.1), 28, 150);
      updateFollowButton(this.follow);
    }, { passive: false });
    this.canvas.addEventListener("pointerdown", (event) => {
      this.dragStart = {
        pointerX: event.clientX,
        pointerY: event.clientY,
        offsetX: this.offset.x,
        offsetY: this.offset.y,
      };
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.parentElement.classList.add("dragging");
    });
    this.canvas.addEventListener("pointermove", (event) => {
      const world = this.screenToWorld(event.offsetX, event.offsetY);
      $("#cursorCoordinates").textContent = `X ${formatSigned(world.x)} / Y ${formatSigned(world.y)}`;
      if (!this.dragStart) return;
      this.follow = false;
      this.offset.x = (
        this.dragStart.offsetX + event.clientX - this.dragStart.pointerX
      );
      this.offset.y = (
        this.dragStart.offsetY + event.clientY - this.dragStart.pointerY
      );
      updateFollowButton(this.follow);
    });
    const endDrag = () => {
      this.dragStart = null;
      this.canvas.parentElement.classList.remove("dragging");
    };
    this.canvas.addEventListener("pointerup", endDrag);
    this.canvas.addEventListener("pointercancel", endDrag);
  }

  resize() {
    const ratio = window.devicePixelRatio || 1;
    const bounds = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(bounds.width * ratio));
    const height = Math.max(1, Math.round(bounds.height * ratio));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    this.context.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.width = bounds.width;
    this.height = bounds.height;
  }

  center() {
    this.offset = { x: 0, y: 0 };
    this.follow = true;
    updateFollowButton(this.follow);
  }

  origin() {
    const pose = this.store.state.pose;
    return {
      x: this.width / 2 + this.offset.x - (this.follow ? pose.x * this.scale : 0),
      y: this.height / 2 + this.offset.y + (this.follow ? pose.y * this.scale : 0),
    };
  }

  worldToScreen(x, y, z = 0) {
    const origin = this.origin();
    if (this.mode === "3d") {
      return {
        x: origin.x + (x - y) * this.scale * 0.66,
        y: origin.y + (x + y) * this.scale * 0.32 - z * this.scale,
      };
    }
    return { x: origin.x + x * this.scale, y: origin.y - y * this.scale };
  }

  screenToWorld(x, y) {
    const origin = this.origin();
    if (this.mode === "3d") {
      const sx = (x - origin.x) / (this.scale * 0.66);
      const sy = (y - origin.y) / (this.scale * 0.32);
      return { x: (sx + sy) / 2, y: (sy - sx) / 2 };
    }
    return { x: (x - origin.x) / this.scale, y: -(y - origin.y) / this.scale };
  }

  isExplored(x, y) {
    const size = this.store.state.mapCellSize;
    const index = this.store.state.source === "simulation" ? Math.round : Math.floor;
    return this.store.state.exploredCells.has(`${index(x / size)},${index(y / size)}`);
  }

  draw(timestamp) {
    if (!this.enabled) {
      requestAnimationFrame((time) => this.draw(time));
      return;
    }
    this.frames.push(timestamp);
    while (this.frames.length > 1 && timestamp - this.frames[0] > 1000) this.frames.shift();
    $("#renderRate").textContent = `${Math.max(0, this.frames.length - 1)} FPS`;

    const ctx = this.context;
    ctx.clearRect(0, 0, this.width, this.height);
    ctx.fillStyle = "#070c0e";
    ctx.fillRect(0, 0, this.width, this.height);

    if (this.mode === "3d") this.draw3d(ctx, timestamp);
    else this.draw2d(ctx, timestamp);

    requestAnimationFrame((time) => this.draw(time));
  }

  drawGrid(ctx) {
    const origin = this.origin();
    const spacing = this.scale;
    ctx.strokeStyle = "rgba(121, 162, 148, 0.055)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = ((origin.x % spacing) + spacing) % spacing; x < this.width; x += spacing) {
      ctx.moveTo(x, 0); ctx.lineTo(x, this.height);
    }
    for (let y = ((origin.y % spacing) + spacing) % spacing; y < this.height; y += spacing) {
      ctx.moveTo(0, y); ctx.lineTo(this.width, y);
    }
    ctx.stroke();
  }

  draw2d(ctx, timestamp) {
    this.drawGrid(ctx);
    const size = this.store.state.mapCellSize;
    this.store.state.exploredCells.forEach((key) => {
      const [ix, iy] = key.split(",").map(Number);
      const simulation = this.store.state.source === "simulation";
      const topLeft = this.worldToScreen(
        simulation ? ix * size - size / 2 : ix * size,
        simulation ? iy * size + size / 2 : (iy + 1) * size,
      );
      const alpha = this.store.state.source === "simulation"
        ? 0.16 + ((ix * 17 + iy * 31) & 3) * 0.018
        : 0.11;
      ctx.fillStyle = `rgba(39, 70, 61, ${alpha})`;
      ctx.fillRect(topLeft.x, topLeft.y, size * this.scale + 1, size * this.scale + 1);
    });

    if (this.store.state.source === "simulation") {
      this.drawWalls2d(ctx);
      this.drawRubble2d(ctx);
    } else {
      this.drawOccupied2d(ctx);
      this.drawCloud2d(ctx);
    }
    this.drawPath(ctx);
    this.drawMarkers(ctx, timestamp);
  }

  drawWalls2d(ctx) {
    ctx.lineCap = "round";
    this.walls.forEach(([x1, y1, x2, y2]) => {
      const midX = (x1 + x2) / 2;
      const midY = (y1 + y2) / 2;
      if (!this.isExplored(midX, midY) && !this.isExplored(x1, y1) && !this.isExplored(x2, y2)) return;
      const start = this.worldToScreen(x1, y1);
      const end = this.worldToScreen(x2, y2);
      ctx.strokeStyle = "rgba(135, 177, 163, 0.52)";
      ctx.lineWidth = 4;
      ctx.beginPath(); ctx.moveTo(start.x, start.y); ctx.lineTo(end.x, end.y); ctx.stroke();
      ctx.strokeStyle = "rgba(196, 226, 215, 0.18)";
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(start.x, start.y - 2); ctx.lineTo(end.x, end.y - 2); ctx.stroke();
    });
  }

  drawRubble2d(ctx) {
    this.rubble.forEach((point) => {
      if (!this.isExplored(point.x, point.y)) return;
      const screen = this.worldToScreen(point.x, point.y);
      ctx.fillStyle = `rgba(${90 + point.tone * 45}, ${112 + point.tone * 38}, ${104 + point.tone * 30}, 0.55)`;
      ctx.fillRect(screen.x, screen.y, 1.4 + point.z * 5, 1.4 + point.z * 5);
    });
  }

  drawOccupied2d(ctx) {
    const size = this.store.state.mapCellSize;
    ctx.fillStyle = "rgba(156, 193, 181, 0.72)";
    this.store.state.occupiedCells.forEach((key) => {
      const [ix, iy] = key.split(",").map(Number);
      const topLeft = this.worldToScreen(ix * size, (iy + 1) * size);
      ctx.fillRect(topLeft.x, topLeft.y, size * this.scale + 1, size * this.scale + 1);
    });
  }

  drawCloud2d(ctx) {
    if (this.store.state.occupiedCells.size > 0) return;
    ctx.fillStyle = "rgba(142, 187, 173, 0.66)";
    const pointSize = clamp(this.scale * 0.035, 1, 3);
    const cloud = this.store.state.cloudPoints;
    if (cloud?.positions instanceof Float32Array) {
      // SPC1 packets are stored as flat typed arrays. Sample only enough
      // points for a readable top-down fallback without blocking the UI.
      const stride = Math.max(3, Math.ceil(cloud.positions.length / 24000) * 3);
      for (let index = 0; index < cloud.positions.length; index += stride) {
        const x = cloud.positions[index];
        const y = cloud.positions[index + 1];
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        const screen = this.worldToScreen(x, y);
        ctx.fillRect(screen.x, screen.y, pointSize, pointSize);
      }
      return;
    }
    if (Array.isArray(cloud)) {
      cloud.forEach(([x, y]) => {
        const screen = this.worldToScreen(x, y);
        ctx.fillRect(screen.x, screen.y, pointSize, pointSize);
      });
    }
  }

  draw3d(ctx, timestamp) {
    if (this.store.state.source !== "simulation") {
      this.drawLive3d(ctx, timestamp);
      return;
    }
    const allPoints = [];
    this.walls.forEach(([x1, y1, x2, y2]) => {
      const length = Math.hypot(x2 - x1, y2 - y1);
      const steps = Math.ceil(length * 12);
      for (let index = 0; index <= steps; index += 1) {
        const amount = index / steps;
        const x = lerp(x1, x2, amount);
        const y = lerp(y1, y2, amount);
        if (!this.isExplored(x, y)) continue;
        for (let level = 0; level < 10; level += 1) {
          allPoints.push({ x, y, z: level * 0.17, wall: true });
        }
      }
    });
    this.rubble.forEach((point) => {
      if (this.isExplored(point.x, point.y)) allPoints.push(point);
    });
    allPoints.sort((a, b) => a.x + a.y - (b.x + b.y));
    allPoints.forEach((point) => {
      const screen = this.worldToScreen(point.x, point.y, point.z);
      const alpha = point.wall ? 0.58 : 0.72;
      ctx.fillStyle = point.wall
        ? `rgba(98, ${155 + point.z * 22}, 137, ${alpha})`
        : `rgba(155, 125, 88, ${alpha})`;
      ctx.fillRect(screen.x, screen.y, point.wall ? 1.3 : 2.2, point.wall ? 1.3 : 2.2);
    });
    this.drawPath(ctx);
    this.drawMarkers(ctx, timestamp);
  }

  drawLive3d(ctx, timestamp) {
    const points = this.store.state.cloudPoints;
    if (points.length > 0) {
      const pointSize = clamp(this.scale * 0.032, 1.5, 3.2);
      points.forEach(([x, y, z, red, green, blue]) => {
        const screen = this.worldToScreen(x, y, z);
        if ([red, green, blue].every(Number.isFinite)) {
          ctx.fillStyle = `rgba(${red}, ${green}, ${blue}, 0.88)`;
        } else {
          const heightTone = clamp((z + 1.5) / 4.5, 0, 1);
          ctx.fillStyle = `rgba(${60 + heightTone * 100}, ${135 + heightTone * 80}, ${170 - heightTone * 30}, 0.82)`;
        }
        ctx.fillRect(screen.x, screen.y, pointSize, pointSize);
      });
    } else {
      const size = this.store.state.mapCellSize;
      this.store.state.occupiedCells.forEach((key) => {
        const [ix, iy] = key.split(",").map(Number);
        for (let level = 0; level < 8; level += 1) {
          const screen = this.worldToScreen(
            (ix + 0.5) * size,
            (iy + 0.5) * size,
            level * size,
          );
          ctx.fillStyle = "rgba(105, 174, 153, 0.62)";
          ctx.fillRect(screen.x, screen.y, 1.4, 1.4);
        }
      });
    }
    this.drawPath(ctx);
    this.drawMarkers(ctx, timestamp);
  }

  drawPath(ctx) {
    const path = this.store.state.path;
    if (path.length < 2) return;
    ctx.strokeStyle = "rgba(84, 245, 169, 0.86)";
    ctx.lineWidth = 2.2;
    ctx.shadowColor = "rgba(84, 245, 169, 0.55)";
    ctx.shadowBlur = 8;
    ctx.beginPath();
    path.forEach((point, index) => {
      const screen = this.worldToScreen(point.x, point.y, this.mode === "3d" ? 0.05 : 0);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
    ctx.shadowBlur = 0;
  }

  drawMarkers(ctx, timestamp) {
    this.drawStart(ctx);
    if (this.store.state.target) this.drawTarget(ctx, this.store.state.target, timestamp);
    this.drawSegments(ctx);
    this.drawRobot(ctx, timestamp);
  }

  drawSegments(ctx) {
    const segments = this.store.state.segmentPoses;
    if (!segments || segments.length === 0) return;
    ctx.strokeStyle = "rgba(255, 178, 92, 0.72)";
    ctx.lineWidth = 5;
    ctx.lineCap = "round";
    ctx.beginPath();
    segments.forEach((pose, index) => {
      const screen = this.worldToScreen(pose.x, pose.y, this.mode === "3d" ? pose.z : 0);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
  }

  drawStart(ctx) {
    const start = this.store.state.start;
    const screen = this.worldToScreen(start.x, start.y, this.mode === "3d" ? 0.08 : 0);
    ctx.fillStyle = "#54f5a9";
    ctx.shadowColor = "#54f5a9";
    ctx.shadowBlur = 12;
    ctx.beginPath(); ctx.arc(screen.x, screen.y, 5, 0, Math.PI * 2); ctx.fill();
    ctx.shadowBlur = 0;
    ctx.fillStyle = "rgba(154, 255, 210, 0.8)";
    ctx.font = "8px ui-monospace, monospace";
    ctx.fillText("START", screen.x + 9, screen.y - 8);
  }

  drawRobot(ctx, timestamp) {
    const { pose } = this.store.state;
    const screen = this.worldToScreen(pose.x, pose.y, this.mode === "3d" ? 0.16 : 0);
    const pulse = 13 + Math.sin(timestamp / 240) * 3;
    ctx.strokeStyle = "rgba(255, 139, 66, 0.22)";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(screen.x, screen.y, pulse, 0, Math.PI * 2); ctx.stroke();
    ctx.save();
    ctx.translate(screen.x, screen.y);
    ctx.rotate(-pose.yaw);
    ctx.fillStyle = "#ff8b42";
    ctx.shadowColor = "rgba(255, 139, 66, 0.8)";
    ctx.shadowBlur = 14;
    ctx.beginPath(); ctx.moveTo(11, 0); ctx.lineTo(-7, -6); ctx.lineTo(-4, 0); ctx.lineTo(-7, 6); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.shadowBlur = 0;
  }

  drawTarget(ctx, target, timestamp) {
    if (this.store.state.source === "simulation" && !this.isExplored(target.x, target.y)) return;
    const screen = this.worldToScreen(target.x, target.y, this.mode === "3d" ? 0.2 : 0);
    const size = 10 + Math.sin(timestamp / 180) * 2;
    ctx.strokeStyle = "#ffd166";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(screen.x - size / 2, screen.y - size / 2, size, size);
    ctx.fillStyle = "#ffd166";
    ctx.font = "8px ui-monospace, monospace";
    const mapDistance = Math.hypot(
      target.x - this.store.state.pose.x,
      target.y - this.store.state.pose.y,
      (target.z ?? 0) - (this.store.state.pose.z ?? 0),
    );
    const distance = Number.isFinite(target.distanceMeters)
      ? target.distanceMeters
      : mapDistance;
    ctx.fillText(
      `${target.id}  ${distance.toFixed(2)} m`,
      screen.x + 8,
      screen.y - 8,
    );
  }
}

const vectorSubtract = (a, b) => a.map((value, index) => value - b[index]);
const vectorDot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
const vectorCross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const vectorNormalize = (value) => {
  const length = Math.hypot(...value) || 1;
  return value.map((component) => component / length);
};

function matrixMultiply(left, right) {
  const output = new Float32Array(16);
  for (let column = 0; column < 4; column += 1) {
    for (let row = 0; row < 4; row += 1) {
      let value = 0;
      for (let index = 0; index < 4; index += 1) {
        value += left[index * 4 + row] * right[column * 4 + index];
      }
      output[column * 4 + row] = value;
    }
  }
  return output;
}

function perspectiveMatrix(fieldOfView, aspect, near, far) {
  const scale = 1 / Math.tan(fieldOfView / 2);
  const range = 1 / (near - far);
  return new Float32Array([
    scale / aspect, 0, 0, 0,
    0, scale, 0, 0,
    0, 0, (far + near) * range, -1,
    0, 0, 2 * far * near * range, 0,
  ]);
}

function lookAtMatrix(eye, target, up) {
  const backward = vectorNormalize(vectorSubtract(eye, target));
  const right = vectorNormalize(vectorCross(up, backward));
  const cameraUp = vectorCross(backward, right);
  return new Float32Array([
    right[0], cameraUp[0], backward[0], 0,
    right[1], cameraUp[1], backward[1], 0,
    right[2], cameraUp[2], backward[2], 0,
    -vectorDot(right, eye),
    -vectorDot(cameraUp, eye),
    -vectorDot(backward, eye),
    1,
  ]);
}

class CloudRenderer {
  constructor(canvas, store) {
    this.canvas = canvas;
    this.store = store;
    this.gl = canvas.getContext("webgl", {
      antialias: true,
      alpha: false,
      depth: true,
      powerPreference: "high-performance",
      preserveDrawingBuffer: false,
    });
    this.enabled = true;
    this.follow = true;
    this.yaw = -Math.PI / 4;
    this.pitch = 0.62;
    this.distance = 6;
    this.pixelRatio = 1;
    this.target = { x: 0, y: 0, z: 0.35 };
    this.frames = [];
    this.drag = null;
    this.cloudReference = null;
    this.pathReference = null;
    this.pathLength = -1;
    this.segmentReference = null;
    this.segmentLength = -1;
    this.markerSignature = "";
    this.pendingState = this.store.state;
    this.dataDirty = true;
    this.lastDataUpload = 0;
    this.autoFrame = true;
    this.bounds = null;
    if (!this.gl) {
      this.failed = true;
      return;
    }
    this.canvas.addEventListener("webglcontextlost", (event) => {
      event.preventDefault();
      this.failed = true;
      showToast("3D GPU 컨텍스트 복구 중입니다.");
    });
    this.program = this.createProgram();
    this.positionLocation = this.gl.getAttribLocation(this.program, "a_position");
    this.colorLocation = this.gl.getAttribLocation(this.program, "a_color");
    this.matrixLocation = this.gl.getUniformLocation(this.program, "u_viewProjection");
    this.pointSizeLocation = this.gl.getUniformLocation(this.program, "u_pointSize");
    this.roundPointLocation = this.gl.getUniformLocation(this.program, "u_roundPoint");
    // Double buffering keeps the last complete cloud drawable while the next
    // 480 KB snapshot is uploaded. This avoids exposing an orphaned GPU buffer
    // for a frame on integrated GPUs under ROS/YOLO load.
    this.cloudBuffers = [
      this.createGeometryBuffer(),
      this.createGeometryBuffer(),
    ];
    this.cloudBufferIndex = 0;
    this.cloudBuffer = this.cloudBuffers[this.cloudBufferIndex];
    this.pathBuffer = this.createGeometryBuffer();
    this.markerBuffer = this.createGeometryBuffer();
    this.headingBuffer = this.createGeometryBuffer();
    this.segmentBuffer = this.createGeometryBuffer();
    this.gridBuffer = this.createGridBuffer();
    this.bindEvents();
    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    this.store.addEventListener("update", (event) => {
      this.pendingState = event.detail;
      this.dataDirty = true;
    });
    this.resize();
    requestAnimationFrame((time) => this.draw(time));
  }

  compileShader(type, source) {
    const shader = this.gl.createShader(type);
    this.gl.shaderSource(shader, source);
    this.gl.compileShader(shader);
    if (!this.gl.getShaderParameter(shader, this.gl.COMPILE_STATUS)) {
      throw new Error(this.gl.getShaderInfoLog(shader));
    }
    return shader;
  }

  createProgram() {
    const vertex = this.compileShader(this.gl.VERTEX_SHADER, `
      attribute vec3 a_position;
      attribute vec3 a_color;
      uniform mat4 u_viewProjection;
      uniform float u_pointSize;
      varying vec3 v_color;
      void main() {
        gl_Position = u_viewProjection * vec4(a_position, 1.0);
        gl_PointSize = u_pointSize * clamp(7.0 / gl_Position.w, 0.65, 2.8);
        v_color = a_color;
      }
    `);
    const fragment = this.compileShader(this.gl.FRAGMENT_SHADER, `
      precision mediump float;
      varying vec3 v_color;
      uniform bool u_roundPoint;
      void main() {
        if (u_roundPoint) {
          vec2 delta = gl_PointCoord - vec2(0.5);
          if (dot(delta, delta) > 0.25) discard;
        }
        gl_FragColor = vec4(v_color, 0.92);
      }
    `);
    const program = this.gl.createProgram();
    this.gl.attachShader(program, vertex);
    this.gl.attachShader(program, fragment);
    this.gl.linkProgram(program);
    if (!this.gl.getProgramParameter(program, this.gl.LINK_STATUS)) {
      throw new Error(this.gl.getProgramInfoLog(program));
    }
    return program;
  }

  createGeometryBuffer() {
    return {
      positions: this.gl.createBuffer(),
      colors: this.gl.createBuffer(),
      count: 0,
    };
  }

  uploadGeometry(buffer, positions, colors) {
    buffer.count = positions.length / 3;
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.positions);
    this.gl.bufferData(
      this.gl.ARRAY_BUFFER,
      new Float32Array(positions),
      this.gl.DYNAMIC_DRAW,
    );
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.colors);
    this.gl.bufferData(
      this.gl.ARRAY_BUFFER,
      new Float32Array(colors),
      this.gl.DYNAMIC_DRAW,
    );
  }

  createGridBuffer() {
    const positions = [];
    const colors = [];
    for (let value = -25; value <= 25; value += 1) {
      positions.push(value, -25, -0.03, value, 25, -0.03);
      positions.push(-25, value, -0.03, 25, value, -0.03);
      const major = value % 5 === 0;
      const color = major ? [0.12, 0.31, 0.25] : [0.07, 0.17, 0.15];
      colors.push(...color, ...color, ...color, ...color);
    }
    const buffer = this.createGeometryBuffer();
    this.uploadGeometry(buffer, positions, colors);
    return buffer;
  }

  updateData(state) {
    if (state.cloudPoints !== this.cloudReference) {
      let positions = [];
      let colors = [];
      const minimum = [Infinity, Infinity, Infinity];
      const maximum = [-Infinity, -Infinity, -Infinity];
      if (state.cloudPoints?.positions instanceof Float32Array) {
        positions = state.cloudPoints.positions;
        colors = state.cloudPoints.colors;
        for (let index = 0; index < positions.length; index += 3) {
          const x = positions[index];
          const y = positions[index + 1];
          const z = positions[index + 2];
          minimum[0] = Math.min(minimum[0], x);
          minimum[1] = Math.min(minimum[1], y);
          minimum[2] = Math.min(minimum[2], z);
          maximum[0] = Math.max(maximum[0], x);
          maximum[1] = Math.max(maximum[1], y);
          maximum[2] = Math.max(maximum[2], z);
        }
      } else {
        state.cloudPoints.forEach(([x, y, z, red, green, blue]) => {
          if (![x, y, z].every(Number.isFinite)) return;
          positions.push(x, y, z);
          if ([red, green, blue].every(Number.isFinite)) {
            colors.push(red / 255, green / 255, blue / 255);
          } else {
            const tone = clamp((z + 1.5) / 4.5, 0, 1);
            colors.push(0.24 + tone * 0.35, 0.55 + tone * 0.28, 0.68);
          }
          minimum[0] = Math.min(minimum[0], x);
          minimum[1] = Math.min(minimum[1], y);
          minimum[2] = Math.min(minimum[2], z);
          maximum[0] = Math.max(maximum[0], x);
          maximum[1] = Math.max(maximum[1], y);
          maximum[2] = Math.max(maximum[2], z);
        });
      }
      // Never clear a valid GPU point buffer because a transport/state update
      // briefly contained no cloud. A new gateway session starts a new page
      // cache and the following SPC1 packet replaces this buffer normally.
      if (positions.length === 0) return;
      const nextBufferIndex = (this.cloudBufferIndex + 1) % this.cloudBuffers.length;
      const nextBuffer = this.cloudBuffers[nextBufferIndex];
      this.uploadGeometry(nextBuffer, positions, colors);
      this.cloudReference = state.cloudPoints;
      this.cloudBufferIndex = nextBufferIndex;
      this.cloudBuffer = nextBuffer;
      if (positions.length > 0) {
        this.bounds = { minimum, maximum };
        if (this.autoFrame) {
          this.fitToCloud();
        }
      }
    }

    if (state.path !== this.pathReference || state.path.length !== this.pathLength) {
      this.pathReference = state.path;
      this.pathLength = state.path.length;
      const positions = [];
      const colors = [];
      state.path.forEach(({ x, y, z = 0.04 }) => {
        positions.push(x, y, z + 0.04);
        colors.push(0.33, 0.96, 0.66);
      });
      this.uploadGeometry(this.pathBuffer, positions, colors);
    }

    if (
      state.segmentPoses !== this.segmentReference
      || state.segmentPoses.length !== this.segmentLength
    ) {
      this.segmentReference = state.segmentPoses;
      this.segmentLength = state.segmentPoses.length;
      const positions = [];
      const colors = [];
      state.segmentPoses.forEach(({ x, y, z = 0 }) => {
        positions.push(x, y, z);
        colors.push(1.0, 0.55, 0.26);
      });
      this.uploadGeometry(this.segmentBuffer, positions, colors);
    }

    const markerSignature = [
      state.start.x, state.start.y, state.start.z,
      state.pose.x, state.pose.y, state.pose.z, state.pose.yaw,
      state.target?.x, state.target?.y, state.target?.z,
    ].join("|");
    if (markerSignature !== this.markerSignature) {
      this.markerSignature = markerSignature;
      const markerPositions = [
        state.start.x, state.start.y, state.start.z + 0.08,
        state.pose.x, state.pose.y, state.pose.z + 0.12,
      ];
      const markerColors = [0.33, 0.96, 0.66, 1.0, 0.55, 0.26];
      if (state.target) {
        markerPositions.push(state.target.x, state.target.y, state.target.z ?? 0.2);
        markerColors.push(1.0, 0.82, 0.4);
      }
      this.uploadGeometry(this.markerBuffer, markerPositions, markerColors);
      const headingLength = 0.35;
      this.uploadGeometry(
        this.headingBuffer,
        [
          state.pose.x, state.pose.y, state.pose.z + 0.12,
          state.pose.x + Math.cos(state.pose.yaw) * headingLength,
          state.pose.y + Math.sin(state.pose.yaw) * headingLength,
          state.pose.z + 0.12,
        ],
        [1.0, 0.55, 0.26, 1.0, 0.55, 0.26],
      );
    }
  }

  bindEvents() {
    this.canvas.addEventListener("contextmenu", (event) => event.preventDefault());
    this.canvas.addEventListener("pointerdown", (event) => {
      const pan = event.button === 1 || event.button === 2
        || event.shiftKey || event.ctrlKey || event.metaKey;
      this.drag = { x: event.clientX, y: event.clientY, mode: pan ? "pan" : "orbit" };
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.classList.add("dragging");
    });
    this.canvas.addEventListener("pointermove", (event) => {
      if (!this.drag) return;
      const dx = event.clientX - this.drag.x;
      const dy = event.clientY - this.drag.y;
      this.drag.x = event.clientX;
      this.drag.y = event.clientY;
      if (this.drag.mode === "orbit") {
        this.yaw -= dx * 0.008;
        this.pitch = clamp(
          this.pitch + dy * 0.008,
          -Math.PI / 2 + 0.02,
          Math.PI / 2 - 0.02,
        );
      } else {
        this.follow = false;
        this.autoFrame = false;
        updateFollowButton(false);
        const amount = this.distance * 0.0015;
        const eye = this.eyePosition();
        const target = [this.target.x, this.target.y, this.target.z];
        const forward = vectorNormalize(vectorSubtract(target, eye));
        let right = vectorNormalize(vectorCross(forward, [0, 0, 1]));
        if (Math.abs(vectorDot(forward, [0, 0, 1])) > 0.995) {
          right = [Math.cos(this.yaw + Math.PI / 2), Math.sin(this.yaw + Math.PI / 2), 0];
        }
        const cameraUp = vectorNormalize(vectorCross(right, forward));
        this.target.x += (-dx * right[0] + dy * cameraUp[0]) * amount;
        this.target.y += (-dx * right[1] + dy * cameraUp[1]) * amount;
        this.target.z += (-dx * right[2] + dy * cameraUp[2]) * amount;
      }
    });
    const finishDrag = () => {
      this.drag = null;
      this.canvas.classList.remove("dragging");
    };
    this.canvas.addEventListener("pointerup", finishDrag);
    this.canvas.addEventListener("pointercancel", finishDrag);
    this.canvas.addEventListener("lostpointercapture", finishDrag);
    this.canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.autoFrame = false;
      this.distance = clamp(this.distance * Math.exp(event.deltaY * 0.001), 0.3, 100);
    }, { passive: false });
    this.canvas.addEventListener("dblclick", () => this.centerOnRobot());
  }

  resize() {
    const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
    this.pixelRatio = ratio;
    const bounds = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(bounds.width * ratio));
    const height = Math.max(1, Math.round(bounds.height * ratio));
    if (this.canvas.width === width && this.canvas.height === height) return;
    this.canvas.width = width;
    this.canvas.height = height;
    this.gl?.viewport(0, 0, this.canvas.width, this.canvas.height);
  }

  centerOnRobot() {
    const pose = this.store.state.pose;
    this.target = { x: pose.x, y: pose.y, z: pose.z + 0.25 };
    this.follow = true;
    this.autoFrame = false;
    updateFollowButton(true);
  }

  eyePosition() {
    const cosPitch = Math.cos(this.pitch);
    return [
      this.target.x + this.distance * cosPitch * Math.cos(this.yaw),
      this.target.y + this.distance * cosPitch * Math.sin(this.yaw),
      this.target.z + this.distance * Math.sin(this.pitch),
    ];
  }

  fitToCloud() {
    if (!this.bounds) {
      this.centerOnRobot();
      return;
    }
    const { minimum, maximum } = this.bounds;
    this.target = {
      x: (minimum[0] + maximum[0]) / 2,
      y: (minimum[1] + maximum[1]) / 2,
      z: (minimum[2] + maximum[2]) / 2,
    };
    const extents = maximum.map((value, index) => value - minimum[index]);
    const radius = Math.max(Math.hypot(...extents) / 2, 0.5);
    const aspect = this.canvas.width / Math.max(this.canvas.height, 1);
    const verticalFov = Math.PI / 3;
    const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * aspect);
    const limitingFov = Math.max(0.2, Math.min(verticalFov, horizontalFov));
    this.distance = clamp(
      radius / Math.tan(limitingFov / 2) * 1.3,
      1.5,
      100,
    );
    this.follow = false;
    updateFollowButton(false);
  }

  setPreset(name) {
    if (name === "top") {
      this.yaw = -Math.PI / 2;
      this.pitch = Math.PI / 2 - 0.02;
    } else if (name === "front") {
      this.yaw = -Math.PI / 2;
      this.pitch = 0.08;
    } else if (name === "side") {
      this.yaw = 0;
      this.pitch = 0.08;
    } else if (name === "fit") {
      this.autoFrame = true;
      this.fitToCloud();
    } else if (name === "reset") {
      this.yaw = -Math.PI / 4;
      this.pitch = 0.62;
      this.autoFrame = true;
      this.fitToCloud();
    }
  }

  bindGeometry(buffer) {
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.positions);
    this.gl.vertexAttribPointer(this.positionLocation, 3, this.gl.FLOAT, false, 0, 0);
    this.gl.enableVertexAttribArray(this.positionLocation);
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.colors);
    this.gl.vertexAttribPointer(this.colorLocation, 3, this.gl.FLOAT, false, 0, 0);
    this.gl.enableVertexAttribArray(this.colorLocation);
  }

  renderGeometry(buffer, mode, pointSize) {
    if (!buffer.count) return;
    this.bindGeometry(buffer);
    this.gl.uniform1f(this.pointSizeLocation, pointSize);
    this.gl.uniform1i(this.roundPointLocation, mode === this.gl.POINTS ? 1 : 0);
    this.gl.drawArrays(mode, 0, buffer.count);
  }

  updateTargetLabel(viewProjection) {
    const label = $("#targetMapLabel");
    const target = this.store.state.target;
    if (!target) {
      label.classList.add("hidden");
      return;
    }
    const x = target.x;
    const y = target.y;
    const z = target.z ?? 0.2;
    const clipX = viewProjection[0] * x + viewProjection[4] * y
      + viewProjection[8] * z + viewProjection[12];
    const clipY = viewProjection[1] * x + viewProjection[5] * y
      + viewProjection[9] * z + viewProjection[13];
    const clipW = viewProjection[3] * x + viewProjection[7] * y
      + viewProjection[11] * z + viewProjection[15];
    if (clipW <= 0) {
      label.classList.add("hidden");
      return;
    }
    const normalizedX = clipX / clipW;
    const normalizedY = clipY / clipW;
    if (Math.abs(normalizedX) > 1.05 || Math.abs(normalizedY) > 1.05) {
      label.classList.add("hidden");
      return;
    }
    const bounds = this.canvas.getBoundingClientRect();
    const stageBounds = this.canvas.parentElement.getBoundingClientRect();
    label.style.left = `${bounds.left - stageBounds.left + (normalizedX * 0.5 + 0.5) * bounds.width}px`;
    label.style.top = `${(0.5 - normalizedY * 0.5) * bounds.height}px`;
    const mapDistance = Math.hypot(
      target.x - this.store.state.pose.x,
      target.y - this.store.state.pose.y,
      (target.z ?? 0) - (this.store.state.pose.z ?? 0),
    );
    const distance = Number.isFinite(target.distanceMeters)
      ? target.distanceMeters
      : mapDistance;
    label.textContent = `${target.id}  ${distance.toFixed(2)} m`;
    label.classList.remove("hidden");
  }

  draw(timestamp) {
    if (this.enabled && !this.failed) {
      if (this.dataDirty && timestamp - this.lastDataUpload >= 33) {
        this.updateData(this.pendingState);
        this.dataDirty = false;
        this.lastDataUpload = timestamp;
      }
      if (this.follow) {
        const pose = this.store.state.pose;
        this.target.x = pose.x;
        this.target.y = pose.y;
        this.target.z = pose.z + 0.25;
      }
      this.frames.push(timestamp);
      while (this.frames.length > 1 && timestamp - this.frames[0] > 1000) {
        this.frames.shift();
      }
      $("#renderRate").textContent = `${Math.max(0, this.frames.length - 1)} FPS`;

      const eye = this.eyePosition();
      const target = [this.target.x, this.target.y, this.target.z];
      const span = this.bounds
        ? Math.max(...this.bounds.maximum.map((value, index) => value - this.bounds.minimum[index]))
        : 10;
      const near = Math.max(0.005, this.distance / 1500);
      const far = Math.max(250, this.distance + span * 8);
      const projection = perspectiveMatrix(
        Math.PI / 3,
        this.canvas.width / Math.max(this.canvas.height, 1),
        near,
        far,
      );
      const viewProjection = matrixMultiply(
        projection,
        lookAtMatrix(eye, target, [0, 0, 1]),
      );

      this.gl.clearColor(0.026, 0.043, 0.05, 1);
      this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.DEPTH_BUFFER_BIT);
      this.gl.enable(this.gl.DEPTH_TEST);
      this.gl.enable(this.gl.BLEND);
      this.gl.blendFunc(this.gl.SRC_ALPHA, this.gl.ONE_MINUS_SRC_ALPHA);
      this.gl.useProgram(this.program);
      this.gl.uniformMatrix4fv(this.matrixLocation, false, viewProjection);
      this.renderGeometry(this.gridBuffer, this.gl.LINES, 1);
      this.renderGeometry(this.cloudBuffer, this.gl.POINTS, 4.2 * this.pixelRatio);
      this.renderGeometry(this.pathBuffer, this.gl.LINE_STRIP, 2);
      this.renderGeometry(this.segmentBuffer, this.gl.LINE_STRIP, 4);
      this.renderGeometry(this.headingBuffer, this.gl.LINES, 3);
      this.renderGeometry(this.markerBuffer, this.gl.POINTS, 13 * this.pixelRatio);
      this.updateTargetLabel(viewProjection);

      const yawDegrees = ((-this.yaw * 180 / Math.PI) + 360) % 360;
      const pitchDegrees = this.pitch * 180 / Math.PI;
      $("#cursorCoordinates").textContent = `ORBIT ${yawDegrees.toFixed(0)}° / ${pitchDegrees.toFixed(0)}°`;
    }
    requestAnimationFrame((time) => this.draw(time));
  }
}

const normalizeQuaternion = (quaternion = {}) => {
  const value = {
    x: Number(quaternion.x) || 0,
    y: Number(quaternion.y) || 0,
    z: Number(quaternion.z) || 0,
    w: Number.isFinite(Number(quaternion.w)) ? Number(quaternion.w) : 1,
  };
  const length = Math.hypot(value.x, value.y, value.z, value.w) || 1;
  return {
    x: value.x / length, y: value.y / length,
    z: value.z / length, w: value.w / length,
  };
};

const rotateVectorByQuaternion = (vector, inputQuaternion) => {
  const quaternion = normalizeQuaternion(inputQuaternion);
  const q = [quaternion.x, quaternion.y, quaternion.z];
  const uv = vectorCross(q, vector);
  const uuv = vectorCross(q, uv);
  return vector.map((component, index) => (
    component + 2 * (quaternion.w * uv[index] + uuv[index])
  ));
};

class SnakePoseRenderer {
  constructor(canvas, store) {
    this.canvas = canvas;
    this.store = store;
    this.gl = canvas.getContext("webgl", {
      antialias: true,
      alpha: false,
      depth: true,
      powerPreference: "high-performance",
      preserveDrawingBuffer: false,
    });
    this.enabled = false;
    this.follow = false;
    this.yaw = -0.8;
    this.pitch = 0.5;
    this.distance = 1.55;
    this.target = { x: -0.28, y: 0, z: 0.02 };
    this.pixelRatio = 1;
    this.drag = null;
    this.frames = [];
    this.pendingState = this.store.state;
    this.dataDirty = true;
    this.lastDataUpload = 0;
    if (!this.gl) {
      this.failed = true;
      return;
    }
    this.program = this.createProgram();
    this.positionLocation = this.gl.getAttribLocation(this.program, "a_position");
    this.colorLocation = this.gl.getAttribLocation(this.program, "a_color");
    this.matrixLocation = this.gl.getUniformLocation(this.program, "u_viewProjection");
    this.pointSizeLocation = this.gl.getUniformLocation(this.program, "u_pointSize");
    this.gridBuffer = this.createGridBuffer();
    this.modelBuffer = this.createGeometryBuffer();
    this.axisBuffer = this.createGeometryBuffer();
    this.jointBuffer = this.createGeometryBuffer();
    this.bindEvents();
    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    this.store.addEventListener("update", (event) => {
      this.pendingState = event.detail;
      this.dataDirty = true;
    });
    this.resize();
    requestAnimationFrame((time) => this.draw(time));
  }

  compileShader(type, source) {
    const shader = this.gl.createShader(type);
    this.gl.shaderSource(shader, source);
    this.gl.compileShader(shader);
    if (!this.gl.getShaderParameter(shader, this.gl.COMPILE_STATUS)) {
      throw new Error(this.gl.getShaderInfoLog(shader));
    }
    return shader;
  }

  createProgram() {
    const vertex = this.compileShader(this.gl.VERTEX_SHADER, `
      attribute vec3 a_position;
      attribute vec3 a_color;
      uniform mat4 u_viewProjection;
      uniform float u_pointSize;
      varying vec3 v_color;
      void main() {
        gl_Position = u_viewProjection * vec4(a_position, 1.0);
        gl_PointSize = u_pointSize;
        v_color = a_color;
      }
    `);
    const fragment = this.compileShader(this.gl.FRAGMENT_SHADER, `
      precision highp float;
      varying vec3 v_color;
      void main() { gl_FragColor = vec4(v_color, 0.96); }
    `);
    const program = this.gl.createProgram();
    this.gl.attachShader(program, vertex);
    this.gl.attachShader(program, fragment);
    this.gl.linkProgram(program);
    if (!this.gl.getProgramParameter(program, this.gl.LINK_STATUS)) {
      throw new Error(this.gl.getProgramInfoLog(program));
    }
    return program;
  }

  createGeometryBuffer() {
    return {
      positions: this.gl.createBuffer(),
      colors: this.gl.createBuffer(),
      count: 0,
    };
  }

  uploadGeometry(buffer, positions, colors) {
    buffer.count = positions.length / 3;
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.positions);
    this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(positions), this.gl.DYNAMIC_DRAW);
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.colors);
    this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(colors), this.gl.DYNAMIC_DRAW);
  }

  createGridBuffer() {
    const positions = [];
    const colors = [];
    for (let index = -10; index <= 10; index += 1) {
      const value = index * 0.1;
      positions.push(value, -1, -0.08, value, 1, -0.08);
      positions.push(-1, value, -0.08, 1, value, -0.08);
      const color = index % 5 === 0 ? [0.12, 0.3, 0.24] : [0.055, 0.14, 0.12];
      colors.push(...color, ...color, ...color, ...color);
    }
    const buffer = this.createGeometryBuffer();
    this.uploadGeometry(buffer, positions, colors);
    return buffer;
  }

  updateData(state) {
    const modules = [...(state.snakeModel?.activeModules ?? [])]
      .sort((left, right) => left.moduleIndex - right.moduleIndex);
    const modelPositions = [];
    const modelColors = [];
    const axisPositions = [];
    const axisColors = [];
    const jointPositions = [];
    const jointColors = [];
    let center = [0, 0, 0.02];
    let previousCenter = [0, 0, 0];

    modules.forEach((module, moduleOffset) => {
      const imu = state.imus?.[module.sensorId] ?? {};
      const quaternion = normalizeQuaternion(imu.quaternion);
      const length = clamp(Number(module.visualLength) || 0.28, 0.08, 1);
      if (moduleOffset === 0 && module.translationKnown && module.translation) {
        center = [module.translation.x, module.translation.y, module.translation.z];
      } else if (moduleOffset > 0) {
        const previousModule = modules[moduleOffset - 1];
        const previousImu = state.imus?.[previousModule.sensorId] ?? {};
        const previousLength = clamp(Number(previousModule.visualLength) || 0.28, 0.08, 1);
        center = previousCenter.map((value, index) => (
          value + rotateVectorByQuaternion([-previousLength, 0, 0], previousImu.quaternion)[index]
        ));
      }

      if (moduleOffset === 0) {
        jointPositions.push(0, 0, 0, ...center);
        jointColors.push(0.35, 0.72, 0.82, 0.35, 0.72, 0.82);
      } else {
        jointPositions.push(...previousCenter, ...center);
        jointColors.push(0.95, 0.62, 0.25, 0.95, 0.62, 0.25);
      }

      const half = [length / 2, 0.055, 0.04];
      const corners = [];
      [-1, 1].forEach((sx) => [-1, 1].forEach((sy) => [-1, 1].forEach((sz) => {
        const rotated = rotateVectorByQuaternion(
          [sx * half[0], sy * half[1], sz * half[2]], quaternion,
        );
        corners.push(rotated.map((value, index) => value + center[index]));
      })));
      const edges = [
        [0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3],
        [2, 6], [3, 7], [4, 5], [4, 6], [5, 7], [6, 7],
      ];
      const baseColor = imu.online
        ? (moduleOffset === 0 ? [0.33, 0.96, 0.66] : [0.35, 0.72, 0.82])
        : [0.65, 0.2, 0.23];
      edges.forEach(([start, end]) => {
        modelPositions.push(...corners[start], ...corners[end]);
        modelColors.push(...baseColor, ...baseColor);
      });

      const axisLength = Math.min(0.14, length * 0.55);
      [
        [[axisLength, 0, 0], [1, 0.18, 0.2]],
        [[0, axisLength, 0], [0.2, 1, 0.35]],
        [[0, 0, axisLength], [0.2, 0.5, 1]],
      ].forEach(([axis, color]) => {
        const endpoint = rotateVectorByQuaternion(axis, quaternion)
          .map((value, index) => value + center[index]);
        axisPositions.push(...center, ...endpoint);
        axisColors.push(...color, ...color);
      });
      previousCenter = [...center];
    });

    this.uploadGeometry(this.modelBuffer, modelPositions, modelColors);
    this.uploadGeometry(this.axisBuffer, axisPositions, axisColors);
    this.uploadGeometry(this.jointBuffer, jointPositions, jointColors);
  }

  bindEvents() {
    this.canvas.addEventListener("contextmenu", (event) => event.preventDefault());
    this.canvas.addEventListener("pointerdown", (event) => {
      const pan = event.button === 1 || event.button === 2 || event.shiftKey
        || event.ctrlKey || event.metaKey;
      this.drag = { x: event.clientX, y: event.clientY, mode: pan ? "pan" : "orbit" };
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.classList.add("dragging");
    });
    this.canvas.addEventListener("pointermove", (event) => {
      if (!this.drag) return;
      const dx = event.clientX - this.drag.x;
      const dy = event.clientY - this.drag.y;
      this.drag.x = event.clientX;
      this.drag.y = event.clientY;
      if (this.drag.mode === "orbit") {
        this.yaw -= dx * 0.008;
        this.pitch = clamp(this.pitch + dy * 0.008, -Math.PI / 2 + 0.02, Math.PI / 2 - 0.02);
      } else {
        const eye = this.eyePosition();
        const target = [this.target.x, this.target.y, this.target.z];
        const forward = vectorNormalize(vectorSubtract(target, eye));
        const right = vectorNormalize(vectorCross(forward, [0, 0, 1]));
        const cameraUp = vectorNormalize(vectorCross(right, forward));
        const amount = this.distance * 0.0015;
        this.target.x += (-dx * right[0] + dy * cameraUp[0]) * amount;
        this.target.y += (-dx * right[1] + dy * cameraUp[1]) * amount;
        this.target.z += (-dx * right[2] + dy * cameraUp[2]) * amount;
      }
    });
    const finish = () => { this.drag = null; this.canvas.classList.remove("dragging"); };
    this.canvas.addEventListener("pointerup", finish);
    this.canvas.addEventListener("pointercancel", finish);
    this.canvas.addEventListener("lostpointercapture", finish);
    this.canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.distance = clamp(this.distance * Math.exp(event.deltaY * 0.001), 0.25, 12);
    }, { passive: false });
    this.canvas.addEventListener("dblclick", () => this.setPreset("reset"));
  }

  resize() {
    const bounds = this.canvas.parentElement.getBoundingClientRect();
    this.pixelRatio = Math.min(window.devicePixelRatio || 1, 2.5);
    const width = Math.max(1, Math.round(bounds.width * this.pixelRatio));
    const height = Math.max(1, Math.round(bounds.height * this.pixelRatio));
    if (this.canvas.width === width && this.canvas.height === height) return;
    this.canvas.width = width;
    this.canvas.height = height;
    this.gl?.viewport(0, 0, this.canvas.width, this.canvas.height);
  }

  eyePosition() {
    const cosPitch = Math.cos(this.pitch);
    return [
      this.target.x + this.distance * cosPitch * Math.cos(this.yaw),
      this.target.y + this.distance * cosPitch * Math.sin(this.yaw),
      this.target.z + this.distance * Math.sin(this.pitch),
    ];
  }

  setPreset(name) {
    if (name === "top") {
      this.yaw = -Math.PI / 2; this.pitch = Math.PI / 2 - 0.02;
    } else if (name === "front") {
      this.yaw = -Math.PI / 2; this.pitch = 0.02;
    } else if (name === "side") {
      this.yaw = 0; this.pitch = 0.02;
    } else if (name === "reset") {
      this.yaw = -0.8; this.pitch = 0.5; this.distance = 1.55;
      this.target = { x: -0.28, y: 0, z: 0.02 };
    }
  }

  bindGeometry(buffer) {
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.positions);
    this.gl.vertexAttribPointer(this.positionLocation, 3, this.gl.FLOAT, false, 0, 0);
    this.gl.enableVertexAttribArray(this.positionLocation);
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, buffer.colors);
    this.gl.vertexAttribPointer(this.colorLocation, 3, this.gl.FLOAT, false, 0, 0);
    this.gl.enableVertexAttribArray(this.colorLocation);
  }

  renderGeometry(buffer, mode) {
    if (!buffer.count) return;
    this.bindGeometry(buffer);
    this.gl.uniform1f(this.pointSizeLocation, this.pixelRatio);
    this.gl.drawArrays(mode, 0, buffer.count);
  }

  draw(timestamp) {
    if (this.enabled && !this.failed) {
      if (this.dataDirty && timestamp - this.lastDataUpload >= 33) {
        this.updateData(this.pendingState);
        this.dataDirty = false;
        this.lastDataUpload = timestamp;
      }
      this.frames.push(timestamp);
      while (this.frames.length > 1 && timestamp - this.frames[0] > 1000) this.frames.shift();
      $("#renderRate").textContent = `${Math.max(0, this.frames.length - 1)} FPS`;
      const projection = perspectiveMatrix(
        Math.PI / 3,
        this.canvas.width / Math.max(this.canvas.height, 1),
        0.005,
        50,
      );
      const viewProjection = matrixMultiply(
        projection,
        lookAtMatrix(this.eyePosition(), [this.target.x, this.target.y, this.target.z], [0, 0, 1]),
      );
      this.gl.clearColor(0.026, 0.043, 0.05, 1);
      this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.DEPTH_BUFFER_BIT);
      this.gl.enable(this.gl.DEPTH_TEST);
      this.gl.useProgram(this.program);
      this.gl.uniformMatrix4fv(this.matrixLocation, false, viewProjection);
      this.renderGeometry(this.gridBuffer, this.gl.LINES);
      this.renderGeometry(this.jointBuffer, this.gl.LINES);
      this.renderGeometry(this.modelBuffer, this.gl.LINES);
      this.renderGeometry(this.axisBuffer, this.gl.LINES);
    }
    requestAnimationFrame((time) => this.draw(time));
  }
}

class CameraRenderer {
  constructor(canvas, streamImage) {
    this.canvas = canvas;
    this.streamImage = streamImage;
    this.context = canvas.getContext("2d");
    this.liveImage = null;
    this.lastFrame = null;
    this.streaming = false;
    this.streamUrl = null;
    this.streamRetryTimer = null;
    this.detections = [];
    this.overlayDirty = true;
    this.streamImage.addEventListener("error", () => this.retryStream());
    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    this.resize();
    requestAnimationFrame((time) => this.draw(time));
  }

  setFrame(dataUrl) {
    if (this.streaming) return;
    if (!dataUrl || dataUrl === this.lastFrame) return;
    this.lastFrame = dataUrl;
    const image = new Image();
    image.addEventListener("load", () => { this.liveImage = image; });
    image.src = dataUrl;
  }

  enableStream(url) {
    this.streamUrl = url;
    this.streaming = true;
    this.streamImage.classList.remove("hidden");
    this.connectStream();
  }

  connectStream() {
    if (!this.streamUrl) return;
    clearTimeout(this.streamRetryTimer);
    const separator = this.streamUrl.includes("?") ? "&" : "?";
    this.streamImage.src = `${this.streamUrl}${separator}t=${Date.now()}`;
  }

  retryStream() {
    if (!this.streaming || !this.streamUrl) return;
    clearTimeout(this.streamRetryTimer);
    this.streamRetryTimer = setTimeout(() => this.connectStream(), 1000);
  }

  setDetections(detection) {
    this.detections = detection?.detected ? (detection.candidates ?? []) : [];
    this.overlayDirty = true;
  }

  resize() {
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const bounds = this.canvas.parentElement.getBoundingClientRect();
    const width = Math.max(1, Math.round(bounds.width * ratio));
    const height = Math.max(1, Math.round(bounds.height * ratio));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    this.context.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.width = bounds.width;
    this.height = bounds.height;
    this.overlayDirty = true;
  }

  draw(time) {
    if (this.streaming && !this.overlayDirty) {
      requestAnimationFrame((nextTime) => this.draw(nextTime));
      return;
    }
    const ctx = this.context;
    ctx.clearRect(0, 0, this.width, this.height);
    if (!this.streaming && this.liveImage) {
      const scale = Math.max(
        this.width / this.liveImage.naturalWidth,
        this.height / this.liveImage.naturalHeight,
      );
      const width = this.liveImage.naturalWidth * scale;
      const height = this.liveImage.naturalHeight * scale;
      ctx.drawImage(
        this.liveImage,
        (this.width - width) / 2,
        (this.height - height) / 2,
        width,
        height,
      );
    } else if (!this.streaming) {
      const gradient = ctx.createLinearGradient(0, 0, 0, this.height);
      gradient.addColorStop(0, "#23312c");
      gradient.addColorStop(0.5, "#121b18");
      gradient.addColorStop(1, "#080c0b");
      ctx.fillStyle = gradient;
      ctx.fillRect(0, 0, this.width, this.height);
      const horizon = this.height * 0.48;
      ctx.fillStyle = "rgba(110, 135, 125, 0.13)";
      ctx.beginPath(); ctx.moveTo(0, horizon); ctx.lineTo(this.width, horizon + 7); ctx.lineTo(this.width, this.height); ctx.lineTo(0, this.height); ctx.fill();
      ctx.strokeStyle = "rgba(100, 140, 124, 0.16)";
      for (let index = 0; index < 13; index += 1) {
        const offset = (index * 43 + Math.floor(time / 80) * 29) % this.width;
        ctx.beginPath(); ctx.moveTo(offset, this.height); ctx.lineTo(this.width / 2, horizon); ctx.stroke();
      }
    }
    this.drawDetections(ctx);
    this.overlayDirty = false;
    requestAnimationFrame((nextTime) => this.draw(nextTime));
  }

  drawDetections(ctx) {
    if (this.detections.length === 0) return;
    const sourceWidth = this.streamImage.naturalWidth || 640;
    const sourceHeight = this.streamImage.naturalHeight || 480;
    const scale = Math.max(this.width / sourceWidth, this.height / sourceHeight);
    const renderedWidth = sourceWidth * scale;
    const renderedHeight = sourceHeight * scale;
    const offsetX = (this.width - renderedWidth) / 2;
    const offsetY = (this.height - renderedHeight) / 2;
    this.detections.forEach((candidate) => {
      const [x1, y1, x2, y2] = candidate.bbox ?? [];
      if (![x1, y1, x2, y2].every(Number.isFinite)) return;
      const left = offsetX + x1 * renderedWidth;
      const top = offsetY + y1 * renderedHeight;
      const width = (x2 - x1) * renderedWidth;
      const height = (y2 - y1) * renderedHeight;
      ctx.strokeStyle = "#ffd166";
      ctx.lineWidth = 2;
      ctx.strokeRect(left, top, width, height);
      const label = `PERSON ${Math.round((candidate.confidence ?? 0) * 100)}%`;
      ctx.font = "700 10px ui-monospace, monospace";
      const labelWidth = ctx.measureText(label).width + 10;
      ctx.fillStyle = "rgba(10, 16, 14, 0.88)";
      ctx.fillRect(left, Math.max(0, top - 18), labelWidth, 18);
      ctx.fillStyle = "#ffd166";
      ctx.fillText(label, left + 5, Math.max(12, top - 5));
      (candidate.extremities ?? []).forEach((point) => {
        if (![point.x, point.y].every(Number.isFinite)) return;
        const x = offsetX + point.x * renderedWidth;
        const y = offsetY + point.y * renderedHeight;
        ctx.fillStyle = "#54f5a9";
        ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = "#d9ffeb";
        ctx.font = "8px ui-monospace, monospace";
        ctx.fillText(point.name, x + 6, y - 5);
      });
    });
  }
}

const store = new MissionStore();
const mapRenderer = new MapRenderer($("#mapCanvas"), store);
const cloudRenderer = new CloudRenderer($("#cloudCanvas"), store);
const snakeRenderer = new SnakePoseRenderer($("#snakeCanvas"), store);
const wsUrl = new URLSearchParams(window.location.search).get("ws");
const cameraRenderer = new CameraRenderer($("#cameraCanvas"), $("#cameraStream"));
if (wsUrl) {
  const cameraUrl = new URLSearchParams(window.location.search).get("camera")
    ?? `${window.location.origin}/camera.mjpg`;
  cameraRenderer.enableStream(cameraUrl);
}

const telemetrySource = wsUrl
  ? new WebSocketTelemetrySource(store, wsUrl)
  : new MockTelemetrySource(store);

const rateOrder = ["RGB", "DEPTH", "VO", "SLAM", "IMU50", "IMU51", "IMU52", "EKF"];
$("#rateList").innerHTML = rateOrder.map((name) => `
  <div class="rate-row" data-rate="${name}">
    <span>${name}</span>
    <div class="rate-bar"><i></i></div>
    <strong>-- Hz</strong>
  </div>
`).join("");

function renderDashboard(state) {
  const hours = Math.floor(state.missionSeconds / 3600).toString().padStart(2, "0");
  const minutes = Math.floor((state.missionSeconds % 3600) / 60).toString().padStart(2, "0");
  const seconds = Math.floor(state.missionSeconds % 60).toString().padStart(2, "0");
  $("#missionClock").textContent = `${hours}:${minutes}:${seconds}`;
  $("#distanceTraveled").innerHTML = `${state.distanceTraveled.toFixed(2)} <small>m</small>`;
  $("#distanceTraveled3d").innerHTML = `${(state.distanceTraveled3d ?? 0).toFixed(2)} <small>m</small>`;
  $("#distanceSlamCorrected").innerHTML = `${(state.distanceSlamCorrected ?? 0).toFixed(2)} <small>m</small>`;
  $("#distanceFromStart").innerHTML = `${state.distanceFromStart.toFixed(2)} <small>m</small>`;
  $("#positionX").innerHTML = `${formatSigned(state.pose.x)} <small>m</small>`;
  $("#positionY").innerHTML = `${formatSigned(state.pose.y)} <small>m</small>`;
  const heading = ((state.pose.yaw * 180 / Math.PI) + 360) % 360;
  $("#headingValue").innerHTML = `${Math.round(heading).toString().padStart(3, "0")} <small>°</small>`;
  $("#compass").querySelector("i").style.transform = `rotate(${heading}deg)`;
  $("#distanceMeter").style.width = `${clamp(state.distanceTraveled / 18 * 100, 0, 100)}%`;
  $("#batteryValue").textContent = Number.isFinite(state.battery) ? `${Math.round(state.battery)}%` : "--";
  $("#batteryTrack").style.width = `${Number.isFinite(state.battery) ? state.battery : 0}%`;
  $("#latencyValue").textContent = Number.isFinite(state.latency) ? `${state.latency} ms` : "--";
  $("#mapRateFooter").textContent = `${state.mapRate.toFixed(1)} Hz`;
  $("#mapNodes").textContent = state.mapNodes.toLocaleString();
  $("#exploredArea").textContent = `${(state.exploredCells.size * state.mapCellSize ** 2).toFixed(1)} m²`;
  const cloudPointCount = state.cloudPoints?.count
    ?? state.cloudPointCount
    ?? state.cloudPoints?.length
    ?? 0;
  $("#cloudPointCount").textContent = `${cloudPointCount.toLocaleString()} PTS`;
  const visibleMapItems = state.exploredCells.size + cloudPointCount;
  $("#mapMessage").classList.toggle("hidden", visibleMapItems > 90);
  $("#cameraTimestamp").textContent = new Date().toLocaleTimeString("ko-KR", { hour12: false });
  cameraRenderer.setFrame(state.cameraFrame);
  cameraRenderer.setDetections(state.personDetection);
  $("#cameraFeedTag").textContent = cameraRenderer.streaming
    ? "LOW LATENCY"
    : (state.cameraFrame ? "LIVE FEED" : "DEMO FEED");

  const connection = $("#connectionPill");
  connection.classList.toggle("offline", !state.connected);
  $("#connectionLabel").textContent = state.connected
    ? (state.source === "simulation" ? "SIMULATION LINK" : "ROBOT CONNECTED")
    : "RECONNECTING";

  let healthy = 0;
  rateOrder.forEach((name) => {
    const rate = state.rates[name] ?? { value: 0, expected: 1, ok: false };
    const row = document.querySelector(`[data-rate="${name}"]`);
    const ratio = clamp(rate.value / rate.expected, 0, 1);
    row.querySelector("i").style.width = `${ratio * 100}%`;
    row.querySelector("strong").textContent = `${rate.value.toFixed(1)} Hz`;
    row.classList.toggle("lost", !rate.ok);
    if (rate.ok) healthy += 1;
  });
  $("#healthSummary").textContent = `${healthy} / ${rateOrder.length}`;
  renderImuCards(state.imus ?? {});

  const personDetection = state.personDetection ?? {};
  const hasPerceptionTarget = state.source !== "simulation" && personDetection.detected;
  const hasTarget = hasPerceptionTarget || Boolean(state.target);
  if (state.source !== "simulation" && !personDetection.modelReady) {
    $("#perceptionStatus").textContent = "YOLO 대기 중";
    const error = String(personDetection.error ?? "");
    $("#perceptionHint").textContent = error || "사람 전용 YOLO 모델이 연결되지 않았습니다.";
  } else {
    $("#perceptionStatus").textContent = "탐색 중";
    $("#perceptionHint").innerHTML = "사람이 감지되면<br />이곳에 표시됩니다.";
  }
  $("#targetCount").textContent = hasPerceptionTarget
    ? `${personDetection.count} FOUND`
    : (state.target ? "1 FOUND" : "0 FOUND");
  $("#emptyTarget").classList.toggle("hidden", hasTarget);
  $("#targetCard").classList.toggle("hidden", !hasTarget);
  if (hasPerceptionTarget) {
    const candidate = personDetection.candidates?.[0] ?? {};
    const names = candidate.extremities?.map((item) => item.name) ?? [];
    const parts = names.length > 0 ? names.join(", ") : "사람";
    const distance = Number.isFinite(candidate.distanceMeters)
      ? `${candidate.distanceMeters.toFixed(2)} m 거리 / `
      : "거리 계산 중 / ";
    $("#targetDistance").textContent = `${distance}${parts} / 신뢰도 ${Math.round((candidate.confidence ?? 0) * 100)}%`;
  } else if (state.target) {
    const distance = Math.hypot(state.target.x - state.pose.x, state.target.y - state.pose.y);
    $("#targetDistance").textContent = `${distance.toFixed(1)} m 거리 / 신뢰도 ${Math.round(state.target.confidence * 100)}%`;
  }
  renderEvents(state.events);
}

function renderImuCards(imus) {
  const safeNumber = (value, digits = 1) => (
    Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--"
  );
  $("#imuCardGrid").innerHTML = ["50", "51", "52"].map((sensorId) => {
    const imu = imus[sensorId] ?? {};
    const euler = imu.eulerDeg ?? {};
    const quaternion = imu.quaternion ?? {};
    return `
      <article class="imu-card ${imu.online ? "online" : ""}">
        <header><i></i>${imu.displayId ?? `0x${sensorId}`}<span>${imu.role ?? "UNASSIGNED"} · M${imu.moduleIndex ?? "--"}</span></header>
        <div class="imu-values">
          <div><span>R</span> ${safeNumber(euler.roll)}°</div>
          <div><span>P</span> ${safeNumber(euler.pitch)}°</div>
          <div><span>Y</span> ${safeNumber(euler.yaw)}°</div>
          <div class="imu-quaternion">q [${safeNumber(quaternion.x, 3)}, ${safeNumber(quaternion.y, 3)}, ${safeNumber(quaternion.z, 3)}, ${safeNumber(quaternion.w, 3)}]</div>
        </div>
      </article>
    `;
  }).join("");
}

function renderEvents(events) {
  const escapeHtml = (value) => String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
  $("#eventList").innerHTML = events.map((event) => `
    <li>
      <time>${event.time.toLocaleTimeString("ko-KR", { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false })}</time>
      <span class="${escapeHtml(event.level)}">${escapeHtml(event.message)}</span>
    </li>
  `).join("");
}

function updateFollowButton(following) {
  $("#followButton").classList.toggle("active", following);
}

function showToast(message) {
  const toast = $("#toast");
  toast.textContent = message;
  toast.classList.add("visible");
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => toast.classList.remove("visible"), 2600);
}

let dashboardRenderPending = false;
let lastDashboardRender = 0;
store.addEventListener("update", () => {
  if (dashboardRenderPending) return;
  dashboardRenderPending = true;
  const delay = Math.max(0, 100 - (performance.now() - lastDashboardRender));
  window.setTimeout(() => requestAnimationFrame(() => {
    dashboardRenderPending = false;
    lastDashboardRender = performance.now();
    renderDashboard(store.state);
  }), delay);
});

let activeMapView = "overview";
$$('.view-tab').forEach((button) => {
  button.addEventListener("click", () => {
    $$('.view-tab').forEach((item) => item.classList.remove("active"));
    button.classList.add("active");
    const view = button.dataset.view;
    activeMapView = view;
    const isOverview = view === "overview";
    const is2d = view === "2d";
    const is3d = view === "3d";
    const isSnake = view === "snake";
    mapRenderer.enabled = is2d || isOverview;
    cloudRenderer.enabled = is3d || isOverview;
    snakeRenderer.enabled = isSnake;
    $("#mapCanvas").classList.toggle("hidden-view", !(is2d || isOverview));
    $("#cloudCanvas").classList.toggle("hidden-view", !(is3d || isOverview));
    $("#snakeCanvas").classList.toggle("hidden-view", !isSnake);
    $("#cloudViewActions").classList.toggle("hidden", !(is3d || isOverview));
    $("#snakeViewActions").classList.toggle("hidden", !isSnake);
    $("#orbitHelp").classList.toggle("hidden", is2d);
    $("#imuPosePanel").classList.toggle("hidden", !isSnake);
    $("#mapStage").classList.toggle("snake-mode", isSnake);
    $("#mapStage").classList.toggle("overview-mode", isOverview);
    if (!(is3d || isOverview)) $("#targetMapLabel").classList.add("hidden");
    $("#followButton").disabled = isSnake;
    updateFollowButton(
      isOverview
        ? (cloudRenderer.follow && mapRenderer.follow)
        : (is3d ? cloudRenderer.follow : (is2d && mapRenderer.follow)),
    );
    if (is3d || isOverview) cloudRenderer.resize();
    if (is2d || isOverview) mapRenderer.resize();
    if (isSnake) snakeRenderer.resize();
    if ((is3d && cloudRenderer.failed) || (isSnake && snakeRenderer.failed)) {
      showToast("이 브라우저에서 WebGL을 사용할 수 없습니다.");
    } else {
      const labels = {
        overview: "2D·3D·카메라·사람 탐지를 동시에 표시합니다.",
        "2d": "탑다운 지도로 전환했습니다.",
        "3d": "자유 회전 3D 포인트클라우드로 전환했습니다.",
        snake: "세 IMU 기반 뱀 로봇 자세로 전환했습니다.",
      };
      showToast(labels[view]);
    }
  });
});

$("#followButton").addEventListener("click", () => {
  if (activeMapView === "overview") {
    const follow = !(cloudRenderer.follow && mapRenderer.follow);
    cloudRenderer.follow = follow;
    mapRenderer.follow = follow;
    if (follow) cloudRenderer.centerOnRobot();
    updateFollowButton(follow);
  } else if (cloudRenderer.enabled) {
    cloudRenderer.follow = !cloudRenderer.follow;
    if (cloudRenderer.follow) cloudRenderer.centerOnRobot();
    updateFollowButton(cloudRenderer.follow);
  } else {
    mapRenderer.follow = !mapRenderer.follow;
    updateFollowButton(mapRenderer.follow);
  }
});
$("#centerButton").addEventListener("click", () => {
  if (snakeRenderer.enabled) snakeRenderer.setPreset("reset");
  else if (activeMapView === "overview") {
    mapRenderer.center();
    cloudRenderer.centerOnRobot();
  } else if (cloudRenderer.enabled) cloudRenderer.centerOnRobot();
  else mapRenderer.center();
});
$("#zoomInButton").addEventListener("click", () => {
  if (snakeRenderer.enabled) snakeRenderer.distance = clamp(snakeRenderer.distance / 1.15, 0.25, 12);
  else if (activeMapView === "overview") {
    mapRenderer.scale = clamp(mapRenderer.scale * 1.15, 28, 150);
    cloudRenderer.distance = clamp(cloudRenderer.distance / 1.15, 0.3, 100);
  } else if (cloudRenderer.enabled) cloudRenderer.distance = clamp(cloudRenderer.distance / 1.15, 0.3, 100);
  else mapRenderer.scale = clamp(mapRenderer.scale * 1.15, 28, 150);
});
$("#zoomOutButton").addEventListener("click", () => {
  if (snakeRenderer.enabled) snakeRenderer.distance = clamp(snakeRenderer.distance * 1.15, 0.25, 12);
  else if (activeMapView === "overview") {
    mapRenderer.scale = clamp(mapRenderer.scale / 1.15, 28, 150);
    cloudRenderer.distance = clamp(cloudRenderer.distance * 1.15, 0.3, 100);
  } else if (cloudRenderer.enabled) cloudRenderer.distance = clamp(cloudRenderer.distance * 1.15, 0.3, 100);
  else mapRenderer.scale = clamp(mapRenderer.scale / 1.15, 28, 150);
});
$$('[data-cloud-view]').forEach((button) => {
  button.addEventListener("click", () => cloudRenderer.setPreset(button.dataset.cloudView));
});
$$('[data-snake-view]').forEach((button) => {
  button.addEventListener("click", () => snakeRenderer.setPreset(button.dataset.snakeView));
});
$("#clearEvents").addEventListener("click", () => { store.state.events = []; renderEvents([]); });
$("#estopButton").addEventListener("click", () => showToast("시제품 UI입니다. 실제 정지 명령은 연결되지 않았습니다."));
$("#fullscreenButton").addEventListener("click", async () => {
  if (!document.fullscreenElement) await document.documentElement.requestFullscreen();
  else await document.exitFullscreen();
});

renderDashboard(store.state);
const initialView = new URLSearchParams(window.location.search).get("view");
if (["2d", "3d", "snake"].includes(initialView)) {
  document.querySelector(`[data-view="${initialView}"]`)?.click();
}
telemetrySource.start();
