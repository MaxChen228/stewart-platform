const poseSpec = [
  { key: "roll", label: "ROLL", min: -30, max: 30, step: 0.5, unit: "deg" },
  { key: "pitch", label: "PITCH", min: -30, max: 30, step: 0.5, unit: "deg" },
  { key: "yaw", label: "YAW", min: -30, max: 30, step: 0.5, unit: "deg" },
  { key: "x", label: "X", min: -40, max: 40, step: 0.5, unit: "mm" },
  { key: "y", label: "Y", min: -40, max: 40, step: 0.5, unit: "mm" },
  { key: "z", label: "Z", min: 80, max: 180, step: 0.5, unit: "mm" },
];

const geometrySpec = [
  ["base_radius", "BASE RADIUS"],
  ["base_angle", "BASE ANGLE"],
  ["platform_radius", "PLATFORM RADIUS"],
  ["platform_angle", "PLATFORM ANGLE"],
  ["lower_leg", "LOWER LEG"],
  ["upper_leg", "UPPER LEG"],
  ["home_z", "HOME Z"],
  ["servo_pulses_per_rev", "PULSES / REV"],
];

let state = null;

function $(selector) {
  return document.querySelector(selector);
}

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  return response.json();
}

function initControls() {
  const container = $("#poseControls");
  poseSpec.forEach((spec) => {
    const row = el("div", "pose-row");
    const label = el("label", "", spec.label);
    label.htmlFor = `pose-${spec.key}`;

    const range = document.createElement("input");
    range.type = "range";
    range.min = spec.min;
    range.max = spec.max;
    range.step = spec.step;
    range.id = `pose-${spec.key}`;
    range.dataset.key = spec.key;
    range.addEventListener("input", onPoseChange);

    const input = document.createElement("input");
    input.type = "number";
    input.step = spec.step;
    input.dataset.key = spec.key;
    input.addEventListener("change", onPoseChange);

    const unit = el("div", "unit-tag", spec.unit);

    row.append(label, range, input, unit);
    container.appendChild(row);
  });

  const geometryGrid = $("#geometryGrid");
  geometrySpec.forEach(([key, labelText]) => {
    const field = el("div", "geometry-field");
    const label = el("label", "", labelText);
    const input = document.createElement("input");
    input.type = "number";
    input.step = "0.01";
    input.dataset.geometry = key;
    input.addEventListener("change", onGeometryChange);
    field.append(label, input);
    geometryGrid.appendChild(field);
  });
}

async function onPoseChange(event) {
  const key = event.target.dataset.key;
  const value = Number(event.target.value);
  const pose = { ...state.pose, [key]: value };
  state = await api("/api/pose", {
    method: "POST",
    body: JSON.stringify({ pose }),
  });
  render();
}

async function onGeometryChange() {
  const payload = {};
  document.querySelectorAll("[data-geometry]").forEach((input) => {
    payload[input.dataset.geometry] = Number(input.value);
  });
  state = await api("/api/geometry", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  render();
}

async function setMode(mode) {
  const liveSend = $("#liveSendToggle").checked;
  state = await api("/api/mode", {
    method: "POST",
    body: JSON.stringify({ mode, liveSend }),
  });
  render();
}

async function sendCommand(command) {
  state = await api("/api/command", {
    method: "POST",
    body: JSON.stringify({ command }),
  });
  render();
}

async function applyPose() {
  state = await api("/api/pose", {
    method: "POST",
    body: JSON.stringify({ pose: state.pose, applyHardware: true }),
  });
  render();
}

function renderPoseControls() {
  poseSpec.forEach((spec) => {
    const range = document.getElementById(`pose-${spec.key}`);
    const input = document.querySelector(`input[type="number"][data-key="${spec.key}"]`);
    range.value = state.pose[spec.key];
    input.value = state.pose[spec.key].toFixed(2);
  });
}

function renderGeometry() {
  geometrySpec.forEach(([key]) => {
    const input = document.querySelector(`[data-geometry="${key}"]`);
    input.value = state.geometry[key];
  });
}

function renderChips() {
  $("#modeChip").textContent = state.mode;
  $("#reachChip").textContent = state.solution.reachable ? "REACHABLE" : "UNREACHABLE";
  $("#reachChip").style.background = state.solution.reachable ? "rgba(74,107,140,0.12)" : "rgba(192,57,43,0.12)";
  $("#reachChip").style.borderColor = state.solution.reachable ? "transparent" : "transparent";
  $("#linkChip").textContent = state.hardware.connected ? "HW ONLINE" : "HW OFFLINE";
  $("#linkChip").style.background = state.hardware.connected ? "rgba(74,107,140,0.12)" : "rgba(192,57,43,0.12)";

  document.querySelectorAll(".chip-button").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.mode === state.mode);
  });
  $("#liveSendToggle").checked = state.liveSend;
}

function renderSession() {
  const online = state.hardware.motors.filter((motor) => motor.on).length;
  $("#sessionMeta").innerHTML = [
    `Port: ${state.hardware.port || "--"}`,
    `Ready: ${state.hardware.ready ? "YES" : "NO"}`,
    `Motors Online: ${online}/6`,
    `Telemetry: ${state.hardware.stale ? "STALE" : "LIVE"}`,
  ].join("<br>");
}

function renderSolve() {
  $("#solveStatus").textContent = state.solution.reachable ? "REACHABLE" : "UNREACHABLE";
  $("#solveStatus").style.color = state.solution.reachable ? "var(--contra)" : "var(--dev)";
  $("#solveIssues").textContent = state.solution.issues.length
    ? state.solution.issues.join(" | ")
    : "No constraint issues.";

  const matrix = $("#solveMatrix");
  matrix.innerHTML = "";
  state.solution.servo_angles_deg.forEach((angle, index) => {
    const card = el("div", "matrix-card");
    card.appendChild(el("strong", "", `M${index + 1}`));
    card.appendChild(makeMetric("Target", `${angle.toFixed(2)} deg`));
    card.appendChild(makeMetric("Pulse", `${state.solution.motor_pulses[index]}`));
    card.appendChild(makeMetric("Crank", `${state.solution.motor_angles_deg[index].toFixed(2)} deg`));
    matrix.appendChild(card);
  });
}

function renderHardware() {
  const matrix = $("#hardwareMatrix");
  matrix.innerHTML = "";
  state.hardware.motors.forEach((motor, index) => {
    const target = state.solution.servo_angles_deg[index] || 0;
    const actual = motor.deg || 0;
    const error = actual - target;
    const card = el("div", "matrix-card");
    card.appendChild(el("strong", "", `M${motor.id}`));
    card.appendChild(makeMetric("State", motor.on ? "ONLINE" : "OFFLINE"));
    card.appendChild(makeMetric("Actual", `${actual.toFixed(2)} deg`));
    card.appendChild(makeMetric("Target", `${target.toFixed(2)} deg`));
    card.appendChild(makeMetric("Error", `${error.toFixed(2)} deg`));
    matrix.appendChild(card);
  });
}

function renderSequence() {
  const list = $("#sequenceList");
  list.innerHTML = "";
  if (!state.sequence.length) {
    list.textContent = "No keyframes recorded.";
    return;
  }
  state.sequence.forEach((frame, index) => {
    const item = el("div", "sequence-item");
    item.textContent = `${String(index + 1).padStart(2, "0")} | r:${frame.roll.toFixed(1)} p:${frame.pitch.toFixed(1)} y:${frame.yaw.toFixed(1)} x:${frame.x.toFixed(1)} y:${frame.y.toFixed(1)} z:${frame.z.toFixed(1)}`;
    list.appendChild(item);
  });
}

function renderOverlays() {
  $("#poseOverlay").innerHTML = `
    <div class="overlay-title">POSE</div>
    <div class="overlay-grid">
      <div>ROLL</div><div>${state.pose.roll.toFixed(2)}</div>
      <div>PITCH</div><div>${state.pose.pitch.toFixed(2)}</div>
      <div>YAW</div><div>${state.pose.yaw.toFixed(2)}</div>
      <div>X</div><div>${state.pose.x.toFixed(2)}</div>
      <div>Y</div><div>${state.pose.y.toFixed(2)}</div>
      <div>Z</div><div>${state.pose.z.toFixed(2)}</div>
    </div>
  `;

  $("#summaryOverlay").innerHTML = `
    <div class="overlay-title">SESSION</div>
    <div class="overlay-grid">
      <div>MODE</div><div>${state.mode}</div>
      <div>LINK</div><div>${state.hardware.connected ? "ONLINE" : "OFFLINE"}</div>
      <div>SOLVE</div><div>${state.solution.reachable ? "OK" : "LIMIT"}</div>
      <div>LIVE</div><div>${state.liveSend ? "ON" : "OFF"}</div>
    </div>
  `;

  const errorLines = state.hardware.motors.map((motor, index) => {
    const target = state.solution.servo_angles_deg[index] || 0;
    const actual = motor.deg || 0;
    return `<div>M${motor.id}: ${(actual - target).toFixed(2)} deg</div>`;
  }).join("");
  $("#errorOverlay").innerHTML = `<div class="overlay-title">TARGET VS ACTUAL</div>${errorLines}`;
}

function makeMetric(label, value) {
  const row = el("div", "matrix-value");
  row.appendChild(el("span", "metric-label", label));
  row.appendChild(el("span", "", value));
  return row;
}

function project(point, width, height, rotation) {
  const [x, y, z] = rotatePoint(point, rotation);
  const scale = Math.min(width, height) * 0.85 / 420;
  const perspective = 1 + z / 900;
  return {
    x: width / 2 + x * scale / perspective,
    y: height / 2 - y * scale / perspective,
    z,
  };
}

function rotatePoint(point, rotation) {
  const [x, y, z] = point;
  const ay = rotation.y;
  const ax = rotation.x;

  const x1 = x * Math.cos(ay) + z * Math.sin(ay);
  const z1 = -x * Math.sin(ay) + z * Math.cos(ay);
  const y2 = y * Math.cos(ax) - z1 * Math.sin(ax);
  const z2 = y * Math.sin(ax) + z1 * Math.cos(ax);

  return [x1, y2, z2];
}

function renderCanvas() {
  const canvas = $("#platformCanvas");
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * devicePixelRatio;
  canvas.height = rect.height * devicePixelRatio;
  const ctx = canvas.getContext("2d");
  ctx.scale(devicePixelRatio, devicePixelRatio);

  ctx.fillStyle = "#f8f7f4";
  ctx.fillRect(0, 0, rect.width, rect.height);

  drawGrid(ctx, rect.width, rect.height);

  const rotation = { x: -0.55, y: 0.7 };
  const base = state.solution.base_points.map((point) => project(point, rect.width, rect.height, rotation));
  const platform = state.solution.platform_points_world.map((point) => project(point, rect.width, rect.height, rotation));
  const crank = state.solution.crank_points.map((point) => project(point, rect.width, rect.height, rotation));

  drawPolygon(ctx, base, "#2a252066", 2.5);
  drawPolygon(ctx, platform, state.solution.reachable ? "#4a6b8c" : "#c0392b", 5);

  base.forEach((point, index) => {
    const crankPoint = crank[index];
    const platformPoint = platform[index];
    drawLine(ctx, point, crankPoint, "#d35400", 3.5);
    drawLine(ctx, crankPoint, platformPoint, state.solution.reachable ? "#2a2520" : "#c0392b", 4);
    drawPoint(ctx, point, "#2a2520");
    drawPoint(ctx, crankPoint, "#e6b300");
    drawPoint(ctx, platformPoint, state.solution.reachable ? "#4a6b8c" : "#c0392b");
    drawLabel(ctx, `M${index + 1}`, platformPoint);
  });
}

function drawGrid(ctx, width, height) {
  ctx.save();
  ctx.strokeStyle = "#e8e4db";
  ctx.lineWidth = 1.5;
  for (let x = 0; x <= width; x += 48) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += 48) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawPolygon(ctx, points, strokeStyle, width) {
  ctx.save();
  ctx.strokeStyle = strokeStyle;
  ctx.lineWidth = width;
  ctx.beginPath();
  points.forEach((point, index) => {
    if (!index) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.closePath();
  ctx.stroke();
  ctx.restore();
}

function drawLine(ctx, a, b, strokeStyle, width) {
  ctx.save();
  ctx.strokeStyle = strokeStyle;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
  ctx.restore();
}

function drawPoint(ctx, point, fillStyle) {
  ctx.save();
  ctx.fillStyle = fillStyle;
  ctx.strokeStyle = "#2a2520";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(point.x, point.y, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawLabel(ctx, label, point) {
  ctx.save();
  ctx.font = "700 14px 'Noto Sans TC'";
  ctx.lineWidth = 6;
  ctx.strokeStyle = "#fcfbfa";
  ctx.strokeText(label, point.x + 8, point.y - 8);
  ctx.fillStyle = "#2a2520";
  ctx.fillText(label, point.x + 8, point.y - 8);
  ctx.restore();
}

function render() {
  if (!state) return;
  renderPoseControls();
  renderGeometry();
  renderChips();
  renderSession();
  renderSolve();
  renderHardware();
  renderSequence();
  renderOverlays();
  renderCanvas();
}

async function refresh() {
  state = await api("/api/state");
  render();
}

window.addEventListener("resize", renderCanvas);

document.addEventListener("DOMContentLoaded", async () => {
  initControls();
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => setMode(button.dataset.mode));
  });
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.addEventListener("click", () => sendCommand(button.dataset.command));
  });
  $("#applyBtn").addEventListener("click", applyPose);
  $("#homeBtn").addEventListener("click", async () => {
    await sendCommand("home");
    render();
  });
  $("#liveSendToggle").addEventListener("change", () => setMode(state.mode));
  $("#geometryToggle").addEventListener("click", () => {
    $("#geometryGrid").classList.toggle("hidden");
    $("#geometryToggle").textContent = $("#geometryGrid").classList.contains("hidden") ? "EXPAND" : "COLLAPSE";
  });

  await refresh();
  setInterval(refresh, 500);
});
