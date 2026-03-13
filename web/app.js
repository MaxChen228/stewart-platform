import * as THREE from "https://esm.sh/three@0.179.1";
import { OrbitControls } from "https://esm.sh/three@0.179.1/examples/jsm/controls/OrbitControls.js";

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
let scene;
let camera;
let renderer;
let controls;
let platformGroup;
let baseLoop;
let platformLoop;
let needsCameraFit = true;
const crankLines = [];
const rodLines = [];
const basePoints = [];
const crankPoints = [];
const platformPoints = [];

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
  needsCameraFit = true;
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

async function sendCommand(command, extra = {}) {
  state = await api("/api/command", {
    method: "POST",
    body: JSON.stringify({ command, ...extra }),
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
    card.appendChild(makeMetric("Raw", `${(motor.rawDeg || 0).toFixed(2)} deg`));
    card.appendChild(makeMetric("Target", `${target.toFixed(2)} deg`));
    card.appendChild(makeMetric("Error", `${error.toFixed(2)} deg`));
    card.appendChild(makeMetric("Hold", motor.enabled ? "ENABLED" : "DISABLED"));
    const zeroButton = el("button", "mini-action", "Zero");
    zeroButton.addEventListener("click", () => sendCommand("zero_motor", { motorId: motor.id }));
    card.appendChild(zeroButton);
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

function toVector3(point) {
  return new THREE.Vector3(point[0], point[1], point[2]);
}

function makeLoop(color, width) {
  const material = new THREE.LineBasicMaterial({ color, linewidth: width });
  const geometry = new THREE.BufferGeometry();
  const loop = new THREE.LineLoop(geometry, material);
  platformGroup.add(loop);
  return loop;
}

function makeLine(color) {
  const material = new THREE.LineBasicMaterial({ color });
  const geometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(),
    new THREE.Vector3(),
  ]);
  const line = new THREE.Line(geometry, material);
  platformGroup.add(line);
  return line;
}

function makePoint(color, radius) {
  const geometry = new THREE.SphereGeometry(radius, 18, 18);
  const material = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.72,
    metalness: 0.0,
  });
  const mesh = new THREE.Mesh(geometry, material);
  platformGroup.add(mesh);
  return mesh;
}

function initScene() {
  const viewport = $("#sceneViewport");
  scene = new THREE.Scene();
  scene.background = new THREE.Color("#f8f7f4");
  scene.fog = new THREE.Fog("#f8f7f4", 500, 1200);

  camera = new THREE.PerspectiveCamera(40, 1, 0.1, 3000);
  camera.up.set(0, 0, 1);
  camera.position.set(320, -320, 260);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  viewport.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0, 0, 90);
  controls.minDistance = 180;
  controls.maxDistance = 900;
  controls.screenSpacePanning = true;

  const ambient = new THREE.AmbientLight("#fcfbfa", 1.3);
  scene.add(ambient);

  const keyLight = new THREE.DirectionalLight("#ffffff", 1.0);
  keyLight.position.set(220, 320, 160);
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight("#e6b300", 0.35);
  fillLight.position.set(-180, 140, -140);
  scene.add(fillLight);

  const grid = new THREE.GridHelper(900, 18, 0xdbd6cd, 0xe8e4db);
  grid.rotateX(Math.PI / 2);
  grid.position.z = -0.5;
  scene.add(grid);

  const axes = new THREE.AxesHelper(120);
  axes.material.transparent = true;
  axes.material.opacity = 0.35;
  scene.add(axes);

  platformGroup = new THREE.Group();
  scene.add(platformGroup);

  baseLoop = makeLoop("#2a2520", 2);
  platformLoop = makeLoop("#4a6b8c", 4);

  for (let i = 0; i < 6; i += 1) {
    crankLines.push(makeLine("#d35400"));
    rodLines.push(makeLine("#2a2520"));
    basePoints.push(makePoint("#2a2520", 5.5));
    crankPoints.push(makePoint("#e6b300", 5.2));
    platformPoints.push(makePoint("#4a6b8c", 5.8));
  }

  resizeScene();
  animateScene();
}

function fitCameraToPlatform(points) {
  if (!points.length) return;

  const box = new THREE.Box3().setFromPoints(points);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const radius = Math.max(size.x, size.y, size.z, 180) * 0.8;
  const distance = radius / Math.tan(THREE.MathUtils.degToRad(camera.fov * 0.5));

  controls.target.copy(center);
  camera.position.copy(
    center.clone().add(new THREE.Vector3(distance * 0.95, -distance * 0.95, distance * 0.72))
  );
  camera.near = Math.max(0.1, distance / 200);
  camera.far = distance * 12;
  camera.updateProjectionMatrix();
  controls.update();
}

function setLinePoints(line, start, end) {
  line.geometry.setFromPoints([start, end]);
}

function updateLoop(loop, points) {
  loop.geometry.setFromPoints(points);
}

function renderCanvas() {
  if (!state || !scene) return;

  const reachable = state.solution.reachable;
  platformLoop.material.color.set(reachable ? "#4a6b8c" : "#c0392b");

  const base = state.solution.base_points.map(toVector3);
  const crank = state.solution.crank_points.map(toVector3);
  const platform = state.solution.platform_points_world.map(toVector3);

  updateLoop(baseLoop, base);
  updateLoop(platformLoop, platform);

  for (let i = 0; i < 6; i += 1) {
    basePoints[i].position.copy(base[i]);
    crankPoints[i].position.copy(crank[i]);
    platformPoints[i].position.copy(platform[i]);
    platformPoints[i].material.color.set(reachable ? "#4a6b8c" : "#c0392b");
    rodLines[i].material.color.set(reachable ? "#2a2520" : "#c0392b");
    setLinePoints(crankLines[i], base[i], crank[i]);
    setLinePoints(rodLines[i], crank[i], platform[i]);
  }

  if (needsCameraFit) {
    fitCameraToPlatform([...base, ...crank, ...platform]);
    needsCameraFit = false;
  }
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

function resizeScene() {
  if (!renderer || !camera) return;
  const viewport = $("#sceneViewport");
  const width = viewport.clientWidth;
  const height = viewport.clientHeight;
  camera.aspect = width / Math.max(height, 1);
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, true);
}

function animateScene() {
  requestAnimationFrame(animateScene);
  if (!renderer || !scene || !camera) return;
  controls.update();
  renderer.render(scene, camera);
}

window.addEventListener("resize", () => {
  resizeScene();
  renderCanvas();
});

document.addEventListener("DOMContentLoaded", async () => {
  initControls();
  initScene();
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => setMode(button.dataset.mode));
  });
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.addEventListener("click", () => sendCommand(button.dataset.command));
  });
  $("#applyBtn").addEventListener("click", applyPose);
  $("#homeBtn").addEventListener("click", async () => {
    await sendCommand("home");
    needsCameraFit = true;
    render();
  });
  $("#liveSendToggle").addEventListener("change", () => setMode(state.mode));
  $("#geometryToggle").addEventListener("click", () => {
    $("#geometryGrid").classList.toggle("hidden");
    $("#geometryToggle").textContent = $("#geometryGrid").classList.contains("hidden") ? "EXPAND" : "COLLAPSE";
  });

  await refresh();
  setInterval(refresh, 150);
});
