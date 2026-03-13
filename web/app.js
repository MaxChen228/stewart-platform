import * as THREE from "https://esm.sh/three@0.179.1";
import { OrbitControls } from "https://esm.sh/three@0.179.1/examples/jsm/controls/OrbitControls.js";

const poseSpec = [
  { key: "roll", label: "ROLL", min: -30, max: 30, step: 0.5, unit: "deg" },
  { key: "pitch", label: "PITCH", min: -30, max: 30, step: 0.5, unit: "deg" },
  { key: "yaw", label: "YAW", min: -30, max: 30, step: 0.5, unit: "deg" },
  { key: "x", label: "X", min: -40, max: 40, step: 0.5, unit: "mm" },
  { key: "y", label: "Y", min: -40, max: 40, step: 0.5, unit: "mm" },
  { key: "z", label: "Z OFF", min: -120, max: 0, step: 0.5, unit: "mm" },
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
let actualPlatformLoop;
let needsCameraFit = true;
let calibrationFeedback = null;
const crankLines = [];
const rodLines = [];
const basePoints = [];
const crankPoints = [];
const platformPoints = [];
const actualCrankLines = [];
const actualRodLines = [];
const actualCrankPoints = [];
const actualPlatformPoints = [];

function hardwareAvailable() {
  return Boolean(state?.hardware?.connected && !state?.hardware?.stale && state?.hardware?.ready);
}

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
    input.min = spec.min;
    input.max = spec.max;
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

function getPoseUiSpec(spec) {
  if (spec.key !== "z") {
    return spec;
  }
  return spec;
}

async function onPoseChange(event) {
  const key = event.target.dataset.key;
  const value = Number(event.target.value);
  const pose = { ...state.pose, [key]: value };
  if (key === "z") {
    pose.z = (state.alignment?.calibrationZ || state.geometry.home_z || 0) + value;
  }
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
  const liveSend = false;
  state = await api("/api/mode", {
    method: "POST",
    body: JSON.stringify({ mode, liveSend }),
  });
  render();
}

async function sendCommand(command, extra = {}) {
  const hardwareCommands = new Set([
    "enable_all",
    "disable_all",
    "stop",
    "calibrate",
    "apply_pose",
  ]);
  if (hardwareCommands.has(command) && !hardwareAvailable()) {
    render();
    return;
  }
  state = await api("/api/command", {
    method: "POST",
    body: JSON.stringify({ command, ...extra }),
  });
  if (command === "calibrate") {
    calibrationFeedback = {
      updatedAt: Date.now(),
    };
  }
  render();
}

async function applyPose() {
  if (!hardwareAvailable()) {
    render();
    return;
  }
  state = await api("/api/pose", {
    method: "POST",
    body: JSON.stringify({ pose: state.pose, applyHardware: true }),
  });
  render();
}

function renderPoseControls() {
  poseSpec.forEach((spec) => {
    const uiSpec = getPoseUiSpec(spec);
    const range = document.getElementById(`pose-${spec.key}`);
    const input = document.querySelector(`input[type="number"][data-key="${spec.key}"]`);
    range.min = uiSpec.min;
    range.max = uiSpec.max;
    input.min = uiSpec.min;
    input.max = uiSpec.max;
    if (spec.key === "z") {
      const offset = state.pose.z - (state.alignment?.calibrationZ || state.geometry.home_z || 0);
      range.value = offset;
      input.value = offset.toFixed(2);
    } else {
      range.value = state.pose[spec.key];
      input.value = state.pose[spec.key].toFixed(2);
    }
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
  $("#linkChip").textContent = state.hardware.connected ? "HW ONLINE" : "HW OFFLINE";
  $("#modeChip").className = "chip";
  $("#reachChip").className = `chip ${state.solution.reachable ? "status-ok" : "status-danger"}`;
  $("#linkChip").className = `chip ${
    hardwareAvailable() ? "status-ok" : (state.hardware.connected ? "status-warn" : "status-danger")
  }`;

}

function renderHardwareAccess() {
  const available = hardwareAvailable();
  $("#applyBtn").disabled = !available;
  $("#applyBtn").title = available ? "" : "Hardware offline";
  $("#calibrateBtn").disabled = !available;
  $("#calibrateBtn").title = available ? "" : "Hardware offline";

  document.querySelectorAll("[data-command]").forEach((button) => {
    const command = button.dataset.command;
    const needsHardware = ["enable_all", "disable_all", "stop"].includes(command);
    if (needsHardware) {
      button.disabled = !available;
      button.title = available ? "" : "Hardware offline";
    } else {
      button.disabled = false;
      button.title = "";
    }
  });

}

function renderMotorStatus() {
  const motors = state.hardware.motors || [];
  const online = motors.filter((motor) => motor.on).length;
  $("#motorSummary").innerHTML = [
    `Port: ${state.hardware.port || "--"}`,
    `Ready: ${state.hardware.ready ? "YES" : "NO"}`,
    `Online: ${online}/6`,
  ].join("<br>");

  const grid = $("#motorGrid");
  grid.innerHTML = "";
  motors.slice(0, 6).forEach((motor) => {
    const actual = Number(motor.deg || 0);
    const target = Number(motor.targetDeg || 0);
    const error = actual - target;
    const stateClass = motor.on ? "metric-ok" : "metric-danger";
    const enabledClass = motor.enabled ? "metric-ok" : "metric-warn";
    const errorClass = Math.abs(error) < 1 ? "metric-ok" : (Math.abs(error) < 3 ? "metric-warn" : "metric-danger");
    const card = el("div", "motor-card");
    card.innerHTML = `
      <strong>M${motor.id}</strong>
      <div class="motor-row"><span>State</span><span class="${stateClass}">${motor.on ? "ONLINE" : "OFFLINE"}</span></div>
      <div class="motor-row"><span>Hold</span><span class="${enabledClass}">${motor.enabled ? "ON" : "OFF"}</span></div>
      <div class="motor-row"><span>Actual</span><span>${actual.toFixed(1)} deg</span></div>
      <div class="motor-row"><span>Target</span><span>${target.toFixed(1)} deg</span></div>
      <div class="motor-row"><span>Error</span><span class="${errorClass}">${error.toFixed(1)} deg</span></div>
    `;
    grid.appendChild(card);
  });
}

function toVector3(point) {
  return new THREE.Vector3(point[0], point[1], point[2]);
}

function makeLoop(color, width) {
  const material = new THREE.LineBasicMaterial({ color, linewidth: width, transparent: true });
  const geometry = new THREE.BufferGeometry();
  const loop = new THREE.LineLoop(geometry, material);
  platformGroup.add(loop);
  return loop;
}

function makeLine(color, opacity = 1) {
  const material = new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity });
  const geometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(),
    new THREE.Vector3(),
  ]);
  const line = new THREE.Line(geometry, material);
  platformGroup.add(line);
  return line;
}

function makePoint(color, radius, opacity = 1) {
  const geometry = new THREE.SphereGeometry(radius, 18, 18);
  const material = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.72,
    metalness: 0.0,
    transparent: opacity < 1,
    opacity,
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
  actualPlatformLoop = makeLoop("#c0392b", 4);
  actualPlatformLoop.material.opacity = 0.8;

  for (let i = 0; i < 6; i += 1) {
    crankLines.push(makeLine("#d35400"));
    rodLines.push(makeLine("#2a2520"));
    basePoints.push(makePoint("#2a2520", 5.5));
    crankPoints.push(makePoint("#e6b300", 5.2));
    platformPoints.push(makePoint("#4a6b8c", 5.8));
    actualCrankLines.push(makeLine("#d35400", 0.45));
    actualRodLines.push(makeLine("#c0392b", 0.65));
    actualCrankPoints.push(makePoint("#d35400", 4.4, 0.55));
    actualPlatformPoints.push(makePoint("#c0392b", 5.0, 0.75));
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
  const actualValid = Boolean(
    state.actualSolution?.converged &&
    Number.isFinite(state.actualSolution?.residualNorm) &&
    state.actualSolution.residualNorm < 100.0
  );
  const actualCrankSource = actualValid ? state.actualSolution.crank_points : state.solution.crank_points;
  const actualPlatformSource = actualValid ? state.actualSolution.platform_points_world : state.solution.platform_points_world;
  const actualCrank = actualCrankSource.map(toVector3);
  const actualPlatform = actualPlatformSource.map(toVector3);

  updateLoop(baseLoop, base);
  updateLoop(platformLoop, platform);
  updateLoop(actualPlatformLoop, actualPlatform);
  actualPlatformLoop.visible = actualValid;

  for (let i = 0; i < 6; i += 1) {
    basePoints[i].position.copy(base[i]);
    crankPoints[i].position.copy(crank[i]);
    platformPoints[i].position.copy(platform[i]);
    platformPoints[i].material.color.set(reachable ? "#4a6b8c" : "#c0392b");
    rodLines[i].material.color.set(reachable ? "#2a2520" : "#c0392b");
    setLinePoints(crankLines[i], base[i], crank[i]);
    setLinePoints(rodLines[i], crank[i], platform[i]);
    actualCrankPoints[i].visible = actualValid;
    actualPlatformPoints[i].visible = actualValid;
    actualCrankLines[i].visible = actualValid;
    actualRodLines[i].visible = actualValid;
    if (actualValid) {
      actualCrankPoints[i].position.copy(actualCrank[i]);
      actualPlatformPoints[i].position.copy(actualPlatform[i]);
      setLinePoints(actualCrankLines[i], base[i], actualCrank[i]);
      setLinePoints(actualRodLines[i], actualCrank[i], actualPlatform[i]);
    }
  }

  if (needsCameraFit) {
    fitCameraToPlatform(actualValid ? [...base, ...crank, ...platform, ...actualCrank, ...actualPlatform] : [...base, ...crank, ...platform]);
    needsCameraFit = false;
  }
}

function render() {
  if (!state) return;
  renderPoseControls();
  renderGeometry();
  renderChips();
  renderHardwareAccess();
  renderMotorStatus();
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
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.addEventListener("click", () => sendCommand(button.dataset.command));
  });
  $("#applyBtn").addEventListener("click", applyPose);
  $("#homeBtn").addEventListener("click", async () => {
    await sendCommand("home");
    needsCameraFit = true;
    render();
  });
  $("#calibrateBtn").addEventListener("click", () => sendCommand("calibrate"));
  $("#geometryToggle").addEventListener("click", () => {
    $("#geometryGrid").classList.toggle("hidden");
    $("#geometryToggle").textContent = $("#geometryGrid").classList.contains("hidden") ? "EXPAND" : "COLLAPSE";
  });

  await refresh();
  setInterval(refresh, 150);
});
