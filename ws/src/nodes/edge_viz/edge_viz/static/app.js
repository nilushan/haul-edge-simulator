/* Haul-edge 3D physics visualizer */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const G = 9.80665;
const RAD2DEG = 180 / Math.PI;

const els = {
  conn: document.getElementById('conn'),
  clock: document.getElementById('clock'),
  cycle: document.getElementById('cycle'),
  camMode: document.getElementById('camMode'),
  source: document.getElementById('source'),
  mapSelect: document.getElementById('mapSelect'),
  p_speed: document.getElementById('p_speed'),
  p_vel: document.getElementById('p_vel'),
  p_yaw: document.getElementById('p_yaw'),
  p_att: document.getElementById('p_att'),
  p_pos: document.getElementById('p_pos'),
  p_alt: document.getElementById('p_alt'),
  p_sf: document.getElementById('p_sf'),
  p_g: document.getElementById('p_g'),
  p_yr: document.getElementById('p_yr'),
  p_pr: document.getElementById('p_pr'),
  p_cent: document.getElementById('p_cent'),
  p_fix: document.getElementById('p_fix'),
  p_ll: document.getElementById('p_ll'),
  p_lidar: document.getElementById('p_lidar'),
  p_road: document.getElementById('p_road'),
  p_vibe: document.getElementById('p_vibe'),
  alertBadge: document.getElementById('alertBadge'),
  alertFeed: document.getElementById('alertFeed'),
  legend: document.getElementById('legend'),
  classToggles: document.getElementById('classToggles'),
  classCounts: document.getElementById('classCounts'),
  togRaw: document.getElementById('togRaw'),
  togMap: document.getElementById('togMap'),
  togDetectMap: document.getElementById('togDetectMap'),
  togMarkers: document.getElementById('togMarkers'),
  togLabels: document.getElementById('togLabels'),
  togFullScan: document.getElementById('togFullScan'),
};

// ——— Semantic class / event styles (served by /api/classes) ———
const UNKNOWN_RGB = [0.49, 0.53, 0.6];
const styles = {
  classes: [],
  layers: [],
  events: [],
  severities: { info: '#4aa3ff', warn: '#e6b450', critical: '#f07178' },
};
const classByLabel = new Map(); // point label → class style
const classByKey = new Map();
const eventByType = new Map();
const layerVisible = new Map(); // layer key → shown?

function hexToRgb(hex) {
  const m = /^#?([0-9a-f]{6})$/i.exec(String(hex || ''));
  if (!m) return UNKNOWN_RGB.slice();
  const v = parseInt(m[1], 16);
  return [((v >> 16) & 255) / 255, ((v >> 8) & 255) / 255, (v & 255) / 255];
}

function classFor(label) {
  return classByLabel.get(Number(label)) || null;
}

function eventStyleFor(type) {
  return eventByType.get(String(type)) || null;
}

function eventColor(type, severity) {
  const style = eventStyleFor(type);
  if (style) return style.color;
  return styles.severities[severity] || styles.severities.info;
}

/** Classes outside the driving surface are worth keeping in the world map. */
function accumulates(layer) {
  return layer !== 'ground' && layer !== 'unknown';
}

async function loadStyles() {
  try {
    const data = await fetch('/api/classes').then((r) => r.json());
    styles.classes = data.classes || [];
    styles.layers = data.layers || [];
    styles.events = data.events || [];
    styles.severities = data.severities || styles.severities;
  } catch (_) {
    return false; // keep the neutral fallback palette
  }
  classByLabel.clear();
  classByKey.clear();
  for (const c of styles.classes) {
    const entry = { ...c, rgb: hexToRgb(c.color) };
    classByLabel.set(Number(c.label), entry);
    classByKey.set(c.key, entry);
  }
  eventByType.clear();
  for (const e of styles.events) eventByType.set(e.type, e);
  for (const layer of styles.layers) {
    if (!layerVisible.has(layer.key)) layerVisible.set(layer.key, layer.default_on !== false);
  }
  return true;
}

function fmt(n, d = 2) {
  if (n === undefined || n === null || Number.isNaN(n)) return '—';
  return Number(n).toFixed(d);
}

function enuToThree(x, y, z) {
  // ENU (x east, y north, z up) → Three (x, y-up, z)
  return new THREE.Vector3(x, z, -y);
}

// ——— Charts (physics units) ———
const chartDefaults = {
  responsive: true,
  animation: false,
  maintainAspectRatio: false,
  scales: {
    x: {
      type: 'linear',
      ticks: { color: '#8fa0b5', maxTicksLimit: 5, font: { size: 10 } },
      grid: { color: 'rgba(255,255,255,0.04)' },
    },
    y: {
      ticks: { color: '#8fa0b5', font: { size: 10 } },
      grid: { color: 'rgba(255,255,255,0.06)' },
    },
  },
  plugins: {
    legend: {
      labels: { color: '#c9d7e8', boxWidth: 10, font: { size: 10 } },
    },
  },
  elements: { point: { radius: 0 }, line: { borderWidth: 1.4, tension: 0.12 } },
};

function mkChart(id, datasets, yRange) {
  const ChartCtor = window.Chart;
  if (!ChartCtor) {
    console.warn('Chart.js not loaded yet');
    return null;
  }
  const canvas = document.getElementById(id);
  canvas.style.height = `${canvas.getAttribute('height') || 110}px`;
  return new ChartCtor(canvas, {
    type: 'line',
    data: { datasets },
    options: {
      ...chartDefaults,
      scales: {
        ...chartDefaults.scales,
        y: {
          ...chartDefaults.scales.y,
          suggestedMin: yRange?.[0],
          suggestedMax: yRange?.[1],
        },
      },
    },
  });
}

const accelChart = mkChart('chartAccel', [
  { label: 'aₓ', data: [], borderColor: '#4aa3ff' },
  { label: 'aᵧ', data: [], borderColor: '#3ecf8e' },
  { label: 'a_z', data: [], borderColor: '#e6b450' },
], [-4, 14]);

const gyroChart = mkChart('chartGyro', [
  { label: 'ωₓ', data: [], borderColor: '#4aa3ff' },
  { label: 'ωᵧ', data: [], borderColor: '#3ecf8e' },
  { label: 'ω_z', data: [], borderColor: '#c084fc' },
], [-30, 30]);

const speedChart = mkChart('chartSpeed', [
  { label: 'speed', data: [], borderColor: '#4aa3ff' },
  { label: 'load g', data: [], borderColor: '#e6b450', yAxisID: 'y1' },
], [0, 40]);
if (speedChart) {
  speedChart.options.scales.y1 = {
    position: 'right',
    suggestedMin: 0.5,
    suggestedMax: 1.8,
    ticks: { color: '#8fa0b5', font: { size: 10 } },
    grid: { drawOnChartArea: false },
  };
}

function setSeries(chart, series) {
  if (!chart) return;
  series.forEach((pts, i) => {
    chart.data.datasets[i].data = pts;
  });
  chart.update('none');
}

// ——— Three.js world (data-driven only: odom path + LiDAR; no fake scenery) ———
const canvas = document.getElementById('scene3d');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x1a222c, 1);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
// Very light distant fog only — keep most of the map visible
scene.fog = new THREE.Fog(0x1a222c, 2500, 7000);

const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.5, 8000);
camera.position.set(-20, 14, 24);

// Orbit around a moving target (truck). In follow mode we slide camera+target together.
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.maxPolarAngle = Math.PI * 0.48;
controls.minDistance = 5;
controls.maxDistance = 5000;
controls.enablePan = false; // follow default: orbit/zoom only; pan in free mode
controls.panSpeed = 1.0;
controls.zoomSpeed = 1.2;
controls.rotateSpeed = 0.85;

const hemi = new THREE.HemisphereLight(0xc8d6e5, 0x3a3228, 0.7);
scene.add(hemi);
const sun = new THREE.DirectionalLight(0xfff1dd, 1.0);
sun.position.set(40, 70, 30);
sun.castShadow = true;
sun.shadow.mapSize.set(1024, 1024);
sun.shadow.camera.left = -50;
sun.shadow.camera.right = 50;
sun.shadow.camera.top = 50;
sun.shadow.camera.bottom = -50;
scene.add(sun);
scene.add(new THREE.AmbientLight(0x4a5560, 0.3));

// Neutral reference plane under the truck (not sim geometry — LiDAR is the surface).
// Follows vehicle XY so it stays under the ego; height tracks odom.z.
// Subtle local pad under the truck only (map cloud is the real surface)
const ground = new THREE.Mesh(
  new THREE.CircleGeometry(40, 48),
  new THREE.MeshStandardMaterial({
    color: 0x2a3340,
    roughness: 1,
    metalness: 0,
    transparent: true,
    opacity: 0.35,
  }),
);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.02;
ground.receiveShadow = true;
scene.add(ground);

const grid = new THREE.GridHelper(80, 16, 0x3d4a58, 0x2a3340);
grid.position.y = 0.01;
grid.material.opacity = 0.25;
grid.material.transparent = true;
scene.add(grid);

// Truck: group +X = forward, +Y = up, +Z = right (Three). Matches body +X forward after yaw.
function buildTruck() {
  const g = new THREE.Group();
  const yellow = new THREE.MeshStandardMaterial({ color: 0xd4a017, metalness: 0.35, roughness: 0.45 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x2a2e33, metalness: 0.4, roughness: 0.55 });
  const glass = new THREE.MeshStandardMaterial({
    color: 0x8ec8e8, metalness: 0.2, roughness: 0.15, transparent: true, opacity: 0.55,
  });
  const rubber = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.95 });

  // Cab at +X (front)
  const cab = new THREE.Mesh(new THREE.BoxGeometry(2.4, 2.0, 3.2), yellow);
  cab.position.set(2.8, 2.0, 0);
  cab.castShadow = true;
  g.add(cab);

  const wind = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.0, 2.6), glass);
  wind.position.set(4.05, 2.35, 0);
  g.add(wind);

  // Dump bed behind cab (−X)
  const bed = new THREE.Mesh(new THREE.BoxGeometry(5.8, 1.7, 3.4), yellow);
  bed.position.set(-1.0, 1.55, 0);
  bed.castShadow = true;
  g.add(bed);

  const rim = new THREE.Mesh(new THREE.BoxGeometry(5.4, 0.22, 3.5), yellow);
  rim.position.set(-1.0, 2.5, 0);
  g.add(rim);

  // Wheels: cylinder default axis = Y; rotate to axle along +Z (left–right)
  const wheelGeo = new THREE.CylinderGeometry(0.72, 0.72, 0.5, 18);
  const places = [
    [3.0, 0.72, 1.55], [3.0, 0.72, -1.55],   // front
    [0.2, 0.72, 1.55], [0.2, 0.72, -1.55],
    [-2.4, 0.72, 1.55], [-2.4, 0.72, -1.55], // rear
  ];
  for (const [x, y, z] of places) {
    const w = new THREE.Mesh(wheelGeo, rubber);
    w.rotation.x = Math.PI / 2; // axis → Z
    w.position.set(x, y, z);
    w.castShadow = true;
    g.add(w);
  }

  // LiDAR pod on cab roof (front)
  const pod = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.3, 0.5), dark);
  pod.position.set(2.8, 3.15, 0);
  g.add(pod);
  const dome = new THREE.Mesh(
    new THREE.SphereGeometry(0.2, 12, 10),
    new THREE.MeshStandardMaterial({ color: 0x222831, metalness: 0.6, roughness: 0.3 }),
  );
  dome.position.set(2.8, 3.4, 0);
  g.add(dome);

  return g;
}

const truck = buildTruck();
scene.add(truck);

// Path trail (odom)
const trailMax = 1200;
const trailPositions = new Float32Array(trailMax * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
trailGeo.setDrawRange(0, 0);
const trail = new THREE.Line(
  trailGeo,
  new THREE.LineBasicMaterial({ color: 0x4aa3ff, transparent: true, opacity: 0.9 }),
);
scene.add(trail);
let trailCount = 0;

// ——— Accumulated LiDAR map (world frame, pose-aligned frames) ———
// Keep as much map as the browser can reasonably draw
const MAP_MAX = 400000;
const VOXEL = 0.22; // denser accumulated map
const MAP_KEEP_RADIUS_M = 2500; // only prune beyond this from ego when over cap
const mapVoxels = new Map(); // key -> {x,y,z,r,g,b} in Three space
const mapPositions = new Float32Array(MAP_MAX * 3);
const mapColors = new Float32Array(MAP_MAX * 3);
const mapGeo = new THREE.BufferGeometry();
mapGeo.setAttribute('position', new THREE.BufferAttribute(mapPositions, 3));
mapGeo.setAttribute('color', new THREE.BufferAttribute(mapColors, 3));
mapGeo.setDrawRange(0, 0);
const mapCloud = new THREE.Points(
  mapGeo,
  new THREE.PointsMaterial({
    size: 0.16,
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    // Kept dim so the classified live scan reads on top of it.
    opacity: 0.45,
    depthWrite: false,
  }),
);
// Avoid frustum culling hiding chunks when bounding sphere is stale/huge
mapCloud.frustumCulled = false;
mapCloud.renderOrder = 1;
scene.add(mapCloud);

// Current-scan highlight (body-frame points this tick)
const scanMax = 20000;
const scanPositions = new Float32Array(scanMax * 3);
const scanColors = new Float32Array(scanMax * 3);
const scanGeo = new THREE.BufferGeometry();
scanGeo.setAttribute('position', new THREE.BufferAttribute(scanPositions, 3));
scanGeo.setAttribute('color', new THREE.BufferAttribute(scanColors, 3));
scanGeo.setDrawRange(0, 0);
const scanCloud = new THREE.Points(
  scanGeo,
  new THREE.PointsMaterial({
    size: 0.18,
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.95,
    depthWrite: true,
  }),
);
scanCloud.frustumCulled = false;
scanCloud.renderOrder = 2; // draw current frame on top
scene.add(scanCloud);

/** Colored detection layers (body → world each tick). */
function makeDetectCloud(maxN, size, opacity) {
  const positions = new Float32Array(maxN * 3);
  const colors = new Float32Array(maxN * 3);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geo.setDrawRange(0, 0);
  const pts = new THREE.Points(
    geo,
    new THREE.PointsMaterial({
      size,
      vertexColors: true,
      sizeAttenuation: true,
      transparent: true,
      opacity,
      depthWrite: false,
    }),
  );
  pts.frustumCulled = false;
  pts.renderOrder = 3;
  scene.add(pts);
  return { maxN, positions, colors, geo, pts };
}

// Bus-mode detector clouds land in these layers; hub mode labels the raw scan
// in place and leaves them empty.
const detectLayers = {
  ground: makeDetectCloud(8000, 0.14, 0.45),
  obstacle: makeDetectCloud(8000, 0.28, 0.9),
  rock: makeDetectCloud(6000, 0.32, 0.95),
  bund: makeDetectCloud(10000, 0.22, 0.9),
};

/** Bus cloud topic name → class key whose colour and toggle it follows. */
const CLOUD_CLASS = {
  ground: 'ground',
  obstacles: 'obstacle',
  rocks: 'rock',
  bunds: 'bund',
};

function layerRgb(classKey) {
  const entry = classByKey.get(classKey);
  return entry ? entry.rgb : UNKNOWN_RGB;
}

// Persistent world obstacle voxels (client-side HAG + detector clouds)
const OBS_MAP_MAX = 80000;
const OBS_VOXEL = 0.35;
const obsVoxels = new Map();
const obsMapPositions = new Float32Array(OBS_MAP_MAX * 3);
const obsMapColors = new Float32Array(OBS_MAP_MAX * 3);
const obsMapGeo = new THREE.BufferGeometry();
obsMapGeo.setAttribute('position', new THREE.BufferAttribute(obsMapPositions, 3));
obsMapGeo.setAttribute('color', new THREE.BufferAttribute(obsMapColors, 3));
obsMapGeo.setDrawRange(0, 0);
const obsMapCloud = new THREE.Points(
  obsMapGeo,
  new THREE.PointsMaterial({
    size: 0.26,
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.9,
    depthWrite: false,
  }),
);
obsMapCloud.frustumCulled = false;
obsMapCloud.renderOrder = 3;
scene.add(obsMapCloud);

const alertMarkers = new THREE.Group();
scene.add(alertMarkers);
const alertMarkerById = new Map();
const ALERT_MARKER_TTL_MS = 120000;

let lastLidarT = -1;
let mapCount = 0;
let mapGen = 0; // increments each integrated scan (for age tint)

/** Historical map: muted slate → dusty amber by height (not competing with live scan). */
function mapHeightColor(z, age01, out, i) {
  const h = Math.max(0, Math.min(1, (z + 0.1) / 1.6));
  // base: cool gray-blue ground → muted brown high
  const r0 = 0.28 + 0.22 * h;
  const g0 = 0.34 + 0.10 * h;
  const b0 = 0.42 - 0.12 * h;
  // newer map points slightly brighter; older dimmer
  const a = 0.55 + 0.45 * (1 - age01);
  out[i] = r0 * a;
  out[i + 1] = g0 * a;
  out[i + 2] = b0 * a + 0.05;
}

/** Current scan only: high-contrast cyan → lime by height. */
function scanHeightColor(z, out, i) {
  const h = Math.max(0, Math.min(1, (z + 0.1) / 1.6));
  // cyan ground → yellow-green obstacles
  out[i] = 0.15 + 0.55 * h;
  out[i + 1] = 0.95 - 0.15 * h;
  out[i + 2] = 0.95 - 0.75 * h;
}

/** Obstacle / rock emphasis on live scan. */
function obstacleColor(hag, out, i) {
  const h = Math.max(0, Math.min(1, hag / 1.2));
  out[i] = 0.95 + 0.05 * h;
  out[i + 1] = 0.25 + 0.35 * h;
  out[i + 2] = 0.05 + 0.15 * (1 - h);
}

/**
 * Per-frame height-above-local-ground in body frame.
 * Returns Float32Array hag aligned with lidar.xy (NaN if unknown).
 */
function estimateScanHag(lidar, cell = 1.0) {
  const n = lidar?.xy?.length || 0;
  const hag = new Float32Array(n);
  if (!n) return hag;
  const bins = new Map(); // key -> zs[]
  for (let i = 0; i < n; i++) {
    const x = lidar.xy[i][0];
    const y = lidar.xy[i][1];
    const z = (lidar.z && lidar.z[i]) || 0;
    const kx = Math.floor(x / cell);
    const ky = Math.floor(y / cell);
    const key = `${kx}:${ky}`;
    let arr = bins.get(key);
    if (!arr) {
      arr = [];
      bins.set(key, arr);
    }
    arr.push(z);
  }
  const groundZ = new Map();
  for (const [key, zs] of bins) {
    zs.sort((a, b) => a - b);
    const idx = Math.max(0, Math.floor(zs.length * 0.2));
    groundZ.set(key, zs[idx]);
  }
  for (let i = 0; i < n; i++) {
    const x = lidar.xy[i][0];
    const y = lidar.xy[i][1];
    const z = (lidar.z && lidar.z[i]) || 0;
    const key = `${Math.floor(x / cell)}:${Math.floor(y / cell)}`;
    const gz = groundZ.get(key);
    hag[i] = gz == null ? NaN : z - gz;
  }
  return hag;
}

function rebuildObsMapGeometry() {
  let i = 0;
  for (const v of obsVoxels.values()) {
    if (i >= OBS_MAP_MAX) break;
    const j = i * 3;
    obsMapPositions[j] = v.x;
    obsMapPositions[j + 1] = v.y;
    obsMapPositions[j + 2] = v.z;
    obsMapColors[j] = v.r;
    obsMapColors[j + 1] = v.g;
    obsMapColors[j + 2] = v.b;
    i += 1;
  }
  obsMapGeo.setDrawRange(0, i);
  obsMapGeo.attributes.position.needsUpdate = true;
  obsMapGeo.attributes.color.needsUpdate = true;
}

function appendObstacleWorld(wx, wy, wz, rgb) {
  const inv = 1.0 / OBS_VOXEL;
  const ix = Math.floor(wx * inv);
  const iy = Math.floor(wy * inv);
  const iz = Math.floor(wz * inv);
  const key = `${ix},${iy},${iz}`;
  if (obsVoxels.has(key)) return;
  if (obsVoxels.size >= OBS_MAP_MAX) {
    // drop arbitrary oldest-ish entry
    const first = obsVoxels.keys().next().value;
    obsVoxels.delete(first);
  }
  obsVoxels.set(key, {
    x: (ix + 0.5) * OBS_VOXEL,
    y: (iy + 0.5) * OBS_VOXEL,
    z: (iz + 0.5) * OBS_VOXEL,
    r: rgb[0],
    g: rgb[1],
    b: rgb[2],
  });
}

function rebuildMapGeometry() {
  const genNow = Math.max(mapGen, 1);
  let i = 0;
  for (const v of mapVoxels.values()) {
    if (i >= MAP_MAX) break;
    const j = i * 3;
    mapPositions[j] = v.x;
    mapPositions[j + 1] = v.y;
    mapPositions[j + 2] = v.z;
    const age01 = Math.min(1, (genNow - (v.gen || 0)) / 80);
    mapHeightColor(v.bz ?? 0, age01, mapColors, j);
    i += 1;
  }
  mapCount = i;
  mapGeo.setDrawRange(0, mapCount);
  mapGeo.attributes.position.needsUpdate = true;
  mapGeo.attributes.color.needsUpdate = true;
  mapGeo.computeBoundingSphere();
}

function clearMap() {
  mapVoxels.clear();
  mapCount = 0;
  mapGen = 0;
  mapGeo.setDrawRange(0, 0);
  obsVoxels.clear();
  obsMapGeo.setDrawRange(0, 0);
  lastLidarT = -1;
}

/**
 * Body LiDAR frame is ROS-style: +X forward, +Y left, +Z up.
 * Transform with *raw odom* (not smoothed truck mesh) so rocks/bunds stay
 * locked relative to the vehicle — matching edge_sim body_to_enu.
 */
const _local = new THREE.Vector3();
const _world = new THREE.Vector3();

function bodyRosToTruckLocal(bx, byLeft, bz, out) {
  // ROS (x,y,z)=(fwd,left,up) → Three local (x,y,z)=(fwd,up,right)=(bx,bz,-by)
  out.set(bx, bz, -byLeft);
  return out;
}

/** Body → ENU using same R = Rz(yaw)·Ry(pitch)·Rx(roll) as the sim. */
function bodyToEnu(bx, by, bz, odom) {
  const yaw = odom.yaw || 0;
  const pitch = odom.pitch || 0;
  const roll = odom.roll || 0;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  // Rx
  let x1 = bx;
  let y1 = by * cr - bz * sr;
  let z1 = by * sr + bz * cr;
  // Ry
  let x2 = x1 * cp + z1 * sp;
  let y2 = y1;
  let z2 = -x1 * sp + z1 * cp;
  // Rz
  const ex = x2 * cy - y2 * sy;
  const ey = x2 * sy + y2 * cy;
  const ez = z2;
  return {
    x: (odom.x || 0) + ex,
    y: (odom.y || 0) + ey,
    z: (odom.z || 0) + ez,
  };
}

/** Body ROS → Three world via raw odom (no display lag). */
function bodyToWorld(bx, by, bz, odom, out) {
  const enu = bodyToEnu(bx, by, bz, odom);
  const t = enuToThree(enu.x, enu.y, enu.z);
  out.copy(t);
  return out;
}

/** True if point is ahead of vehicle (body +X). */
function isForward(bx, minX = 1.0) {
  return bx >= minX;
}

/** Shoulder / bund band in body frame (always present along haul road). */
function isBundBand(bx, by, roadHw = 6.5, band = 3.5) {
  if (!isForward(bx, 0.5)) return false;
  const ay = Math.abs(by);
  return ay >= roadHw - 0.8 && ay <= roadHw + band;
}

function bundColor(sideLeft, out, i) {
  // cyan / teal — left slightly greener
  if (sideLeft) {
    out[i] = 0.1; out[i + 1] = 0.95; out[i + 2] = 0.85;
  } else {
    out[i] = 0.2; out[i + 1] = 0.85; out[i + 2] = 1.0;
  }
}

function pruneMapFarFrom(egoThree, maxKeep = MAP_MAX) {
  const r2 = MAP_KEEP_RADIUS_M * MAP_KEEP_RADIUS_M;
  // First drop anything beyond keep radius
  for (const [k, v] of [...mapVoxels.entries()]) {
    const dx = v.x - egoThree.x;
    const dy = v.y - egoThree.y;
    const dz = v.z - egoThree.z;
    if (dx * dx + dy * dy + dz * dz > r2) mapVoxels.delete(k);
  }
  if (mapVoxels.size <= maxKeep) return;
  // Then drop farthest until under budget
  const ranked = [];
  for (const [k, v] of mapVoxels) {
    const dx = v.x - egoThree.x;
    const dy = v.y - egoThree.y;
    const dz = v.z - egoThree.z;
    ranked.push([k, dx * dx + dy * dy + dz * dz]);
  }
  ranked.sort((a, b) => b[1] - a[1]);
  const excess = mapVoxels.size - maxKeep;
  for (let i = 0; i < excess; i++) mapVoxels.delete(ranked[i][0]);
}

function appendLidarToMap(lidar, odom) {
  if (!lidar?.xy?.length || !odom) return;
  // Only integrate when a new scan timestamp arrives
  if (lidar.t != null && Math.abs(lidar.t - lastLidarT) < 1e-6) return;
  lastLidarT = lidar.t ?? lastLidarT;

  mapGen += 1;
  const invV = 1.0 / VOXEL;
  const n = lidar.xy.length;
  // Keep most points for detail (cap integration work ~12k/frame)
  const step = Math.max(1, Math.floor(n / 12000));

  const full = !els.togFullScan || els.togFullScan.checked;
  for (let i = 0; i < n; i += step) {
    const bx = lidar.xy[i][0]; // forward
    const by = lidar.xy[i][1]; // left
    const bz = (lidar.z && lidar.z[i]) || 0;
    if (!full && !(bx > 0.5)) continue; // optional forward-only

    bodyToWorld(bx, by, bz, odom, _world);

    const ix = Math.floor(_world.x * invV);
    const iy = Math.floor(_world.y * invV);
    const iz = Math.floor(_world.z * invV);
    const key = `${ix},${iy},${iz}`;

    const prev = mapVoxels.get(key);
    if (!prev || bz >= (prev.bz ?? -999)) {
      mapVoxels.set(key, {
        x: (ix + 0.5) * VOXEL,
        y: (iy + 0.5) * VOXEL,
        z: (iz + 0.5) * VOXEL,
        bz,
        gen: mapGen,
      });
    } else {
      // refresh age so recently re-seen cells stay a bit brighter
      prev.gen = mapGen;
    }
  }

  // Drop farthest voxels only (keep corridor behind + ahead near ego)
  const ego = enuToThree(odom.x, odom.y, odom.z);
  pruneMapFarFrom(ego, MAP_MAX);
  rebuildMapGeometry();
}

let camMode = 'follow'; // follow | top | free
let lastOdom = null;
let followDist = 40;
let followHeight = 22;
let followYawOff = 0.35; // rad side angle so you see the side/rear, not locked dead-center

function frameBehindTruck(odom, dist = followDist, height = followHeight) {
  const yaw = (odom.yaw || 0) + followYawOff;
  const ex = odom.x - Math.cos(yaw) * dist;
  const ey = odom.y - Math.sin(yaw) * dist;
  const target = enuToThree(odom.x, odom.y, odom.z);
  target.y += 1.8;
  controls.target.copy(target);
  camera.position.copy(enuToThree(ex, ey, odom.z + height));
  controls.update();
}

function setCamMode(mode) {
  camMode = mode;
  els.camMode.textContent =
    mode === 'follow' ? 'follow vehicle' : mode === 'top' ? 'top-down' : 'free (world)';
  controls.enabled = true;
  // Pan only in free mode (pan moves world target; follow keeps target = truck)
  controls.enablePan = mode === 'free';
  controls.enableRotate = true;
  controls.enableZoom = true;

  if (!lastOdom) return;
  if (mode === 'follow') {
    frameBehindTruck(lastOdom);
  } else if (mode === 'top') {
    const t = enuToThree(lastOdom.x, lastOdom.y, lastOdom.z);
    t.y += 1.0;
    controls.target.copy(t);
    // High overview so most of the accumulated corridor is in frame
    camera.position.copy(enuToThree(lastOdom.x, lastOdom.y, lastOdom.z + 180));
    followDist = 180;
    controls.update();
  }
}

// Wheel in follow mode: change chase distance (and still allow OrbitControls zoom)
canvas.addEventListener('wheel', (e) => {
  if (camMode !== 'follow') return;
  followDist = Math.min(800, Math.max(8, followDist + (e.deltaY > 0 ? 4 : -4)));
}, { passive: true });

window.addEventListener('keydown', (e) => {
  if (e.key === '1') setCamMode('follow');
  if (e.key === '2') setCamMode('top');
  if (e.key === '3') setCamMode('free');
  if (e.key === 'm' || e.key === 'M') {
    clearMap();
    if (els.p_lidar) els.p_lidar.textContent = 'map cleared';
  }
});

// Smooth pose for mesh (display only — HUD still shows raw samples)
const disp = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0, init: false };
function lerp(a, b, k) { return a + (b - a) * k; }
function lerpAngle(a, b, k) {
  let d = b - a;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return a + d * k;
}

let followSeeded = false;

function updateTruck(odom) {
  if (!odom) return;
  lastOdom = odom;

  if (!followSeeded) {
    followSeeded = true;
    frameBehindTruck(odom);
    els.camMode.textContent = 'follow vehicle';
  }

  if (!disp.init) {
    disp.x = odom.x; disp.y = odom.y; disp.z = odom.z;
    disp.yaw = odom.yaw || 0;
    disp.pitch = odom.pitch || 0;
    disp.roll = odom.roll || 0;
    disp.init = true;
  } else {
    const k = 0.18; // light display smoothing — not extra dynamics
    disp.x = lerp(disp.x, odom.x, k);
    disp.y = lerp(disp.y, odom.y, k);
    disp.z = lerp(disp.z, odom.z, k);
    disp.yaw = lerpAngle(disp.yaw, odom.yaw || 0, k);
    // attitude already small/filtered in sim; tiny display lag only
    disp.pitch = lerp(disp.pitch, odom.pitch || 0, 0.12);
    disp.roll = lerp(disp.roll, odom.roll || 0, 0.12);
  }

  const p = enuToThree(disp.x, disp.y, disp.z);
  truck.position.copy(p);
  truck.rotation.order = 'YXZ';
  truck.rotation.y = disp.yaw;
  truck.rotation.x = -disp.pitch;
  truck.rotation.z = disp.roll;

  // Reference pad under ego (not terrain content)
  ground.position.x = p.x;
  ground.position.z = p.z;
  ground.position.y = p.y - 0.05;
  grid.position.x = p.x;
  grid.position.z = p.z;
  grid.position.y = p.y + 0.01;

  // trail from raw odom (truth path)
  const raw = enuToThree(odom.x, odom.y, odom.z);
  if (trailCount < trailMax) {
    const i = trailCount * 3;
    trailPositions[i] = raw.x;
    trailPositions[i + 1] = raw.y + 0.12;
    trailPositions[i + 2] = raw.z;
    trailCount += 1;
    trailGeo.setDrawRange(0, trailCount);
    trailGeo.attributes.position.needsUpdate = true;
  } else {
    trailPositions.copyWithin(0, 3);
    const i = (trailMax - 1) * 3;
    trailPositions[i] = raw.x;
    trailPositions[i + 1] = raw.y + 0.12;
    trailPositions[i + 2] = raw.z;
    trailGeo.attributes.position.needsUpdate = true;
  }
}

function updateLidar(lidar, odom) {
  if (!lidar?.xy?.length || !odom) return null;

  // Display mesh can lag slightly; clouds always use raw odom.
  updateTruck(odom);

  // 1) Append this scan into the persistent world map
  if (mapCloud.visible) appendLidarToMap(lidar, odom);

  // 2) Current scan, coloured by class when the perception pass labelled it.
  const count = lidar.xy.length;
  const labels = Array.isArray(lidar.label) && lidar.label.length === count ? lidar.label : null;
  const confs = Array.isArray(lidar.conf) && lidar.conf.length === count ? lidar.conf : null;
  const hag = labels ? null : estimateScanHag(lidar, 0.75);
  const full = !els.togFullScan || els.togFullScan.checked;
  const showBund = layerVisible.get('bund') !== false;
  const showObs = layerVisible.get('obstacle') !== false;
  const n = Math.min(count, scanMax);
  const col = [0, 0, 0];
  const counts = {};
  const liveBund = []; // unlabelled fallback only — shoulders from raw geometry
  let w = 0;
  let obsMapDirty = false;

  if (!scanCloud.visible) {
    scanGeo.setDrawRange(0, 0);
    return { scan: 0, counts, labelled: !!labels };
  }

  for (let i = 0; i < n; i++) {
    const bx = lidar.xy[i][0];
    const by = lidar.xy[i][1];
    const bz = (lidar.z && lidar.z[i]) || 0;
    if (!full && !(bx > 0.5)) continue;
    // Ignore near-field returns (cab / hood / bumper) — common false "rocks"
    if (bx < 3.5 && Math.abs(by) < 2.5) continue;

    const cls = labels ? classFor(labels[i]) : null;
    if (labels) {
      const layer = cls ? cls.layer : 'unknown';
      if (layerVisible.get(layer) === false) continue;
      const key = cls ? cls.key : 'unknown';
      counts[key] = (counts[key] || 0) + 1;
    }

    bodyToWorld(bx, by, bz, odom, _world);
    const j = w * 3;
    scanPositions[j] = _world.x;
    scanPositions[j + 1] = _world.y;
    scanPositions[j + 2] = _world.z;

    if (labels) {
      const rgb = cls ? cls.rgb : UNKNOWN_RGB;
      const c = confs ? Math.max(0.45, Math.min(1, confs[i])) : 1;
      const shade = 0.55 + 0.45 * c;
      col[0] = rgb[0] * shade;
      col[1] = rgb[1] * shade;
      col[2] = rgb[2] * shade;
      // World-fixed detection map keeps everything that is not driving surface.
      if (cls && accumulates(cls.layer) && bx >= 6.0) {
        appendObstacleWorld(_world.x, _world.y, _world.z, col);
        obsMapDirty = true;
      }
    } else {
      // No labels on this source (raw bus scan): keep the geometric heuristic.
      const h = hag[i];
      const bund = isBundBand(bx, by);
      const elevated = Number.isFinite(h) && h > 0.28 && bx >= 5.0;
      if (bund) {
        bundColor(by > 0, col, 0);
        counts.bund = (counts.bund || 0) + 1;
        if (showBund) liveBund.push(_world.x, _world.y, _world.z);
      } else if (elevated && showObs) {
        obstacleColor(h, col, 0);
        counts.obstacle = (counts.obstacle || 0) + 1;
        if (bx >= 8.0) {
          appendObstacleWorld(_world.x, _world.y, _world.z, col);
          obsMapDirty = true;
        }
      } else {
        scanHeightColor(bz, col, 0);
      }
    }

    scanColors[j] = col[0];
    scanColors[j + 1] = col[1];
    scanColors[j + 2] = col[2];
    w += 1;
  }

  scanGeo.setDrawRange(0, w);
  scanGeo.attributes.position.needsUpdate = true;
  scanGeo.attributes.color.needsUpdate = true;

  if (!labels) {
    // Bunds from the live scan keep the unlabelled view usable.
    fillWorldLayer(detectLayers.bund, liveBund, layerRgb('bund'), showBund);
  }
  if (obsMapDirty) rebuildObsMapGeometry();
  return { scan: w, counts, labelled: !!labels };
}

function fillWorldLayer(layer, xyzFlat, rgb, visible) {
  if (!visible || !xyzFlat.length) {
    layer.geo.setDrawRange(0, 0);
    return 0;
  }
  const n = Math.min(Math.floor(xyzFlat.length / 3), layer.maxN);
  for (let i = 0; i < n; i++) {
    const j = i * 3;
    layer.positions[j] = xyzFlat[j];
    layer.positions[j + 1] = xyzFlat[j + 1];
    layer.positions[j + 2] = xyzFlat[j + 2];
    layer.colors[j] = rgb[0];
    layer.colors[j + 1] = rgb[1];
    layer.colors[j + 2] = rgb[2];
  }
  layer.geo.setDrawRange(0, n);
  layer.geo.attributes.position.needsUpdate = true;
  layer.geo.attributes.color.needsUpdate = true;
  return n;
}

function fillDetectLayer(layer, cloud, rgb, odom, { forwardOnly = false, minX = 0.0, keepPreviousIfEmpty = false } = {}) {
  if (!layer.pts.visible || !odom) {
    if (!keepPreviousIfEmpty) layer.geo.setDrawRange(0, 0);
    return 0;
  }
  if (!cloud?.xy?.length) {
    // No current detection. For detector-only layers (rocks/obstacles/ground)
    // clear the overlay so stale points don't persist across frames — the
    // world-fixed obstacle voxel map already retains history. For dual-sourced
    // layers (bunds, also filled from the live scan) keep the prior content.
    if (!keepPreviousIfEmpty) layer.geo.setDrawRange(0, 0);
    return 0;
  }
  const n = Math.min(cloud.xy.length, layer.maxN);
  let w = 0;
  const confs = cloud.conf || [];
  for (let i = 0; i < n; i++) {
    const bx = cloud.xy[i][0];
    const by = cloud.xy[i][1];
    const bz = (cloud.z && cloud.z[i]) || 0;
    if (bx < minX) continue;
    if (forwardOnly && !isForward(bx, minX || 1.0)) continue;
    bodyToWorld(bx, by, bz, odom, _world);
    const j = w * 3;
    layer.positions[j] = _world.x;
    layer.positions[j + 1] = _world.y;
    layer.positions[j + 2] = _world.z;
    const c = confs[i] != null ? Math.max(0.35, Math.min(1, confs[i])) : 1;
    layer.colors[j] = rgb[0] * c;
    layer.colors[j + 1] = rgb[1] * c;
    layer.colors[j + 2] = rgb[2] * (0.7 + 0.3 * c);
    w += 1;
  }
  // Always update the draw range for detector-only layers, even when w == 0,
  // so filtered-out or empty detections clear the overlay instead of leaving
  // stale points. Dual-sourced layers keep their prior content when empty.
  if (w > 0 || !keepPreviousIfEmpty) {
    layer.geo.setDrawRange(0, w);
    layer.geo.attributes.position.needsUpdate = true;
    layer.geo.attributes.color.needsUpdate = true;
  }
  return w;
}

/** Split one labelled cloud across the class layers it covers. */
function fillSemanticCloud(cloud, odom) {
  const counts = {};
  const buckets = new Map(); // class key → flat world xyz
  const labels = cloud.label || [];
  const n = Math.min(cloud.xy.length, 20000);
  for (let i = 0; i < n; i++) {
    const cls = classFor(labels[i]);
    const key = cls ? cls.key : 'unknown';
    counts[key] = (counts[key] || 0) + 1;
    const layer = cls ? cls.layer : 'unknown';
    if (layerVisible.get(layer) === false) continue;
    const bx = cloud.xy[i][0];
    const by = cloud.xy[i][1];
    const bz = (cloud.z && cloud.z[i]) || 0;
    bodyToWorld(bx, by, bz, odom, _world);
    let bucket = buckets.get(key);
    if (!bucket) {
      bucket = [];
      buckets.set(key, bucket);
    }
    bucket.push(_world.x, _world.y, _world.z);
    if (cls && accumulates(cls.layer) && isForward(bx, 8.0)) {
      appendObstacleWorld(_world.x, _world.y, _world.z, cls.rgb);
    }
  }
  for (const [layerKey, layer] of Object.entries(detectLayers)) {
    const classKeys = styles.classes.filter((c) => c.layer === layerKey).map((c) => c.key);
    const flat = [];
    for (const key of classKeys) {
      const bucket = buckets.get(key);
      // Push in a loop: spreading a multi-thousand point bucket overflows.
      if (bucket) for (const v of bucket) flat.push(v);
    }
    fillWorldLayer(layer, flat, layerRgb(classKeys[0] || layerKey), layerVisible.get(layerKey) !== false);
  }
  rebuildObsMapGeometry();
  return counts;
}

function updateDetectClouds(detect, odom) {
  const counts = {};
  if (!odom) return counts;
  const clouds = detect?.clouds || {};
  // A labelled full frame supersedes the per-class clouds: drawing both would
  // paint the same returns twice.
  if (clouds.semantic?.xy?.length) return fillSemanticCloud(clouds.semantic, odom);

  for (const [kind, classKey] of Object.entries(CLOUD_CLASS)) {
    const layer = detectLayers[classKey];
    if (!layer) continue;
    // Detector rock/obstacle clouds only past the near field, so body-locked
    // ghosts in front of the cab drop out instead of sliding with the truck.
    const near = classKey === 'rock' || classKey === 'obstacle';
    counts[classKey] = fillDetectLayer(layer, clouds[kind], layerRgb(classKey), odom, {
      forwardOnly: near,
      minX: near ? 8.0 : 2.0,
      // Bunds are also drawn from the live scan, so an empty frame is not
      // evidence that the previous shoulder points are gone.
      keepPreviousIfEmpty: classKey === 'bund',
    });
  }

  // World-fixed detection map from the far part of the detector clouds.
  let dirty = false;
  for (const [kind, classKey] of Object.entries(CLOUD_CLASS)) {
    if (!accumulates(classKey)) continue;
    const cloud = clouds[kind];
    if (!cloud?.xy?.length) continue;
    const rgb = layerRgb(classKey);
    const n = Math.min(cloud.xy.length, 4000);
    for (let i = 0; i < n; i++) {
      const bx = cloud.xy[i][0];
      const by = cloud.xy[i][1];
      const bz = (cloud.z && cloud.z[i]) || 0;
      if (!isForward(bx, 8.0)) continue;
      bodyToWorld(bx, by, bz, odom, _world);
      appendObstacleWorld(_world.x, _world.y, _world.z, rgb);
      dirty = true;
    }
  }
  if (dirty) rebuildObsMapGeometry();
  return counts;
}

function updateDetectionMarkers(_detections) {
  // Per-frame body-frame detections get no sphere of their own: re-projecting
  // them every tick made false rocks sit in front of the truck and slide with
  // it. Classes ride on the scan colours; events below freeze in the world.
}

function severityColor(sev) {
  return new THREE.Color(styles.severities[sev] || styles.severities.info).getHex();
}

const LABEL_HEIGHT_M = 1.7;
const LABEL_STALK_M = 3.4;

/** Billboard text tag drawn above an event marker. */
function makeLabelSprite(text, color, severity) {
  const dpr = 2;
  const pad = 14 * dpr;
  const fontPx = 22 * dpr;
  const font = `600 ${fontPx}px "IBM Plex Sans", "Segoe UI", system-ui, sans-serif`;
  const canvas = document.createElement('canvas');
  const measure = canvas.getContext('2d');
  measure.font = font;
  const textWidth = measure.measureText(text).width;
  canvas.width = Math.ceil(textWidth + pad * 2 + 10 * dpr);
  canvas.height = Math.ceil(fontPx + pad * 1.4);

  const ctx = canvas.getContext('2d');
  ctx.font = font;
  ctx.textBaseline = 'middle';
  const radius = canvas.height / 2;
  ctx.beginPath();
  ctx.moveTo(radius, 0);
  ctx.lineTo(canvas.width - radius, 0);
  ctx.arcTo(canvas.width, 0, canvas.width, radius, radius);
  ctx.arcTo(canvas.width, canvas.height, canvas.width - radius, canvas.height, radius);
  ctx.lineTo(radius, canvas.height);
  ctx.arcTo(0, canvas.height, 0, radius, radius);
  ctx.arcTo(0, 0, radius, 0, radius);
  ctx.closePath();
  ctx.fillStyle = 'rgba(8, 13, 20, 0.86)';
  ctx.fill();
  ctx.lineWidth = 2.5 * dpr;
  ctx.strokeStyle = styles.severities[severity] || color;
  ctx.stroke();

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pad, canvas.height / 2, 5 * dpr, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#e8eef6';
  ctx.fillText(text, pad + 12 * dpr, canvas.height / 2 + dpr);

  const texture = new THREE.CanvasTexture(canvas);
  texture.anisotropy = 4;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false, depthWrite: false }),
  );
  sprite.scale.set((LABEL_HEIGHT_M * canvas.width) / canvas.height, LABEL_HEIGHT_M, 1);
  sprite.renderOrder = 10;
  return sprite;
}

/** Deterministic 0/1/2 rung so tags on clustered events do not overlap. */
function labelRung(id) {
  let hash = 0;
  for (let i = 0; i < id.length; i++) hash = (hash * 31 + id.charCodeAt(i)) % 997;
  return hash % 3;
}

function makeEventMarker(alert, id) {
  const type = String(alert.type || 'event');
  const style = eventStyleFor(type);
  const colorHex = eventColor(type, alert.severity);
  const color = new THREE.Color(colorHex);
  const group = new THREE.Group();

  const radius = Math.min(1.6, Math.max(0.4, Number(alert.geometry?.radius_m) || 0.6));
  const shape =
    type === 'rock'
      ? new THREE.SphereGeometry(radius, 14, 10)
      : new THREE.OctahedronGeometry(Math.max(radius, 0.9));
  const mesh = new THREE.Mesh(
    shape,
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.45, depthWrite: false }),
  );
  mesh.renderOrder = 4;
  group.add(mesh);

  const stalkGeo = new THREE.BufferGeometry();
  stalkGeo.setAttribute(
    'position',
    new THREE.BufferAttribute(new Float32Array([0, 0, 0, 0, LABEL_STALK_M, 0]), 3),
  );
  group.add(
    new THREE.Line(
      stalkGeo,
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.75, depthTest: false }),
    ),
  );

  const text = String(alert.label || (style ? style.short : type)).slice(0, 44);
  const sprite = makeLabelSprite(text, colorHex, alert.severity);
  const stalk = LABEL_STALK_M + labelRung(id) * LABEL_HEIGHT_M * 1.15;
  stalkGeo.attributes.position.setY(1, stalk);
  sprite.position.set(0, stalk + LABEL_HEIGHT_M * 0.65, 0);
  sprite.visible = !els.togLabels || els.togLabels.checked;
  group.add(sprite);
  group.userData.sprite = sprite;
  return group;
}

function disposeMarker(group) {
  group.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (child.material.map) child.material.map.dispose();
      child.material.dispose();
    }
  });
}

function upsertAlertMarker(alert) {
  const id = alert.event_id || `${alert.type}-${alert.t_ros || alert.t_vehicle}`;
  const existing = alertMarkerById.get(id);
  if (existing) {
    // Keep the frozen world pose — never re-anchor from body each tick.
    existing.t0 = performance.now();
    return;
  }

  const pose = alert.pose || {};
  const frame = alert.frame_id || 'map';
  let position;
  if (frame === 'base_link' || frame === 'lidar_link') {
    // Body-frame events (bus mode) are only trustworthy well ahead of the cab.
    if (!lastOdom || !isForward(pose.x || 0, 8.0)) return;
    bodyToWorld(pose.x || 0, pose.y || 0, pose.z || 0, lastOdom, _world);
    position = _world.clone();
  } else {
    if (pose.x == null && pose.y == null) return;
    position = enuToThree(pose.x || 0, pose.y || 0, pose.z || 0);
    position.y += 0.4;
  }

  const group = makeEventMarker(alert, id);
  group.position.copy(position);
  alertMarkers.add(group);
  alertMarkerById.set(id, { mesh: group, t0: performance.now() });
}

function pruneAlertMarkers() {
  const now = performance.now();
  for (const [id, entry] of [...alertMarkerById.entries()]) {
    if (now - entry.t0 > ALERT_MARKER_TTL_MS) {
      alertMarkers.remove(entry.mesh);
      disposeMarker(entry.mesh);
      alertMarkerById.delete(id);
    }
  }
}

function applyLabelVisibility() {
  const show = !els.togLabels || els.togLabels.checked;
  for (const entry of alertMarkerById.values()) {
    const sprite = entry.mesh.userData?.sprite;
    if (sprite) sprite.visible = show;
  }
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (c) => (
    { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]
  ));
}

function alertBody(a) {
  const det = a.details || {};
  if (a.type === 'rock') {
    return `Ø ${fmt(2 * (a.geometry?.radius_m ?? 0), 2)} m · range ${fmt(det.range_m, 1)} m · `
      + `${det.in_lane ? 'in lane' : 'off lane'} · conf ${fmt(a.confidence, 2)}`;
  }
  if (a.type === 'bund_low') {
    return `${det.side || ''} · ${fmt(det.min_height_m, 2)} m vs ${fmt(det.spec_height_m, 2)} m spec`
      + ` · short by ${fmt(det.deficit_m, 2)} m over ${fmt(det.length_m, 0)} m`;
  }
  if (a.type === 'bund_gap') {
    return `${det.side || ''} · no crest over ${fmt(det.length_m, 0)} m`;
  }
  if (a.type === 'excessive_vibration') {
    return `RMS az ${fmt(det.rms_az, 2)} · peak ${fmt(det.peak_az, 2)} · v ${fmt(det.speed_mps, 1)} m/s`;
  }
  return escapeHtml(JSON.stringify(det).slice(0, 80));
}

function renderAlertFeed(alerts) {
  if (!els.alertFeed) return;
  const list = Array.isArray(alerts) ? alerts : [];
  if (els.alertBadge) {
    els.alertBadge.textContent = `events ${list.length}`;
    const hot = list.some((a) => a.severity === 'critical' || a.severity === 'warn');
    els.alertBadge.className = hot ? 'badge alert-hot' : 'badge';
  }
  if (!list.length) {
    els.alertFeed.innerHTML = '<div class="alert-empty">No events yet</div>';
    return;
  }
  els.alertFeed.innerHTML = list
    .slice(0, 12)
    .map((a) => {
      const sev = a.severity || 'info';
      const t = a.t_vehicle ?? a.t_ros ?? 0;
      const style = eventStyleFor(a.type);
      const title = escapeHtml(style ? style.title : a.type || 'event');
      const swatch = `<span class="swatch" style="background:${eventColor(a.type, sev)}"></span>`;
      return `<div class="alert-item ${escapeHtml(sev)}" title="${escapeHtml(a.label || title)}">`
        + `<div class="ah"><span class="atype">${swatch}${title}</span>`
        + `<span class="asev">${escapeHtml(sev)} · t=${fmt(t, 1)}s</span></div>`
        + `<div class="abody">${alertBody(a)}</div></div>`;
    })
    .join('');
}

function renderClassCounts(counts) {
  if (!els.classCounts) return;
  els.classCounts.innerHTML = styles.classes
    .map((c) => {
      const hidden = layerVisible.get(c.layer) === false;
      const n = hidden ? 'hidden' : Number(counts?.[c.key] || 0);
      return `<span class="class-chip${hidden ? ' off' : ''}" title="${escapeHtml(c.description || '')}">`
        + `<span class="swatch" style="background:${c.color}"></span>${escapeHtml(c.title)} <b>${n}</b></span>`;
    })
    .join('');
}

function buildClassToggles() {
  if (!els.classToggles) return;
  els.classToggles.innerHTML = '';
  for (const layer of styles.layers) {
    const label = document.createElement('label');
    label.className = 'tog';
    label.title = styles.classes
      .filter((c) => c.layer === layer.key)
      .map((c) => c.title)
      .join(' · ');
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.checked = layerVisible.get(layer.key) !== false;
    input.addEventListener('change', () => {
      layerVisible.set(layer.key, input.checked);
      applyLayerToggles();
    });
    const swatch = document.createElement('span');
    const first = styles.classes.find((c) => c.layer === layer.key);
    swatch.className = 'swatch';
    swatch.style.background = first ? first.color : '#7d8899';
    label.append(input, swatch, document.createTextNode(` ${layer.title}`));
    els.classToggles.appendChild(label);
  }
}

function buildLegend() {
  if (!els.legend) return;
  const items = [
    ...styles.classes.map(
      (c) => `<span class="legend-item" title="${escapeHtml(c.description || '')}">`
        + `<span class="swatch" style="background:${c.color}"></span>${escapeHtml(c.title)}</span>`,
    ),
    ...styles.events.map(
      (e) => `<span class="legend-item event" title="${escapeHtml(e.description || '')}">`
        + `<span class="swatch" style="background:${e.color}"></span>${escapeHtml(e.short)}</span>`,
    ),
    '<span class="legend-item"><span class="swatch map-old"></span>prior map</span>',
    '<span class="legend-item"><span class="swatch truck"></span>truck</span>',
  ];
  els.legend.innerHTML = items.join('');
}

function applyLayerToggles() {
  scanCloud.visible = !els.togRaw || els.togRaw.checked;
  mapCloud.visible = !els.togMap || els.togMap.checked;
  for (const [key, layer] of Object.entries(detectLayers)) {
    layer.pts.visible = layerVisible.get(key) !== false;
  }
  alertMarkers.visible = !els.togMarkers || els.togMarkers.checked;
  obsMapCloud.visible = !els.togDetectMap || els.togDetectMap.checked;
  applyLabelVisibility();
}

for (const key of ['togRaw', 'togMap', 'togDetectMap', 'togMarkers', 'togLabels', 'togFullScan']) {
  const el = els[key];
  if (el) el.addEventListener('change', applyLayerToggles);
}
applyLayerToggles();

function updateCamera() {
  if (!lastOdom) return;

  const target = enuToThree(lastOdom.x, lastOdom.y, lastOdom.z);
  target.y += 1.8;

  if (camMode === 'follow') {
    // Slide camera with the truck, keeping the current orbit offset (from user drag/zoom).
    const offset = new THREE.Vector3().subVectors(camera.position, controls.target);
    if (offset.lengthSq() < 4) {
      frameBehindTruck(lastOdom, followDist, followHeight);
      return;
    }
    // Nudge distance toward wheel-selected followDist without killing orbit angle
    offset.setLength(lerp(offset.length(), followDist, 0.12));
    controls.target.copy(target);
    camera.position.copy(target).add(offset);
  } else if (camMode === 'top') {
    controls.target.lerp(target, 0.25);
    const up = new THREE.Vector3(0, Math.max(120, followDist), 0.01);
    camera.position.lerp(target.clone().add(up), 0.2);
  }
  // free: OrbitControls only — world-fixed target, no auto move
}

function onResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}
window.addEventListener('resize', onResize);

function animate() {
  requestAnimationFrame(animate);
  updateCamera();
  controls.update();
  renderer.render(scene, camera);
}
animate();

// ——— Physics HUD + charts ———
function applyPhysics(msg) {
  const o = msg.odom || {};
  const imu = msg.imu || {};
  const gnss = msg.gnss || {};

  const speed = o.speed ?? Math.hypot(o.vx || 0, o.vy || 0);
  const kmh = speed * 3.6;
  els.p_speed.innerHTML = `${fmt(speed, 2)} m/s &nbsp;·&nbsp; ${fmt(kmh, 1)} km/h`;
  els.p_vel.textContent = `${fmt(o.vx, 2)}, ${fmt(o.vy, 2)}, ${fmt(o.vz, 2)} m/s`;
  els.p_yaw.textContent = `${fmt((o.yaw || 0) * RAD2DEG, 1)}°`;
  els.p_att.textContent = `${fmt((o.pitch || 0) * RAD2DEG, 2)}° / ${fmt((o.roll || 0) * RAD2DEG, 2)}°`;
  els.p_pos.textContent = `${fmt(o.x, 1)}, ${fmt(o.y, 1)}, ${fmt(o.z, 2)} m`;
  els.p_alt.textContent = gnss.alt != null ? `${fmt(gnss.alt, 1)} m MSL` : '—';

  const ax = imu.ax ?? o.ax ?? 0;
  const ay = imu.ay ?? o.ay ?? 0;
  const az = imu.az ?? o.az ?? G;
  const sf = Math.hypot(ax, ay, az);
  const aHoriz = Math.hypot(ax, ay);
  const vertG = az / G;
  els.p_sf.textContent = `${fmt(ax, 2)}, ${fmt(ay, 2)}, ${fmt(az, 2)}  (|f|=${fmt(sf, 2)})`;
  els.p_g.textContent = `${fmt(aHoriz / G, 2)} g horiz · ${fmt(vertG, 2)} g vert`;

  const gz = imu.gz ?? o.yaw_rate ?? 0;
  const gx = imu.gx ?? o.roll_rate ?? 0;
  const gy = imu.gy ?? o.pitch_rate ?? 0;
  els.p_yr.textContent = `${fmt(gz, 3)} rad/s · ${fmt(gz * RAD2DEG, 1)} °/s`;
  els.p_pr.textContent = `${fmt(gy * RAD2DEG, 1)} / ${fmt(gx * RAD2DEG, 1)} °/s`;

  // a_n ≈ v * ω (horizontal)
  const cent = speed * Math.abs(gz);
  els.p_cent.textContent = `${fmt(cent, 2)} m/s² · ${fmt(cent / G, 2)} g`;

  if (gnss.fix_ok) {
    els.p_fix.innerHTML = '<span style="color:var(--ok)">FIX</span>';
  } else if (gnss.fix_ok === false) {
    els.p_fix.innerHTML = '<span style="color:var(--bad)">NO FIX</span>';
  } else {
    els.p_fix.textContent = '—';
  }
  els.p_ll.textContent =
    gnss.lat != null ? `${fmt(gnss.lat, 6)}°, ${fmt(gnss.lon, 6)}°` : '—';
  const scanN = msg.lidar?.n;
  els.p_lidar.textContent =
    scanN != null ? `scan ${scanN} · map ${mapCount.toLocaleString()} vox` : `map ${mapCount.toLocaleString()} vox`;

  const roadHw = msg.detect?.road_half_width_m;
  if (els.p_road) {
    els.p_road.textContent = roadHw ? `${fmt(roadHw, 2)} m` : '—';
  }

  const vibe = msg.detect?.vibe;
  if (els.p_vibe) {
    els.p_vibe.textContent = vibe
      ? `${fmt(vibe.rms_az, 2)} · peak ${fmt(vibe.peak_az, 2)}`
      : '—';
  }

  els.clock.textContent = `t = ${fmt(msg.t, 2)} s`;
  els.cycle.textContent = `cycle ${msg.cycle ?? 0}`;
  if (els.source) {
    const src = msg.source || msg.map_id || '—';
    els.source.textContent = src;
  }
  if (els.mapSelect && msg.map_id && els.mapSelect.value !== msg.map_id) {
    const opt = [...els.mapSelect.options].find((o) => o.value === msg.map_id);
    if (opt) els.mapSelect.value = msg.map_id;
  }
}

function applyCharts(msg) {
  if (msg.imu_tail?.length) {
    setSeries(accelChart, [
      msg.imu_tail.map((s) => ({ x: s.t, y: s.ax })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.ay })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.az })),
    ]);
    setSeries(gyroChart, [
      msg.imu_tail.map((s) => ({ x: s.t, y: s.gx * RAD2DEG })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.gy * RAD2DEG })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.gz * RAD2DEG })),
    ]);
  }
  if (msg.odom_tail?.length && msg.imu_tail?.length) {
    const imuByT = msg.imu_tail;
    setSeries(speedChart, [
      msg.odom_tail.map((s) => ({ x: s.t, y: (s.speed || 0) * 3.6 })),
      imuByT.map((s) => ({ x: s.t, y: s.az / G })),
    ]);
  }
}

function applyTick(msg) {
  applyPhysics(msg);
  applyCharts(msg);
  // LiDAR path updates truck pose then map/scan (avoid double pose jump)
  let live = null;
  if (msg.lidar && msg.odom) {
    live = updateLidar(msg.lidar, msg.odom);
  } else if (msg.odom) {
    updateTruck(msg.odom);
  }

  // Bus-mode detector clouds, if any, land in the same class layers.
  const busCounts = updateDetectClouds(msg.detect, msg.odom || lastOdom);
  updateDetectionMarkers(msg.detect?.detections);
  // Prefer the classified scan; fall back to the detector clouds, then to the
  // per-class totals the server reported for this frame.
  const counts = live?.labelled
    ? live.counts
    : { ...(msg.detect?.counts || {}), ...busCounts, ...(live?.counts || {}) };
  renderClassCounts(counts);

  const alerts = msg.detect?.alerts || [];
  renderAlertFeed(alerts);
  if (alertMarkers.visible) {
    for (const a of alerts) upsertAlertMarker(a);
  }
  pruneAlertMarkers();
}

let lastCycle = 0;
function applyTickFull(msg) {
  lastCycle = msg.cycle ?? lastCycle;
  applyTick(msg);
}

function connectWs() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => {
    els.conn.textContent = 'WS live';
    els.conn.className = 'badge ok';
  };
  ws.onclose = () => {
    els.conn.textContent = 'WS offline';
    els.conn.className = 'badge bad';
    setTimeout(connectWs, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data);
      applyTickFull(msg);
    } catch (err) {
      // Swallowing this silently once hid a render bug behind a frozen clock.
      console.error('tick render failed', err);
    }
  };
}

async function loadCatalog() {
  if (!els.mapSelect) return;
  try {
    const cat = await fetch('/api/catalog').then((r) => r.json());
    const maps = cat.maps || [];
    const active = cat.active?.map_id || '';
    els.mapSelect.innerHTML = '';
    for (const m of maps) {
      const opt = document.createElement('option');
      opt.value = m.id;
      opt.textContent = m.title || m.id;
      els.mapSelect.appendChild(opt);
    }
    if (active) els.mapSelect.value = active;
    els.mapSelect.onchange = async () => {
      const map_id = els.mapSelect.value;
      try {
        await fetch('/api/select', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ map_id }),
        });
      } catch (_) { /* ignore */ }
    };
    // Disable map switch while replaying a fixed stream
    els.mapSelect.disabled = (cat.mode || '') === 'replay';
  } catch (_) { /* starting */ }
}

async function bootstrap() {
  await loadStyles();
  buildClassToggles();
  buildLegend();
  applyLayerToggles();
  await loadCatalog();
  try {
    const st = await fetch('/api/status').then((r) => r.json());
    const lid = await fetch('/api/lidar').then((r) => r.json()).catch(() => null);
    applyTickFull({
      t: st.t,
      cycle: st.cycle,
      map_id: st.map_id,
      source: st.source,
      imu: st.latest?.imu,
      gnss: st.latest?.gnss,
      odom: st.latest?.odom,
      lidar: lid,
      imu_tail: st.history?.imu || [],
      odom_tail: st.history?.odom || [],
      gnss_tail: st.history?.gnss || [],
      detect: st.detect || { clouds: {}, detections: null, alerts: [], vibe: null, counts: {} },
    });
  } catch (_) { /* starting */ }
  connectWs();
}

bootstrap();
