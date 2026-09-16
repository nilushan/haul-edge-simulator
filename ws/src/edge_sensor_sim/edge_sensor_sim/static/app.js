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
};

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
scene.fog = new THREE.Fog(0x1a222c, 60, 180);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 500);
camera.position.set(-20, 14, 24);

// Orbit around a moving target (truck). In follow mode we slide camera+target together.
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.maxPolarAngle = Math.PI * 0.48;
controls.minDistance = 6;
controls.maxDistance = 200;
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
const ground = new THREE.Mesh(
  new THREE.CircleGeometry(70, 64),
  new THREE.MeshStandardMaterial({
    color: 0x2a3340,
    roughness: 1,
    metalness: 0,
    transparent: true,
    opacity: 0.55,
  }),
);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.02;
ground.receiveShadow = true;
scene.add(ground);

const grid = new THREE.GridHelper(120, 24, 0x3d4a58, 0x2a3340);
grid.position.y = 0.01;
grid.material.opacity = 0.35;
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
const MAP_MAX = 60000;
const VOXEL = 0.35; // metres
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
    size: 0.28,
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.92,
  }),
);
scene.add(mapCloud);

// Current-scan highlight (body-frame points this tick)
const scanMax = 4000;
const scanPositions = new Float32Array(scanMax * 3);
const scanColors = new Float32Array(scanMax * 3);
const scanGeo = new THREE.BufferGeometry();
scanGeo.setAttribute('position', new THREE.BufferAttribute(scanPositions, 3));
scanGeo.setAttribute('color', new THREE.BufferAttribute(scanColors, 3));
scanGeo.setDrawRange(0, 0);
const scanCloud = new THREE.Points(
  scanGeo,
  new THREE.PointsMaterial({
    size: 0.4,
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 1,
  }),
);
scene.add(scanCloud);

let lastLidarT = -1;
let mapCount = 0;

function heightColor(z, out, i) {
  const t = Math.max(0, Math.min(1, (z + 0.15) / 1.8));
  out[i] = (139 + (192 - 139) * t) / 255;
  out[i + 1] = (90 + (132 - 90) * t) / 255;
  out[i + 2] = (43 + (252 - 43) * t) / 255;
}

function rebuildMapGeometry() {
  let i = 0;
  for (const v of mapVoxels.values()) {
    if (i >= MAP_MAX) break;
    const j = i * 3;
    mapPositions[j] = v.x;
    mapPositions[j + 1] = v.y;
    mapPositions[j + 2] = v.z;
    mapColors[j] = v.r;
    mapColors[j + 1] = v.g;
    mapColors[j + 2] = v.b;
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
  mapGeo.setDrawRange(0, 0);
  lastLidarT = -1;
}

/**
 * Body LiDAR frame is ROS-style: +X forward, +Y left, +Z up.
 * Truck mesh is Three-style: +X forward, +Y up, +Z right.
 * Convert body → truck local Three, then truck.matrixWorld → world.
 * This keeps the map glued to the same pose as the rendered vehicle.
 */
const _local = new THREE.Vector3();
const _world = new THREE.Vector3();

function bodyRosToTruckLocal(bx, byLeft, bz, out) {
  // ROS (x,y,z)=(fwd,left,up) → Three local (x,y,z)=(fwd,up,right)=(bx,bz,-by)
  out.set(bx, bz, -byLeft);
  return out;
}

function pruneMapFarFrom(egoThree, maxKeep = MAP_MAX) {
  if (mapVoxels.size <= maxKeep) return;
  const ranked = [];
  for (const [k, v] of mapVoxels) {
    const dx = v.x - egoThree.x;
    const dy = v.y - egoThree.y;
    const dz = v.z - egoThree.z;
    ranked.push([k, dx * dx + dy * dy + dz * dz]);
  }
  ranked.sort((a, b) => b[1] - a[1]); // farthest first
  const excess = mapVoxels.size - maxKeep;
  for (let i = 0; i < excess; i++) mapVoxels.delete(ranked[i][0]);
}

function appendLidarToMap(lidar, odom) {
  if (!lidar?.xy?.length || !odom) return;
  // Only integrate when a new scan timestamp arrives
  if (lidar.t != null && Math.abs(lidar.t - lastLidarT) < 1e-6) return;
  lastLidarT = lidar.t ?? lastLidarT;

  truck.updateMatrixWorld(true);
  const invV = 1.0 / VOXEL;
  const col = [0, 0, 0];
  const n = lidar.xy.length;
  const step = Math.max(1, Math.floor(n / 2000));

  for (let i = 0; i < n; i += step) {
    const bx = lidar.xy[i][0]; // forward
    const by = lidar.xy[i][1]; // left
    const bz = (lidar.z && lidar.z[i]) || 0;
    if (!(bx > 0.5)) continue; // only forward returns

    bodyRosToTruckLocal(bx, by, bz, _local);
    _world.copy(_local).applyMatrix4(truck.matrixWorld);

    const ix = Math.floor(_world.x * invV);
    const iy = Math.floor(_world.y * invV);
    const iz = Math.floor(_world.z * invV);
    const key = `${ix},${iy},${iz}`;

    heightColor(bz, col, 0);
    const prev = mapVoxels.get(key);
    if (!prev || bz >= (prev.bz ?? -999)) {
      mapVoxels.set(key, {
        x: (ix + 0.5) * VOXEL,
        y: (iy + 0.5) * VOXEL,
        z: (iz + 0.5) * VOXEL,
        r: col[0],
        g: col[1],
        b: col[2],
        bz,
      });
    }
  }

  // Drop farthest voxels only (keep corridor behind + ahead near ego)
  const ego = enuToThree(odom.x, odom.y, odom.z);
  pruneMapFarFrom(ego, MAP_MAX);
  rebuildMapGeometry();
}

let camMode = 'follow'; // follow | top | free
let lastOdom = null;
let followDist = 26;
let followHeight = 12;
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
    camera.position.copy(enuToThree(lastOdom.x, lastOdom.y, lastOdom.z + 55));
    controls.update();
  }
}

// Wheel in follow mode: change chase distance (and still allow OrbitControls zoom)
canvas.addEventListener('wheel', (e) => {
  if (camMode !== 'follow') return;
  followDist = Math.min(120, Math.max(8, followDist + (e.deltaY > 0 ? 2.5 : -2.5)));
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
  if (!lidar?.xy?.length || !odom) return;

  // Pose truck first so matrixWorld matches this odom (caller order also updates truck)
  updateTruck(odom);

  // 1) Append this scan into the persistent world map (aligned to truck mesh)
  appendLidarToMap(lidar, odom);

  // 2) Current scan highlight — same transform as map
  truck.updateMatrixWorld(true);
  const n = Math.min(lidar.xy.length, scanMax);
  const col = [0, 0, 0];
  let w = 0;
  for (let i = 0; i < n; i++) {
    const bx = lidar.xy[i][0];
    const by = lidar.xy[i][1];
    const bz = (lidar.z && lidar.z[i]) || 0;
    if (!(bx > 0.5)) continue;
    bodyRosToTruckLocal(bx, by, bz, _local);
    _world.copy(_local).applyMatrix4(truck.matrixWorld);
    const j = w * 3;
    scanPositions[j] = _world.x;
    scanPositions[j + 1] = _world.y;
    scanPositions[j + 2] = _world.z;
    heightColor(bz, col, 0);
    scanColors[j] = Math.min(1, col[0] * 0.45 + 0.5);
    scanColors[j + 1] = Math.min(1, col[1] * 0.45 + 0.55);
    scanColors[j + 2] = Math.min(1, col[2] * 0.35 + 0.7);
    w += 1;
  }
  scanGeo.setDrawRange(0, w);
  scanGeo.attributes.position.needsUpdate = true;
  scanGeo.attributes.color.needsUpdate = true;
}

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
    const offset = new THREE.Vector3().subVectors(camera.position, controls.target);
    controls.target.lerp(target, 0.2);
    // Keep camera above target
    const up = new THREE.Vector3(0, Math.max(30, offset.length()), 0.01);
    camera.position.lerp(target.clone().add(up), 0.15);
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

  els.clock.textContent = `t = ${fmt(msg.t, 2)} s`;
  els.cycle.textContent = `cycle ${msg.cycle ?? 0}`;
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
  if (msg.lidar && msg.odom) {
    updateLidar(msg.lidar, msg.odom);
  } else if (msg.odom) {
    updateTruck(msg.odom);
  }
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
    } catch (_) { /* ignore */ }
  };
}

async function bootstrap() {
  try {
    const st = await fetch('/api/status').then((r) => r.json());
    const lid = await fetch('/api/lidar').then((r) => r.json()).catch(() => null);
    applyTickFull({
      t: st.t,
      cycle: st.cycle,
      imu: st.latest?.imu,
      gnss: st.latest?.gnss,
      odom: st.latest?.odom,
      lidar: lid,
      imu_tail: st.history?.imu || [],
      odom_tail: st.history?.odom || [],
      gnss_tail: st.history?.gnss || [],
    });
  } catch (_) { /* starting */ }
  connectWs();
}

bootstrap();
