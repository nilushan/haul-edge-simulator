/* global Chart */

const els = {
  conn: document.getElementById('conn'),
  clock: document.getElementById('clock'),
  cycle: document.getElementById('cycle'),
  speed: document.getElementById('s_speed'),
  az: document.getElementById('s_az'),
  gz: document.getElementById('s_gz'),
  gnss: document.getElementById('s_gnss'),
  lidar: document.getElementById('s_lidar'),
  pose: document.getElementById('s_pose'),
  path: document.getElementById('pathCanvas'),
  lidarCv: document.getElementById('lidarCanvas'),
};

const chartDefaults = {
  responsive: true,
  animation: false,
  scales: {
    x: {
      type: 'linear',
      ticks: { color: '#8b9bb0', maxTicksLimit: 8 },
      grid: { color: 'rgba(255,255,255,0.04)' },
    },
    y: {
      ticks: { color: '#8b9bb0' },
      grid: { color: 'rgba(255,255,255,0.06)' },
    },
  },
  plugins: {
    legend: { labels: { color: '#c9d7e8', boxWidth: 12 } },
  },
};

function mkChart(canvasId, datasets, ySuggested) {
  const ctx = document.getElementById(canvasId);
  return new Chart(ctx, {
    type: 'line',
    data: { datasets },
    options: {
      ...chartDefaults,
      scales: {
        ...chartDefaults.scales,
        y: {
          ...chartDefaults.scales.y,
          suggestedMin: ySuggested?.[0],
          suggestedMax: ySuggested?.[1],
        },
      },
      elements: { point: { radius: 0 }, line: { borderWidth: 1.5, tension: 0.15 } },
    },
  });
}

const accelChart = mkChart('chartAccel', [
  { label: 'ax', data: [], borderColor: '#3d9cf0', backgroundColor: 'transparent' },
  { label: 'ay', data: [], borderColor: '#3ecf8e', backgroundColor: 'transparent' },
  { label: 'az', data: [], borderColor: '#e6b450', backgroundColor: 'transparent' },
], [-2, 12]);

const gyroChart = mkChart('chartGyro', [
  { label: 'gx', data: [], borderColor: '#3d9cf0', backgroundColor: 'transparent' },
  { label: 'gy', data: [], borderColor: '#3ecf8e', backgroundColor: 'transparent' },
  { label: 'gz', data: [], borderColor: '#c084fc', backgroundColor: 'transparent' },
], [-0.5, 0.5]);

const gnssChart = mkChart('chartGnss', [
  { label: 'alt m', data: [], borderColor: '#3d9cf0', backgroundColor: 'transparent' },
  {
    label: 'fix',
    data: [],
    borderColor: '#3ecf8e',
    backgroundColor: 'transparent',
    yAxisID: 'y1',
  },
]);
gnssChart.options.scales.y1 = {
  position: 'right',
  min: -0.1,
  max: 1.1,
  ticks: { color: '#8b9bb0', stepSize: 1 },
  grid: { drawOnChartArea: false },
};

function setSeries(chart, seriesList) {
  seriesList.forEach((pts, i) => {
    chart.data.datasets[i].data = pts;
  });
  chart.update('none');
}

function drawPath(odomTail) {
  const c = els.path;
  const ctx = c.getContext('2d');
  const w = c.width;
  const h = c.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0a1018';
  ctx.fillRect(0, 0, w, h);

  if (!odomTail || odomTail.length < 2) return;

  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const p of odomTail) {
    minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
  }
  const pad = 20;
  const dx = Math.max(10, maxX - minX);
  const dy = Math.max(10, maxY - minY);
  const scale = Math.min((w - 2 * pad) / dx, (h - 2 * pad) / dy);

  const sx = (x) => pad + (x - minX) * scale;
  // canvas y down; ENU north up
  const sy = (y) => h - pad - (y - minY) * scale;

  ctx.strokeStyle = '#1e2a38';
  ctx.lineWidth = 1;
  ctx.strokeRect(pad / 2, pad / 2, w - pad, h - pad);

  ctx.beginPath();
  odomTail.forEach((p, i) => {
    const x = sx(p.x);
    const y = sy(p.y);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = '#3d9cf0';
  ctx.lineWidth = 2;
  ctx.stroke();

  const last = odomTail[odomTail.length - 1];
  const lx = sx(last.x);
  const ly = sy(last.y);
  ctx.fillStyle = '#3ecf8e';
  ctx.beginPath();
  ctx.arc(lx, ly, 5, 0, Math.PI * 2);
  ctx.fill();

  // yaw tick
  const len = 14;
  ctx.strokeStyle = '#e6b450';
  ctx.beginPath();
  ctx.moveTo(lx, ly);
  ctx.lineTo(lx + Math.cos(last.yaw) * len, ly - Math.sin(last.yaw) * len);
  ctx.stroke();
}

function heightColor(z) {
  // brown ground → purple high
  const t = Math.max(0, Math.min(1, (z + 0.2) / 2.0));
  const r = Math.round(139 + (192 - 139) * t);
  const g = Math.round(90 + (132 - 90) * t);
  const b = Math.round(43 + (252 - 43) * t);
  return `rgb(${r},${g},${b})`;
}

function drawLidar(lidar) {
  const c = els.lidarCv;
  const ctx = c.getContext('2d');
  const w = c.width;
  const h = c.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0a1018';
  ctx.fillRect(0, 0, w, h);

  if (!lidar || !lidar.xy || !lidar.xy.length) return;

  // body: x forward (up on plot), y left (left on plot)
  const range = 55;
  const scale = Math.min(w, h) / (2 * range);
  const cx = w / 2;
  const cy = h * 0.85;

  // grid
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  for (let r = 10; r <= range; r += 10) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, Math.PI, 0);
    ctx.stroke();
  }

  const xy = lidar.xy;
  const z = lidar.z || [];
  for (let i = 0; i < xy.length; i++) {
    const x = xy[i][0];
    const y = xy[i][1];
    const px = cx - y * scale;
    const py = cy - x * scale;
    ctx.fillStyle = heightColor(z[i] || 0);
    ctx.fillRect(px, py, 2, 2);
  }

  // ego
  ctx.fillStyle = '#3ecf8e';
  ctx.beginPath();
  ctx.moveTo(cx, cy - 10);
  ctx.lineTo(cx - 6, cy + 6);
  ctx.lineTo(cx + 6, cy + 6);
  ctx.closePath();
  ctx.fill();
}

function fmt(n, d = 2) {
  if (n === undefined || n === null || Number.isNaN(n)) return '—';
  return Number(n).toFixed(d);
}

function applyTick(msg) {
  els.clock.textContent = `t = ${fmt(msg.t, 1)} s`;
  els.cycle.textContent = `cycle ${msg.cycle ?? 0}`;

  if (msg.odom) {
    els.speed.textContent = `${fmt(msg.odom.speed, 1)} m/s`;
    els.pose.textContent = `${fmt(msg.odom.x, 1)}, ${fmt(msg.odom.y, 1)} m`;
  }
  if (msg.imu) {
    els.az.textContent = `${fmt(msg.imu.az, 2)} m/s²`;
    els.gz.textContent = `${fmt(msg.imu.gz, 3)} rad/s`;
  }
  if (msg.gnss) {
    const fix = msg.gnss.fix_ok ? 'FIX' : 'NO FIX';
    els.gnss.textContent = `${fix} · ${fmt(msg.gnss.lat, 5)}, ${fmt(msg.gnss.lon, 5)}`;
  }
  if (msg.lidar) {
    els.lidar.textContent = String(msg.lidar.n ?? '—');
    drawLidar(msg.lidar);
  }

  if (msg.imu_tail && msg.imu_tail.length) {
    setSeries(accelChart, [
      msg.imu_tail.map((s) => ({ x: s.t, y: s.ax })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.ay })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.az })),
    ]);
    setSeries(gyroChart, [
      msg.imu_tail.map((s) => ({ x: s.t, y: s.gx })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.gy })),
      msg.imu_tail.map((s) => ({ x: s.t, y: s.gz })),
    ]);
  }
  if (msg.gnss_tail && msg.gnss_tail.length) {
    setSeries(gnssChart, [
      msg.gnss_tail.map((s) => ({ x: s.t, y: s.alt })),
      msg.gnss_tail.map((s) => ({ x: s.t, y: s.fix_ok ? 1 : 0 })),
    ]);
  }
  if (msg.odom_tail) drawPath(msg.odom_tail);
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
      if (msg.type === 'tick' || msg.imu || msg.odom) applyTick(msg);
    } catch (_) { /* ignore */ }
  };
}

async function bootstrap() {
  try {
    const res = await fetch('/api/status');
    const st = await res.json();
    applyTick({
      t: st.t,
      cycle: st.cycle,
      imu: st.latest?.imu,
      gnss: st.latest?.gnss,
      odom: st.latest?.odom,
      lidar: null,
      imu_tail: st.history?.imu || [],
      gnss_tail: st.history?.gnss || [],
      odom_tail: st.history?.odom || [],
    });
    const lid = await fetch('/api/lidar').then((r) => r.json());
    if (lid && lid.xy) drawLidar(lid);
  } catch (_) { /* server may still be starting */ }
  connectWs();
}

bootstrap();
