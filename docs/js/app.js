// =====================================================================================
//  app.js — the page. State, input, the simulation loop, the canvas, the DOM wiring.
//  All of the maths lives in cbf.js; nothing here knows how a barrier works.
// =====================================================================================

import { ARM, REACH, fk, jacobian, barrier, outline, rows, step, clampReach } from './cbf.js';

const DT = 1 / 240;                              // fixed step: αΔt ≤ 1 keeps invariance intact
const WORLD = { cx: 0.20, cy: 0.20, span: 4.0 }; // metres shown across the square canvas
const OBSTACLE_SIZE = { a: 0.25, b: 0.15 };      // the rectangle itself, half-extents in metres
const TRAIL_MAX = 240, TRAIL_EVERY = 8;
const ARROW_PX_PER_MS = 115;                     // pixels per m/s, for the velocity arrows

// Fixed because they are explained, not tuned: the Cartesian gain and the damping length.
const FIXED = { kp: 2.0, lambda: 0.10 };

// The obstacle is a rectangle and stays one. What ε shapes is the FORBIDDEN REGION around
// it: the superellipse that just contains the rectangle. At the corner (a, b) we need
// (a/A)^n + (b/B)^n = 1 with n = 2/ε, so A = 2^(ε/2)·a. ε → 0 hugs the rectangle exactly,
// ε = 1 is the ellipse circumscribing it.
const regionAround = (c, eps) => {
  const k = Math.pow(2, eps / 2);
  return { c, a: OBSTACLE_SIZE.a * k, b: OBSTACLE_SIZE.b * k, eps };
};

const scene = () => ({
  q: [-14 * Math.PI / 180, 55 * Math.PI / 180],
  target: [0.00, 1.55],
  obstacle: regionAround([1.00, 0.88], 0.25),
  drawn:    null,   // the freehand forbidden region — there is none until you draw one
});

const S = {
  ...scene(),
  P: { ...FIXED, alpha: 2.0, vmax: 0.70, eps: 0.25, filterOn: true },
  tool: 'target', running: false, drag: null, trail: [], diag: null, dirty: true, note: '',
};

const $ = (id) => document.getElementById(id);
const canvas = $('c'), ctx = canvas.getContext('2d');
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const expertMode = () => document.body.dataset.mode === 'expert';


// ---- view: world <-> screen, device pixel ratio, theme --------------------------------

let cssW = 600, cssH = 600, scale = 150, T = {};

function resize() {
  const r = canvas.getBoundingClientRect();
  if (!r.width) return;
  cssW = r.width; cssH = r.height;
  scale = Math.min(cssW, cssH) / WORLD.span;
  const dpr = Math.min(window.devicePixelRatio || 1, 2.5);
  const w = Math.round(cssW * dpr), h = Math.round(cssH * dpr);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);   // from here on, all drawing is in CSS pixels
  S.dirty = true; schedule();
}

const sx = (x) => cssW / 2 + scale * (x - WORLD.cx);
const sy = (y) => cssH / 2 - scale * (y - WORLD.cy);

function toWorld(ev) {
  const r = canvas.getBoundingClientRect();
  const s = Math.min(r.width, r.height) / WORLD.span;
  return [(ev.clientX - r.left - r.width / 2) / s + WORLD.cx,
          WORLD.cy - (ev.clientY - r.top - r.height / 2) / s];
}

function readTheme() {
  const cs = getComputedStyle(document.body);
  for (const k of ['bg', 'card', 'fg', 'muted', 'line', 'arm', 'accent', 'ghost', 'region', 'obstacle', 'ok', 'warn'])
    T[k] = cs.getPropertyValue('--' + k).trim();
  S.dirty = true; schedule();
}


// ---- canvas -------------------------------------------------------------------------

const mv = (p) => ctx.moveTo(sx(p[0]), sy(p[1]));
const ln = (p) => ctx.lineTo(sx(p[0]), sy(p[1]));

function stroke(color, width, dash) {
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]);
}

function dot(p, r, fill, edge) {
  ctx.beginPath(); ctx.arc(sx(p[0]), sy(p[1]), r, 0, 7);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (edge) { ctx.strokeStyle = edge; ctx.lineWidth = 2; ctx.stroke(); }
}

function regionPath(reg) {
  const pts = outline(reg);
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) (i ? ln : mv)(pts[i]);
  ctx.closePath();
}

// An arrow in pixel space, from a world point along a world vector (m/s).
function arrow(from, vec, color, dashed) {
  let px = vec[0] * ARROW_PX_PER_MS, py = -vec[1] * ARROW_PX_PER_MS;
  const len = Math.hypot(px, py);
  if (len < 3) return;
  const cap = 0.34 * cssW;
  if (len > cap) { px *= cap / len; py *= cap / len; }
  const x0 = sx(from[0]), y0 = sy(from[1]), x1 = x0 + px, y1 = y0 + py;
  const a = Math.atan2(py, px), head = Math.min(9, Math.hypot(px, py) * 0.4);

  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1);
  stroke(color, dashed ? 1.6 : 2.4, dashed ? [4, 3] : []);

  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - head * Math.cos(a - 0.42), y1 - head * Math.sin(a - 0.42));
  ctx.lineTo(x1 - head * Math.cos(a + 0.42), y1 - head * Math.sin(a + 0.42));
  ctx.closePath();
  if (dashed) { stroke(color, 1.4); } else { ctx.fillStyle = color; ctx.fill(); }
}

function draw() {
  const expert = expertMode();
  ctx.clearRect(0, 0, cssW, cssH);
  ctx.fillStyle = T.card; ctx.fillRect(0, 0, cssW, cssH);

  // reachable workspace
  for (const r of [REACH.inner, REACH.outer]) {
    ctx.beginPath(); ctx.arc(sx(0), sy(0), scale * r, 0, 7);
    stroke(T.line, 1, [3, 4]);
  }

  // Forbidden regions — the h = 0 curve of each rule, drawn the same way whether it wraps
  // the obstacle or the user dragged it out freehand.
  for (const reg of [S.obstacle, S.drawn]) {
    if (!reg) continue;
    const hit = S.diag && S.diag.rows.some((r) => r.reg === reg && r.h < 0);
    regionPath(reg);
    ctx.fillStyle = T.region; ctx.globalAlpha = 0.10; ctx.fill(); ctx.globalAlpha = 1;
    stroke(T.region, hit ? 3 : 2, [7, 5]);
  }

  // The obstacle itself: a real rectangle, solid, sitting inside its forbidden region.
  if (S.obstacle) {
    const c = S.obstacle.c, a = OBSTACLE_SIZE.a, b = OBSTACLE_SIZE.b;
    ctx.beginPath();
    ctx.rect(sx(c[0] - a), sy(c[1] + b), 2 * a * scale, 2 * b * scale);
    ctx.fillStyle = T.obstacle; ctx.fill();
    stroke(T.obstacle, 1.5);
  }

  // live drag preview
  if (S.drag && S.drag.kind === 'region' && S.drag.a > 0) {
    regionPath({ c: S.drag.c, a: S.drag.a, b: S.drag.b, eps: 1 }, 0);
    stroke(T.region, 1.5, [5, 4]);
    ctx.beginPath(); mv([S.drag.c[0] - 0.05, S.drag.c[1]]); ln([S.drag.c[0] + 0.05, S.drag.c[1]]);
    mv([S.drag.c[0], S.drag.c[1] - 0.05]); ln([S.drag.c[0], S.drag.c[1] + 0.05]);
    stroke(T.region, 1);
  }

  // trail
  if (S.trail.length > 1) {
    ctx.beginPath();
    for (let i = 0; i < S.trail.length; i++) (i ? ln : mv)(S.trail[i]);
    ctx.globalAlpha = 0.45; stroke(T.accent, 2); ctx.globalAlpha = 1;
  }

  const { elbow, ee } = fk(S.q);

  // target
  const t = S.target;
  ctx.beginPath(); ctx.arc(sx(t[0]), sy(t[1]), 8, 0, 7); stroke(T.ok, 2);
  ctx.beginPath();
  mv([t[0], t[1]]); ctx.moveTo(sx(t[0]) - 13, sy(t[1])); ctx.lineTo(sx(t[0]) + 13, sy(t[1]));
  ctx.moveTo(sx(t[0]), sy(t[1]) - 13); ctx.lineTo(sx(t[0]), sy(t[1]) + 13);
  stroke(T.ok, 1);

  // arm
  ctx.lineCap = 'round';
  ctx.beginPath(); mv([0, 0]); ln(elbow); ln(ee); stroke(T.arm, 7);
  ctx.fillStyle = T.arm; ctx.fillRect(sx(0) - 7, sy(0) - 5, 14, 10);
  dot(elbow, 5, T.card, T.arm);
  dot(ee, 7, T.card);                    // halo, so it stays readable over a region fill
  dot(ee, 5, T.accent);

  // what the plan wanted vs what the filter allows, both as end-effector velocity
  const d = S.diag;
  if (d) {
    const J = jacobian(S.q);
    const vel = (u) => [J[0] * u[0] + J[1] * u[1], J[2] * u[0] + J[3] * u[1]];
    const v = vel(d.u), vn = vel(d.uNom);
    if (Math.hypot(vn[0] - v[0], vn[1] - v[1]) > 0.05 * Math.max(0.05, Math.hypot(vn[0], vn[1])))
      arrow(ee, vn, T.ghost, true);
    arrow(ee, v, T.accent, false);

    if (expert && d.rows.length) {
      const r = d.rows.reduce((a, b) => (b.h < a.h ? b : a));
      const gn = Math.hypot(r.gx, r.gy);
      if (gn > 1e-9 && r.h < 0.5) ruler(ee, r, v, gn);
    }
  }
}

// THE APPROACH RULER — the constraint, drawn.
// A fixed track from the end effector along n̂ = −∇h/‖∇h‖ (into the region). The tick on it
// sits at the fastest closing speed the rule allows, α·h/‖∇h‖; the bar shows the closing
// speed actually being used. The bar can never pass the tick, and as the arm arrives the
// tick slides back into the end effector and the bar is crushed to nothing.
function ruler(p, r, v, gn) {
  const TRACK = 92;
  const dx = -r.gx / gn, dy = r.gy / gn;                 // into the region, in screen axes
  const allowed = Math.max(0, S.P.alpha * r.h / gn);
  const actual = Math.max(0, -(r.gx * v[0] + r.gy * v[1]) / gn);
  const ap = clamp(allowed * ARROW_PX_PER_MS, 0, TRACK);
  const bp = clamp(actual * ARROW_PX_PER_MS, 0, TRACK);
  const x0 = sx(p[0]), y0 = sy(p[1]);

  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x0 + dx * TRACK, y0 + dy * TRACK);
  stroke(T.muted, 1, [2, 4]);

  if (bp > 1.5) {
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x0 + dx * bp, y0 + dy * bp);
    stroke(T.warn, 5);
  }

  const tx = x0 + dx * ap, ty = y0 + dy * ap;            // the cap tick
  ctx.beginPath(); ctx.moveTo(tx - dy * 8, ty + dx * 8); ctx.lineTo(tx + dy * 8, ty - dx * 8);
  stroke(T.warn, 2.5);

  // label on the far side of the end effector, so it never sits inside the region
  ctx.font = '600 11px system-ui, sans-serif';
  ctx.textAlign = dx < 0 ? 'left' : 'right';
  ctx.textBaseline = 'middle';
  const lx = x0 - dx * 15, ly = y0 - dy * 15;
  const label = '\u2264 ' + allowed.toFixed(2) + ' m/s';
  ctx.lineWidth = 3.5; ctx.strokeStyle = T.card; ctx.strokeText(label, lx, ly);
  ctx.fillStyle = T.warn; ctx.fillText(label, lx, ly);
  ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
}


// ---- the loop -------------------------------------------------------------------------

let raf = 0, last = 0, acc = 0, tick = 0;
const schedule = () => { if (!raf) raf = requestAnimationFrame(frame); };

function frame(ts) {
  raf = 0;
  const dt = last ? Math.min((ts - last) / 1000, 0.05) : 0;
  last = ts;

  if (S.running) {
    acc += dt;
    let n = 0;
    while (acc >= DT && n < 20) {
      S.diag = step(S, DT); acc -= DT; n++;
      if (++tick % TRAIL_EVERY === 0) {
        S.trail.push(fk(S.q).ee);
        if (S.trail.length > TRAIL_MAX) S.trail.shift();
      }
    }
    if (n === 20) acc = 0;
  } else {
    S.diag = step(S, 0);   // evaluate without moving, so sliders show their effect live
    acc = 0;
  }

  draw();
  hud();
  S.dirty = false;
  if (S.running || S.dirty) schedule();
}


// ---- readouts and banner ----------------------------------------------------------------

const fmt = (v) => v.toFixed(Math.abs(v) >= 1 ? 1 : 2);

function hud() {
  const d = S.diag;
  const chip = $('chip');
  const hs = d ? d.rows.map((r) => r.h) : [];
  const hmin = hs.length ? Math.min(...hs) : null;

  chip.innerHTML = hmin === null
    ? 'clearance <b>h</b> &mdash; no region drawn'
    : `clearance <b>h</b> = ${hmin.toFixed(3)} m`;
  chip.dataset.level = hmin === null ? '' : hmin <= 0 ? 'bad' : hmin < 0.05 ? 'near' : '';

  if (expertMode() && d) {
    const du = Math.hypot(d.u[0] - d.uNom[0], d.u[1] - d.uNom[1]);
    $('r-h').textContent = hmin === null ? '—' : hmin.toFixed(4) + ' m';
    $('r-du').textContent = du.toFixed(4) + ' rad/s';
    $('r-act').textContent = d.active.length
      ? d.active.map((i) => (d.rows[i].reg === S.obstacle ? 'obstacle' : 'drawn region')).join(', ')
      : 'none';
  }

  let msg = null, kind = '';
  if (d && !d.ok) msg = 'No command satisfies every rule at once — the arm has stopped.';
  else if (S.note) msg = S.note;
  else if (d && d.dist < 0.01) { msg = 'Target reached.'; kind = 'ok'; }
  else if (S.running && d && Math.hypot(d.u[0], d.u[1]) < 2e-3)
    msg = 'Stalled — a safety filter guarantees safety, not progress.';

  const b = $('banner');
  b.hidden = !msg;
  if (msg) { b.textContent = msg; b.dataset.kind = kind; }
}


// ---- input --------------------------------------------------------------------------------

const HINTS = {
  target:   'Click or drag on the canvas to move the target.',
  obstacle: 'Click to place the obstacle — a rectangle of fixed size.',
  region:   'Press and drag: the press point is the centre, the drag sets the two radii.',
};

function setTool(tool) {
  S.tool = tool; S.note = '';
  document.querySelectorAll('[data-tool]').forEach((b) =>
    b.setAttribute('aria-checked', String(b.dataset.tool === tool)));
  $('hint').textContent = HINTS[tool];
  S.dirty = true; schedule();
}

function setTarget(p) {
  const { p: q, moved } = clampReach(p);
  S.target = q;
  S.note = moved ? 'That point is outside the arm’s reach; the target was moved to the nearest reachable one.' : '';
}

canvas.addEventListener('pointerdown', (e) => {
  canvas.setPointerCapture(e.pointerId);
  const p = toWorld(e);
  if (S.tool === 'target') { S.drag = { kind: 'target' }; setTarget(p); }
  else if (S.tool === 'obstacle') { S.obstacle = regionAround(p, S.P.eps); S.note = ''; }
  else { S.drag = { kind: 'region', c: p, a: 0, b: 0, moved: false }; S.note = ''; }
  S.dirty = true; schedule();
});

canvas.addEventListener('pointermove', (e) => {
  if (!S.drag) return;
  const p = toWorld(e);
  if (S.drag.kind === 'target') setTarget(p);
  else {
    const dx = Math.abs(p[0] - S.drag.c[0]), dy = Math.abs(p[1] - S.drag.c[1]);
    if (Math.max(dx, dy) > 0.04) S.drag.moved = true;
    S.drag.a = clamp(dx, 0.03, 0.9); S.drag.b = clamp(dy, 0.03, 0.9);
  }
  S.dirty = true; schedule();
});

function endDrag(commit) {
  if (S.drag && S.drag.kind === 'region' && commit && S.drag.moved)
    S.drawn = { c: S.drag.c, a: S.drag.a, b: S.drag.b, eps: 1 };
  S.drag = null; S.dirty = true; schedule();
}
canvas.addEventListener('pointerup', () => endDrag(true));
canvas.addEventListener('pointercancel', () => endDrag(false));

document.querySelectorAll('[data-tool]').forEach((b) => b.addEventListener('click', () => setTool(b.dataset.tool)));

$('run').addEventListener('click', () => {
  S.running = !S.running;
  $('run').innerHTML = S.running ? '&#10074;&#10074; Pause' : '&#9654; Run';
  last = 0; acc = 0; S.dirty = true; schedule();
});

$('reset').addEventListener('click', () => {
  Object.assign(S, scene());
  S.trail = []; S.running = false; S.note = ''; S.drag = null;
  $('run').innerHTML = '&#9654; Run';
  S.obstacle = regionAround(S.obstacle.c, S.P.eps);   // keep whatever the slider shows
  S.dirty = true; schedule();
});

$('clear').addEventListener('click', () => {
  S.obstacle = null; S.drawn = null; S.trail = []; S.note = '';
  S.dirty = true; schedule();
});

$('filterOn').addEventListener('change', (e) => {
  S.P.filterOn = e.target.checked; S.dirty = true; schedule();
});

document.querySelectorAll('[data-param]').forEach((el) => {
  const read = () => {
    if (!el.dataset.log) return +el.value;
    const [lo, hi] = el.dataset.log.split(',').map(Number);
    return lo * Math.pow(hi / lo, +el.value / 100);
  };
  const apply = () => {
    const v = read();
    S.P[el.dataset.param] = v;
    if (el.dataset.param === 'eps' && S.obstacle) S.obstacle = regionAround(S.obstacle.c, v);
    $('v-' + el.dataset.param).textContent = fmt(v) + (el.dataset.unit ? ' ' + el.dataset.unit : '');
    S.dirty = true; schedule();
  };
  el.addEventListener('input', apply);
  apply();
});

for (const m of ['simple', 'expert']) {
  $('m-' + m).addEventListener('click', () => {
    document.body.dataset.mode = m;
    $('m-simple').setAttribute('aria-pressed', String(m === 'simple'));
    $('m-expert').setAttribute('aria-pressed', String(m === 'expert'));
    resize();     // the canvas column changes width when the panel grows
  });
}

document.addEventListener('visibilitychange', () => { last = 0; acc = 0; schedule(); });
new ResizeObserver(resize).observe(canvas);
if (window.matchMedia) window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readTheme);

setTool('target');
readTheme();
resize();
