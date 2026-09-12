// =====================================================================================
//  app.js — the page. State, input, the simulation loop, the two canvases, the wiring.
//  All of the maths lives in cbf.js; nothing here knows how a barrier works.
// =====================================================================================

import { ARMS, reach, fk, jacobianAt, outline, classK, rows, step, clampReach } from './cbf.js';

const DT = 1 / 240;                              // fixed step: α·Δt ≤ 1 keeps invariance intact
const WORLD = { cx: 0.20, cy: 0.20, span: 4.0 }; // metres shown across the square canvas
const OBSTACLE_SIZE = { a: 0.25, b: 0.15 };      // the rectangle itself, half-extents in metres
const TRAIL_MAX = 240, TRAIL_EVERY = 8;
const ARROW_PX_PER_MS = 115;                     // pixels per m/s, for the velocity arrows

// Fixed because they are explained, not tuned.
const FIXED = { kp: 2.0, lambda: 0.10, selfGap: 0.07 };

// A gently curled start pose per joint count. All three put the end effector at the same
// point, so the example scene reads identically however many joints you add.
const POSE = {
  2: [-0.2443, 0.9601],
  3: [-0.2894, 0.6113, 0.6113],
  4: [-0.3020, 0.4526, 0.4526, 0.4526],
};

// The obstacle is a rectangle and stays one. What ε shapes is the FORBIDDEN REGION around
// it: the superellipse that just contains the rectangle. At the corner (a, b) we need
// (a/A)^n + (b/B)^n = 1 with n = 2/ε, so A = 2^(ε/2)·a. ε → 0 hugs the rectangle exactly,
// ε = 1 is the ellipse circumscribing it.
const regionAround = (c, eps) => {
  const k = Math.pow(2, eps / 2);
  return { c, a: OBSTACLE_SIZE.a * k, b: OBSTACLE_SIZE.b * k, eps, tag: 'obstacle' };
};

const scene = (n, eps) => ({
  L: ARMS[n].slice(),
  q: POSE[n].slice(),
  target: [0.00, 1.55],
  obstacle: regionAround([1.00, 0.88], eps),
  drawn: null,                                   // the freehand region — none until you draw one
});

const S = {
  ...scene(2, 0.25),
  P: { ...FIXED, alpha: 2.0, vmax: 0.70, eps: 0.25, expo: 1.0,
       filterOn: true, wholeArm: false, selfAvoid: false },
  tool: 'target', running: false, drag: null, trail: [], diag: null, dirty: true, note: '',
};

const $ = (id) => document.getElementById(id);
const canvas = $('c'), ctx = canvas.getContext('2d');
const plot = $('p'), pctx = plot.getContext('2d');
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const expertMode = () => document.body.dataset.mode === 'expert';


// ---- view: world <-> screen, device pixel ratio, theme --------------------------------

let cssW = 600, cssH = 600, scale = 150, pW = 600, pH = 140, T = {};

function fit(el, c2) {
  const r = el.getBoundingClientRect();
  if (!r.width) return null;
  const dpr = Math.min(window.devicePixelRatio || 1, 2.5);
  const w = Math.round(r.width * dpr), h = Math.round(r.height * dpr);
  if (el.width !== w || el.height !== h) { el.width = w; el.height = h; }
  c2.setTransform(dpr, 0, 0, dpr, 0, 0);         // from here on, all drawing is in CSS pixels
  return r;
}

function resize() {
  const r = fit(canvas, ctx);
  if (r) { cssW = r.width; cssH = r.height; scale = Math.min(cssW, cssH) / WORLD.span; }
  const rp = fit(plot, pctx);
  if (rp) { pW = rp.width; pH = rp.height; }
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
  for (const k of ['bg', 'card', 'fg', 'muted', 'line', 'arm', 'accent',
                   'ghost', 'region', 'obstacle', 'ok', 'warn'])
    T[k] = cs.getPropertyValue('--' + k.replace(/[A-Z]/g, (m) => '-' + m.toLowerCase())).trim();
  S.dirty = true; schedule();
}


// ---- small drawing helpers -------------------------------------------------------------

const mv = (p) => ctx.moveTo(sx(p[0]), sy(p[1]));
const ln = (p) => ctx.lineTo(sx(p[0]), sy(p[1]));

function stroke(color, width, dash, c2 = ctx) {
  c2.strokeStyle = color; c2.lineWidth = width;
  c2.setLineDash(dash || []); c2.stroke(); c2.setLineDash([]);
}

function dot(p, r, fill, edge) {
  ctx.beginPath(); ctx.arc(sx(p[0]), sy(p[1]), r, 0, 7);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (edge) { ctx.strokeStyle = edge; ctx.lineWidth = 1.8; ctx.stroke(); }
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
  if (dashed) stroke(color, 1.4); else { ctx.fillStyle = color; ctx.fill(); }
}


// ---- the robot --------------------------------------------------------------------------

function drawArm(pts) {
  const n = pts.length - 1;
  ctx.lineCap = 'round'; ctx.lineJoin = 'round';

  // links: one solid stroke each, tapering from base to tip
  for (let i = 0; i < n; i++) {
    ctx.beginPath(); mv(pts[i]); ln(pts[i + 1]);
    stroke(T.arm, 9 - 3 * (n === 1 ? 0 : i / (n - 1)));
  }

  // base: a small pedestal, drawn over the first link so it reads as its own block
  const bx = sx(0), by = sy(0);
  ctx.beginPath();
  ctx.moveTo(bx - 8, by - 1); ctx.lineTo(bx + 8, by - 1);
  ctx.lineTo(bx + 14, by + 11); ctx.lineTo(bx - 14, by + 11);
  ctx.closePath();
  ctx.fillStyle = T.arm; ctx.fill();

  // joints: a light dot punched out of the link
  for (let i = 0; i < n; i++) dot(pts[i], i === 0 ? 3.4 : 2.8, T.card);

  // end effector: two jaws and a dot, pointing the way the last link points
  const a = pts[n - 1], b = pts[n];
  ctx.save();
  ctx.translate(sx(b[0]), sy(b[1]));
  ctx.rotate(Math.atan2(sy(b[1]) - sy(a[1]), sx(b[0]) - sx(a[0])));
  ctx.beginPath();
  ctx.moveTo(0, -5.5); ctx.lineTo(6, -5.5);
  ctx.moveTo(0, 5.5); ctx.lineTo(6, 5.5);
  stroke(T.arm, 2.6);
  ctx.restore();
  dot(b, 3.2, T.accent);
}


// ---- the main canvas ---------------------------------------------------------------------

function draw() {
  const expert = expertMode();
  ctx.clearRect(0, 0, cssW, cssH);
  ctx.fillStyle = T.card; ctx.fillRect(0, 0, cssW, cssH);

  const { inner, outer } = reach(S.L);
  for (const r of [inner, outer]) {
    if (r <= 0) continue;
    ctx.beginPath(); ctx.arc(sx(0), sy(0), scale * r, 0, 7);
    stroke(T.line, 1, [3, 4]);
  }

  // Forbidden regions — the h = 0 curve of each rule, drawn the same way whether it wraps
  // the obstacle or the user dragged it out freehand.
  for (const reg of [S.obstacle, S.drawn]) {
    if (!reg) continue;
    const hit = S.diag && S.diag.rows.some((r) => r.tag === reg.tag && r.h < 0);
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
    regionPath({ c: S.drag.c, a: S.drag.a, b: S.drag.b, eps: 1 });
    stroke(T.region, 1.5, [5, 4]);
    ctx.beginPath(); mv([S.drag.c[0] - 0.05, S.drag.c[1]]); ln([S.drag.c[0] + 0.05, S.drag.c[1]]);
    mv([S.drag.c[0], S.drag.c[1] - 0.05]); ln([S.drag.c[0], S.drag.c[1] + 0.05]);
    stroke(T.region, 1);
  }

  if (S.trail.length > 1) {
    ctx.beginPath();
    for (let i = 0; i < S.trail.length; i++) (i ? ln : mv)(S.trail[i]);
    ctx.globalAlpha = 0.45; stroke(T.accent, 2); ctx.globalAlpha = 1;
  }

  const pts = fk(S.q, S.L), ee = pts[pts.length - 1];

  // target
  const t = S.target;
  ctx.beginPath(); ctx.arc(sx(t[0]), sy(t[1]), 8, 0, 7); stroke(T.ok, 2);
  ctx.beginPath();
  ctx.moveTo(sx(t[0]) - 13, sy(t[1])); ctx.lineTo(sx(t[0]) + 13, sy(t[1]));
  ctx.moveTo(sx(t[0]), sy(t[1]) - 13); ctx.lineTo(sx(t[0]), sy(t[1]) + 13);
  stroke(T.ok, 1);

  drawArm(pts);

  const d = S.diag;
  if (d) {
    // what the plan wanted vs what the filter allows, both as end-effector velocity
    const { J } = jacobianAt(pts, S.q.length, 1);
    const vel = (u) => [J.reduce((s, c, i) => s + c[0] * u[i], 0),
                        J.reduce((s, c, i) => s + c[1] * u[i], 0)];
    const v = vel(d.u), vn = vel(d.uNom);
    if (Math.hypot(vn[0] - v[0], vn[1] - v[1]) > 0.05 * Math.max(0.05, Math.hypot(vn[0], vn[1])))
      arrow(ee, vn, T.ghost, true);
    arrow(ee, v, T.accent, false);

    if (expert) {
      // every guarded point, and the pair that a self-collision row is watching
      for (const r of d.rows) {
        if (r.pair) {
          ctx.beginPath(); mv(r.pair[0]); ln(r.pair[1]);
          stroke(r.h < 0 ? T.region : T.warn, 1.4, [3, 3]);
        }
        if (r.at) dot(r.at, 3, null, r.h < 0 ? T.region : T.muted);
      }
      const crit = d.rows.length ? d.rows.reduce((a, b) => (b.h < a.h ? b : a)) : null;
      if (crit && crit.at && crit.h < 0.5) {
        const gn = Math.hypot(crit.gx, crit.gy);
        if (gn > 1e-9) ruler(crit.at, crit, vel(d.u), gn);
      }
    }
  }
}

// THE APPROACH RULER — the constraint, drawn.
// A fixed track from the guarded point along n̂ = −∇h/‖∇h‖ (into the region). The tick on
// it sits at the fastest closing speed the rule allows, α(h)/‖∇h‖; the bar shows the
// closing speed actually being used. The bar can never pass the tick, and as the arm
// arrives the tick slides back into the point and the bar is crushed to nothing.
function ruler(p, r, v, gn) {
  const TRACK = 92;
  const dx = -r.gx / gn, dy = r.gy / gn;
  const allowed = Math.max(0, classK(r.h, S.P) / gn);
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
  const tx = x0 + dx * ap, ty = y0 + dy * ap;
  ctx.beginPath(); ctx.moveTo(tx - dy * 8, ty + dx * 8); ctx.lineTo(tx + dy * 8, ty - dx * 8);
  stroke(T.warn, 2.5);

  ctx.font = '600 11px system-ui, sans-serif';
  ctx.textAlign = dx < 0 ? 'left' : 'right';
  ctx.textBaseline = 'middle';
  const label = '≤ ' + allowed.toFixed(2) + ' m/s';
  ctx.lineWidth = 3.5; ctx.strokeStyle = T.card;
  ctx.strokeText(label, x0 - dx * 15, y0 - dy * 15);
  ctx.fillStyle = T.warn; ctx.fillText(label, x0 - dx * 15, y0 - dy * 15);
  ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
}


// ---- the CBF plot: the rule itself, with the live state on it ----------------------------
//
//  x is the clearance h, y is its rate of change ḣ. The curve is ḣ = −α(h): the fastest
//  h is allowed to fall. Everything above it is permitted, and the dot — the arm, right
//  now — can never be below it. At h = 0 the curve passes through the origin, which is
//  why the arm is turned along a boundary instead of through it.

function drawPlot() {
  const P = S.P;
  const X0 = -0.15, X1 = 1.0;
  const Y = clamp(Math.max(P.alpha, 1.0), 1.0, 4);
  const L = 34, R = 10, TP = 12, B = 30;                 // margins
  const px = (h) => L + (h - X0) / (X1 - X0) * (pW - L - R);
  const py = (v) => TP + (Y - v) / (2 * Y) * (pH - TP - B);

  pctx.clearRect(0, 0, pW, pH);
  pctx.fillStyle = T.card; pctx.fillRect(0, 0, pW, pH);

  // the region that is not allowed: below the curve
  pctx.beginPath();
  pctx.moveTo(px(X0), py(-Y));
  for (let i = 0; i <= 160; i++) {
    const h = X0 + (X1 - X0) * i / 160;
    pctx.lineTo(px(h), py(clamp(-classK(h, P), -Y, Y)));
  }
  pctx.lineTo(px(X1), py(-Y)); pctx.closePath();
  pctx.fillStyle = T.region; pctx.globalAlpha = 0.10; pctx.fill(); pctx.globalAlpha = 1;

  // inside the region, h < 0
  pctx.fillStyle = T.region; pctx.globalAlpha = 0.07;
  pctx.fillRect(px(X0), py(Y), px(0) - px(X0), py(-Y) - py(Y)); pctx.globalAlpha = 1;

  // axes
  pctx.beginPath(); pctx.moveTo(px(X0), py(0)); pctx.lineTo(px(X1), py(0)); stroke(T.line, 1, [], pctx);
  pctx.beginPath(); pctx.moveTo(px(0), py(Y)); pctx.lineTo(px(0), py(-Y)); stroke(T.line, 1, [], pctx);

  // the rule
  pctx.beginPath();
  for (let i = 0; i <= 240; i++) {
    const h = X0 + (X1 - X0) * i / 240;
    const v = clamp(-classK(h, P), -Y, Y);
    i ? pctx.lineTo(px(h), py(v)) : pctx.moveTo(px(h), py(v));
  }
  stroke(T.region, 2, [], pctx);

  // the arm, right now
  const d = S.diag;
  const crit = d && d.rows.length ? d.rows.reduce((a, b) => (b.h < a.h ? b : a)) : null;
  if (crit) {
    const hdot = crit.a.reduce((s, ai, i) => s + ai * d.u[i], 0);
    const X = px(clamp(crit.h, X0, X1)), Yp = py(clamp(hdot, -Y, Y));
    pctx.beginPath(); pctx.moveTo(X, py(clamp(-classK(crit.h, P), -Y, Y))); pctx.lineTo(X, Yp);
    stroke(T.muted, 1, [2, 3], pctx);
    pctx.beginPath(); pctx.arc(X, Yp, 5.5, 0, 7);
    pctx.fillStyle = T.card; pctx.fill(); stroke(T.accent, 2.4, [], pctx);
  }

  // labels
  pctx.font = '10.5px system-ui, sans-serif';
  pctx.fillStyle = T.muted; pctx.textBaseline = 'middle';
  pctx.textAlign = 'right';
  pctx.fillText('+' + Y.toFixed(1), L - 5, py(Y * 0.92));
  pctx.fillText('0', L - 5, py(0));
  pctx.fillText('−' + Y.toFixed(1), L - 5, py(-Y * 0.92));
  pctx.textAlign = 'center'; pctx.textBaseline = 'top';
  for (const h of [0, 0.5, 1.0]) pctx.fillText(h.toFixed(1), px(h), pH - B + 5);
  pctx.fillText('clearance  h  (m)', (px(X0) + px(X1)) / 2, pH - B + 17);
  pctx.save();
  pctx.translate(11, (py(Y) + py(-Y)) / 2); pctx.rotate(-Math.PI / 2);
  pctx.textBaseline = 'middle'; pctx.fillText('ḣ  (m/s)', 0, 0);
  pctx.restore();

  // name the curve alongside it, two thirds along, so it never lands on an axis tick
  const hl = 0.62;
  pctx.textAlign = 'left'; pctx.textBaseline = 'bottom';
  pctx.fillStyle = T.region;
  pctx.fillText('ḣ = −α(h)', px(hl) + 6, py(clamp(-classK(hl, P), -Y, Y)) - 5);
  pctx.textBaseline = 'top';
  pctx.fillStyle = T.muted;
  pctx.fillText('allowed', px(0.05), py(Y) + 3);
  pctx.fillText('forbidden', px(X0) + 3, py(-Y * 0.62));
  pctx.textBaseline = 'alphabetic';
}


// ---- the loop -----------------------------------------------------------------------------

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
        const p = fk(S.q, S.L);
        S.trail.push(p[p.length - 1]);
        if (S.trail.length > TRAIL_MAX) S.trail.shift();
      }
    }
    if (n === 20) acc = 0;
  } else {
    S.diag = step(S, 0);   // evaluate without moving, so sliders show their effect live
    acc = 0;
  }

  draw();
  drawPlot();
  hud();
  S.dirty = false;
  if (S.running || S.dirty) schedule();
}


// ---- readouts and banner --------------------------------------------------------------------

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
    const du = Math.hypot(...d.u.map((x, i) => x - d.uNom[i]));
    $('r-h').textContent = hmin === null ? '—' : hmin.toFixed(4) + ' m';
    $('r-du').textContent = du.toFixed(4) + ' rad/s';
    $('r-rows').textContent = String(d.rows.length);
    $('r-act').textContent = d.active.length
      ? [...new Set(d.active.map((i) => d.rows[i].tag))].join(', ') : 'none';
  }

  let msg = null, kind = '';
  if (d && !d.ok) msg = 'No command satisfies every rule at once — the arm has stopped.';
  else if (S.note) msg = S.note;
  else if (d && d.dist < 0.01) { msg = 'Target reached.'; kind = 'ok'; }
  else if (S.running && d && Math.hypot(...d.u) < 2e-3)
    msg = 'Stalled — a safety filter guarantees safety, not progress.';

  const b = $('banner');
  b.hidden = !msg;
  if (msg) { b.textContent = msg; b.dataset.kind = kind; }
}

function setNote() {
  $('note').textContent = S.P.wholeArm
    ? 'Every link is guarded: the filter watches the closest point of each link to each region.'
    : 'Only the end effector is guarded — the textbook case. Draw a region close in and the links can still pass through it.';
}


// ---- input ------------------------------------------------------------------------------------

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
  const { p: q, moved } = clampReach(p, S.L);
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
    S.drawn = { c: S.drag.c, a: S.drag.a, b: S.drag.b, eps: 1, tag: 'drawn region' };
  S.drag = null; S.dirty = true; schedule();
}
canvas.addEventListener('pointerup', () => endDrag(true));
canvas.addEventListener('pointercancel', () => endDrag(false));

document.querySelectorAll('[data-tool]').forEach((b) =>
  b.addEventListener('click', () => setTool(b.dataset.tool)));

$('run').addEventListener('click', () => {
  S.running = !S.running;
  $('run').innerHTML = S.running ? '&#10074;&#10074; Pause' : '&#9654; Run';
  last = 0; acc = 0; S.dirty = true; schedule();
});

function reset(joints) {
  const n = joints || S.q.length;
  Object.assign(S, scene(n, S.P.eps));
  S.trail = []; S.running = false; S.note = ''; S.drag = null;
  $('run').innerHTML = '&#9654; Run';
  S.dirty = true; schedule();
}
$('reset').addEventListener('click', () => reset());

$('clear').addEventListener('click', () => {
  S.obstacle = null; S.drawn = null; S.trail = []; S.note = '';
  S.dirty = true; schedule();
});

document.querySelectorAll('[data-flag]').forEach((el) =>
  el.addEventListener('change', (e) => {
    S.P[el.dataset.flag] = e.target.checked;
    setNote(); S.dirty = true; schedule();
  }));

document.querySelectorAll('[data-joints]').forEach((b) =>
  b.addEventListener('click', () => {
    const n = +b.dataset.joints;
    document.querySelectorAll('[data-joints]').forEach((o) =>
      o.setAttribute('aria-checked', String(+o.dataset.joints === n)));
    reset(n);
  }));

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
    resize();
  });
}

document.addEventListener('visibilitychange', () => { last = 0; acc = 0; schedule(); });
new ResizeObserver(resize).observe(canvas);
if (window.matchMedia) window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readTheme);

setTool('target');
setNote();
readTheme();
resize();
