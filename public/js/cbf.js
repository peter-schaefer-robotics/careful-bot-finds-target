// =====================================================================================
//  cbf.js — the maths. No DOM, no canvas, no globals.
//
//  The five numbered sections are the five lesson blocks on the page, in order.
//
//  Robot model:  q̇ = u.  The command IS the joint velocity (a lower-level controller is
//  assumed to track it). That is what makes ḣ linear in u further down, which is what
//  makes the safety filter a small quadratic program instead of a search.
//
//  Everything below works for any number of joints n. Nothing is hard-wired to two.
// =====================================================================================

// Link lengths for 2, 3 or 4 joints. Each set sums to 1.80 m, so the workspace — and the
// example scene — stay the same however many joints you add.
export const ARMS = {
  2: [1.00, 0.80],
  3: [0.80, 0.60, 0.40],
  4: [0.70, 0.50, 0.35, 0.25],
};

export function reach(L) {
  const total = L.reduce((a, b) => a + b, 0);
  const longest = Math.max(...L);
  return { inner: Math.max(0, longest - (total - longest)), outer: total };
}


// =====================================================================================
//  §1  FORWARD KINEMATICS — where the end effector is
//
//      p(q) = Σᵢ Lᵢ ( cos(q₁+…+qᵢ) , sin(q₁+…+qᵢ) )
//
//  Each angle is measured against the previous link, so the angles simply accumulate.
//  fk returns EVERY joint position, base first, end effector last — the whole skeleton.
// =====================================================================================

export function fk(q, L) {
  const pts = [[0, 0]];
  let a = 0, x = 0, y = 0;
  for (let i = 0; i < q.length; i++) {
    a += q[i];
    x += L[i] * Math.cos(a);
    y += L[i] * Math.sin(a);
    pts.push([x, y]);
  }
  return pts;
}

// Clamp a point into the reachable annulus, so a click outside it still means something.
export function clampReach(p, L) {
  const { inner, outer } = reach(L);
  const r = Math.hypot(p[0], p[1]);
  const lo = inner + 0.02, hi = outer - 0.02;
  if (r >= lo && r <= hi) return { p: p.slice(), moved: false };
  if (r < 1e-9) return { p: [lo, 0], moved: true };
  const k = (r < lo ? lo : hi) / r;
  return { p: [p[0] * k, p[1] * k], moved: true };
}


// =====================================================================================
//  §2  THE JACOBIAN — how joint speeds become Cartesian speed:  ṗ = J(q) q̇
//
//  Column i of J is  ẑ × (p − joint i)  for every joint before the point, and zero after
//  it: spinning a joint sweeps everything beyond it around that joint, and moves nothing
//  before it. That one sentence is the whole Jacobian, for any n.
//
//  jacobianAt gives the point a fraction s along link k (k = 1…n) and its Jacobian, so
//  the filter can protect a point anywhere on the arm, not only the tip.
// =====================================================================================

export function jacobianAt(pts, k, s) {
  const n = pts.length - 1;
  const A = pts[k - 1], B = pts[k];
  const p = [A[0] + s * (B[0] - A[0]), A[1] + s * (B[1] - A[1])];
  const J = [];
  for (let i = 0; i < n; i++)
    J.push(i < k ? [-(p[1] - pts[i][1]), p[0] - pts[i][0]] : [0, 0]);
  return { p, J };
}


// =====================================================================================
//  §3  THE BARRIER — what is forbidden
//
//      h(p) = √(ab)·[ ( (|x′|/a)ⁿ + (|y′|/b)ⁿ )^(1/n) − 1 ],   n = 2/ε
//
//  h > 0 outside (safe) · h = 0 on the boundary · h < 0 inside (violated).
//
//  ONE formula for every forbidden region: ε = 1 is an ellipse, a small ε squares off the
//  corners. A region is { c:[cx,cy], a, b, eps }.
//
//  The 1/n root and the √(ab) factor are a monotone rescale — they move neither the zero
//  level set nor the safe set — but they put h in METRES, so ‖∇h‖ stays between √(b/a)
//  and √(a/b) and h is the clearance you can see on screen. That is what lets one α mean
//  the same thing for a boxy region and a round one.
// =====================================================================================

export function barrier(reg, p) {
  const n = 2 / reg.eps;
  const X = p[0] - reg.c[0], Y = p[1] - reg.c[1];
  const ax = Math.abs(X) / reg.a, ay = Math.abs(Y) / reg.b;
  const rho = Math.sqrt(reg.a * reg.b);
  const F = Math.pow(Math.pow(ax, n) + Math.pow(ay, n), 1 / n);

  // ∇h vanishes at exactly one point, the centre. Point outwards there rather than 0/0.
  if (F < 1e-9) return { h: -rho, gx: rho / reg.a, gy: 0 };

  // Math.pow(0, n−1) = 0 and Math.sign(0) = 0 give the right answer on the axes, so the
  // absolute values need no special case (and cause no kink, for ε < 2).
  return {
    h: rho * (F - 1),
    gx: rho * Math.pow(ax / F, n - 1) * Math.sign(X) / reg.a,
    gy: rho * Math.pow(ay / F, n - 1) * Math.sign(Y) / reg.b,
  };
}

// The h = 0 curve itself, for drawing — exact, not a sampling of h.
export function outline(reg, steps = 128) {
  const pts = [];
  for (let i = 0; i < steps; i++) {
    const t = (2 * Math.PI * i) / steps;
    const ct = Math.cos(t), st = Math.sin(t);
    pts.push([
      reg.c[0] + reg.a * Math.sign(ct) * Math.pow(Math.abs(ct), reg.eps),
      reg.c[1] + reg.b * Math.sign(st) * Math.pow(Math.abs(st), reg.eps),
    ]);
  }
  return pts;
}

// h is convex along any straight line (F is a p-norm with p = 2/ε ≥ 2), so a ternary
// search finds the GLOBAL closest point of a link to a region — one exact row per link,
// which is both cheaper and tighter than sprinkling sample points along it.
function closestOnLink(reg, pts, k) {
  const A = pts[k - 1], B = pts[k];
  const at = (s) => barrier(reg, [A[0] + s * (B[0] - A[0]), A[1] + s * (B[1] - A[1])]).h;
  let lo = 0, hi = 1;
  for (let i = 0; i < 28; i++) {
    const m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
    if (at(m1) < at(m2)) hi = m2; else lo = m1;
  }
  return (lo + hi) / 2;
}

// Closest points of two segments, for self-collision. Standard 2D construction.
function segToSeg(p1, q1, p2, q2) {
  const d1 = [q1[0] - p1[0], q1[1] - p1[1]];
  const d2 = [q2[0] - p2[0], q2[1] - p2[1]];
  const r = [p1[0] - p2[0], p1[1] - p2[1]];
  const a = d1[0] * d1[0] + d1[1] * d1[1];
  const e = d2[0] * d2[0] + d2[1] * d2[1];
  const f = d2[0] * r[0] + d2[1] * r[1];
  const c = d1[0] * r[0] + d1[1] * r[1];
  const b = d1[0] * d2[0] + d1[1] * d2[1];
  const den = a * e - b * b;
  let s = den > 1e-12 ? Math.min(1, Math.max(0, (b * f - c * e) / den)) : 0;
  let t = (b * s + f) / e;
  if (t < 0) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
  else if (t > 1) { t = 1; s = Math.min(1, Math.max(0, (b - c) / a)); }
  const c1 = [p1[0] + d1[0] * s, p1[1] + d1[1] * s];
  const c2 = [p2[0] + d2[0] * t, p2[1] + d2[1] * t];
  return { s, t, c1, c2, d: Math.hypot(c2[0] - c1[0], c2[1] - c1[1]) };
}


// =====================================================================================
//  §4  THE NOMINAL CONTROLLER — where you want to go
//
//      v = (p* − p) · min( k_p , v_max/‖p* − p‖ )    ← cap in Cartesian space, before
//      u_nom = Jᵀ (J Jᵀ + λ²I)⁻¹ v                      inverting, so the unfiltered path
//                                                       is a genuine straight line
//
//  Damped least squares rather than a plain inverse. J Jᵀ is 2×2 however many joints
//  there are, and its determinant is at least λ⁴ > 0 for every pose, so there is no
//  singular case to special-case — writing `if (det < ε)` would put back exactly the
//  discontinuity that damping exists to remove. λ is a LENGTH and scales with the arm.
// =====================================================================================

export function nominal(q, L, target, P) {
  const n = q.length;
  const pts = fk(q, L);
  const ee = pts[n];
  const ex = target[0] - ee[0], ey = target[1] - ee[1];
  const d = Math.hypot(ex, ey);
  if (d < 0.004) return { u: new Array(n).fill(0), v: [0, 0], dist: d };

  const g = Math.min(P.kp, P.vmax / d);
  const v = [ex * g, ey * g];

  const { J } = jacobianAt(pts, n, 1);
  let a = 0, b = 0, c = 0;                       // J Jᵀ, symmetric 2×2
  for (const col of J) { a += col[0] * col[0]; b += col[0] * col[1]; c += col[1] * col[1]; }
  const l2 = P.lambda * P.lambda;
  a += l2; c += l2;
  const det = a * c - b * b;                     // ≥ λ⁴ > 0
  const w = [(v[0] * c - b * v[1]) / det, (a * v[1] - b * v[0]) / det];

  return { u: J.map((col) => col[0] * w[0] + col[1] * w[1]), v, dist: d };
}


// =====================================================================================
//  §5  THE RULE AND THE FILTER
//
//  The rule, one row per guarded point per region:
//      ḣ = ∂h/∂p · ∂p/∂q · q̇ = ∇h(p)ᵀ J(q) u  ≥  −α(h)      ⇔   a·u ≥ b
//  with a = Jᵀ∇h and b = −α(h). u appears only multiplied by matrices that depend on q:
//  ḣ is LINEAR in u, which is the whole reason the filter below is a quadratic program.
//
//  α(·) may be any extended class-K function — through the origin, strictly increasing.
//  We use α(h) = α·sign(h)·|h|^p. p = 1 is the textbook straight line; p = 2 is the
//  paper's h², written so it keeps increasing through zero and stays legal for h < 0.
// =====================================================================================

export const classK = (h, P) => P.alpha * Math.sign(h) * Math.pow(Math.abs(h), P.expo);

export function rows(q, L, regions, P) {
  const n = q.length;
  const pts = fk(q, L);
  const out = [];
  const push = (h, J, gx, gy, tag, at) =>
    out.push({ a: J.map((col) => col[0] * gx + col[1] * gy), b: -classK(h, P), h, gx, gy, tag, at });

  for (const reg of regions) {
    if (!reg) continue;
    if (P.wholeArm) {
      for (let k = 1; k <= n; k++) {
        const { p, J } = jacobianAt(pts, k, closestOnLink(reg, pts, k));
        const { h, gx, gy } = barrier(reg, p);
        push(h, J, gx, gy, reg.tag, p);
      }
    } else {
      const { p, J } = jacobianAt(pts, n, 1);
      const { h, gx, gy } = barrier(reg, p);
      push(h, J, gx, gy, reg.tag, p);
    }
  }

  // Self-collision: the arm against itself. Only links two apart or more — neighbours
  // share a joint, so their distance is always zero and there is nothing to enforce.
  if (P.selfAvoid) {
    for (let i = 1; i <= n; i++) {
      for (let j = i + 2; j <= n; j++) {
        const r = segToSeg(pts[i - 1], pts[i], pts[j - 1], pts[j]);
        if (r.d < 1e-6) continue;                       // crossing exactly: no usable normal
        const A = jacobianAt(pts, i, r.s), B = jacobianAt(pts, j, r.t);
        const nx = (B.p[0] - A.p[0]) / r.d, ny = (B.p[1] - A.p[1]) / r.d;
        out.push({
          // ḋ = n̂ᵀ(ṗ_j − ṗ_i), so the row is the DIFFERENCE of the two Jacobians
          a: A.J.map((col, i2) => (B.J[i2][0] - col[0]) * nx + (B.J[i2][1] - col[1]) * ny),
          b: -classK(r.d - P.selfGap, P),
          h: r.d - P.selfGap, gx: nx, gy: ny, tag: 'self',
          at: [(A.p[0] + B.p[0]) / 2, (A.p[1] + B.p[1]) / 2], pair: [A.p, B.p],
        });
      }
    }
  }
  return out;
}

const dot = (x, y) => x.reduce((s, xi, i) => s + xi * y[i], 0);
const norm = (x) => Math.sqrt(dot(x, x));

// Solve the small symmetric system G λ = r by Gaussian elimination. null if singular.
function solveSmall(G, r) {
  const m = r.length, M = G.map((row, i) => [...row, r[i]]);
  for (let k = 0; k < m; k++) {
    let piv = k;
    for (let i = k + 1; i < m; i++) if (Math.abs(M[i][k]) > Math.abs(M[piv][k])) piv = i;
    if (Math.abs(M[piv][k]) < 1e-12) return null;
    [M[k], M[piv]] = [M[piv], M[k]];
    for (let i = k + 1; i < m; i++) {
      const f = M[i][k] / M[k][k];
      for (let j = k; j <= m; j++) M[i][j] -= f * M[k][j];
    }
  }
  const x = new Array(m).fill(0);
  for (let i = m - 1; i >= 0; i--) {
    let s = M[i][m];
    for (let j = i + 1; j < m; j++) s -= M[i][j] * x[j];
    x[i] = s / M[i][i];
  }
  return x;
}

//  The filter:
//      u = argmin ½‖u − u_nom‖²   subject to every row
//  The objective is strictly convex, so the answer is the unique projection of u_nom onto
//  the feasible polyhedron, and at the optimum some subset of the rows holds with
//  equality. So: try every subset, solve the equality problem for it, and keep the
//  feasible answer of least cost. That is exact, and needs no multiplier sign test.
//  At most n rows can be active at once, which bounds the subsets we must try.
export function filter(uNom, R) {
  const n = uNom.length;
  if (!R.length) return { u: uNom.slice(), active: [], ok: true };

  // Every h ≥ 0 means every b ≤ 0, so u = 0 is feasible and the answer never moves
  // further from u_nom than ‖u_nom‖. A row with more slack than that can never become
  // active, so it can be dropped — exactly, not heuristically.
  const safe = R.every((r) => r.b <= 0);
  const un = norm(uNom);
  const live = [];
  for (let i = 0; i < R.length; i++) {
    const na = norm(R[i].a);
    if (na < 1e-12) {                                   // this h cannot be steered at all
      if (R[i].b > 1e-12) return { u: new Array(n).fill(0), active: [], ok: false };
      continue;
    }
    if (safe && dot(R[i].a, uNom) - R[i].b > na * un) continue;
    live.push(i);
  }

  const feasible = (u) => R.every((r) =>
    dot(r.a, u) >= r.b - 1e-9 * (1 + norm(u)));

  let best = uNom.slice(), bestCost = feasible(uNom) ? 0 : Infinity, bestSet = [];

  const maxSize = Math.min(n, live.length);
  const S = [];
  const search = (start) => {
    if (S.length) {
      const G = S.map((i) => S.map((j) => dot(R[i].a, R[j].a)));
      const rhs = S.map((i) => R[i].b - dot(R[i].a, uNom));
      const lam = solveSmall(G, rhs);
      if (lam) {
        const u = uNom.slice();
        S.forEach((i, k) => { for (let d2 = 0; d2 < n; d2++) u[d2] += lam[k] * R[i].a[d2]; });
        const diff = u.map((x, d2) => x - uNom[d2]);
        const cost = dot(diff, diff);
        if (cost < bestCost && feasible(u)) { bestCost = cost; best = u; bestSet = S.slice(); }
      }
    }
    if (S.length === maxSize) return;
    for (let i = start; i < live.length; i++) { S.push(live[i]); search(i + 1); S.pop(); }
  };
  search(0);

  // No candidate feasible means the feasible set is provably empty. That can only happen
  // once some h is already negative: while every h ≥ 0, u = 0 satisfies every row. We
  // stop and say so rather than relaxing the constraint.
  if (bestCost === Infinity) return { u: new Array(n).fill(0), active: [], ok: false };
  return { u: best, active: bestSet, ok: true };
}

// One explicit Euler step of q̇ = u. Δt is fixed by the caller; α·Δt ≤ 1 keeps the
// invariance argument intact across the discretisation.
export function step(S, dt) {
  const P = S.P, L = S.L;
  const nom = nominal(S.q, L, S.target, P);
  const R = rows(S.q, L, [S.obstacle, S.drawn], P);
  const f = P.filterOn ? filter(nom.u, R) : { u: nom.u, active: [], ok: true };
  for (let i = 0; i < S.q.length; i++) S.q[i] += f.u[i] * dt;
  return { u: f.u, uNom: nom.u, rows: R, active: f.active, ok: f.ok, dist: nom.dist };
}
