// =====================================================================================
//  cbf.js — the maths. No DOM, no canvas, no globals.
//
//  The five numbered sections below are the five lesson blocks on the page, in order.
//  Everything is plain ES2020; read it top to bottom.
//
//  Robot model:  q̇ = u.  The command IS the joint velocity (a lower-level controller is
//  assumed to track it). That is what makes ḣ linear in u further down, which is what
//  makes the safety filter a small quadratic program instead of a search.
// =====================================================================================

export const ARM = { L1: 1.0, L2: 0.8 };

// Reachable workspace: an annulus around the base.
export const REACH = { inner: Math.abs(ARM.L1 - ARM.L2), outer: ARM.L1 + ARM.L2 };


// =====================================================================================
//  §1  FORWARD KINEMATICS — where the end effector is
//
//      p(q) = ( L₁cos q₁ + L₂cos(q₁+q₂) , L₁sin q₁ + L₂sin(q₁+q₂) )
// =====================================================================================

export function fk(q) {
  const c1 = Math.cos(q[0]), s1 = Math.sin(q[0]);
  const c12 = Math.cos(q[0] + q[1]), s12 = Math.sin(q[0] + q[1]);
  const ex = ARM.L1 * c1, ey = ARM.L1 * s1;
  return { elbow: [ex, ey], ee: [ex + ARM.L2 * c12, ey + ARM.L2 * s12] };
}

// Clamp a point into the reachable annulus, so a click outside it still means something.
export function clampReach(p) {
  const r = Math.hypot(p[0], p[1]);
  const lo = REACH.inner + 0.02, hi = REACH.outer - 0.02;
  if (r >= lo && r <= hi) return { p: p.slice(), moved: false };
  if (r < 1e-9) return { p: [lo, 0], moved: true };
  const k = (r < lo ? lo : hi) / r;
  return { p: [p[0] * k, p[1] * k], moved: true };
}


// =====================================================================================
//  §2  THE JACOBIAN — how joint speeds become end-effector speed:  ṗ = J(q) q̇
//
//      J = [ −L₁s₁ − L₂s₁₂   −L₂s₁₂ ]        det J = L₁L₂ sin q₂
//          [  L₁c₁ + L₂c₁₂    L₂c₁₂ ]
//
//  Column 1 is ẑ × (ee − base), column 2 is ẑ × (ee − elbow): each column is the
//  end-effector velocity you get by spinning the joint it belongs to. Returned row-major.
// =====================================================================================

export function jacobian(q) {
  const s1 = Math.sin(q[0]), c1 = Math.cos(q[0]);
  const s12 = Math.sin(q[0] + q[1]), c12 = Math.cos(q[0] + q[1]);
  return [
    -ARM.L1 * s1 - ARM.L2 * s12, -ARM.L2 * s12,
     ARM.L1 * c1 + ARM.L2 * c12,  ARM.L2 * c12,
  ];
}

export const detJ = (J) => J[0] * J[3] - J[1] * J[2];


// =====================================================================================
//  §3  THE BARRIER — what is forbidden
//
//      h(p) = √(ab)·[ ( (|x′|/a)^(2/ε) + (|y′|/b)^(2/ε) )^(ε/2) − 1 ]
//
//  h > 0 outside (safe) · h = 0 on the boundary · h < 0 inside (violated).
//
//  ONE formula for both shapes: ε = 1 is an ellipse, a small ε squares off the corners
//  into a rounded rectangle. A region is { c:[cx,cy], a, b, eps }.
//
//  The ^(ε/2) root and the √(ab) factor are a monotone rescale — they move neither the
//  zero level set nor the safe set — but they put h in METRES, so ‖∇h‖ stays between
//  √(b/a) and √(a/b) and h is the clearance you can see on screen. That is what lets one
//  α mean the same thing for a boxy region and a round one.
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


// =====================================================================================
//  §4  THE NOMINAL CONTROLLER — where you want to go
//
//      v = (p* − p) · min( k_p , v_max/‖p* − p‖ )        ← cap in Cartesian space, so the
//      (JᵀJ + λ²I) u_nom = Jᵀv                             unfiltered path is a straight line
//
//  Damped least squares rather than J⁻¹. det(JᵀJ + λ²I) ≥ λ⁴ > 0 for every pose, so there
//  is no singular case to special-case — writing `if (|det J| < ε)` would put back exactly
//  the discontinuity that damping exists to remove. λ is a LENGTH and scales with the arm.
// =====================================================================================

export function nominal(q, target, P) {
  const { ee } = fk(q);
  const ex = target[0] - ee[0], ey = target[1] - ee[1];
  const d = Math.hypot(ex, ey);
  if (d < 0.004) return { u: [0, 0], v: [0, 0], dist: d };

  const g = Math.min(P.kp, P.vmax / d);
  const vx = ex * g, vy = ey * g;

  const J = jacobian(q), l2 = P.lambda * P.lambda;
  const A = J[0] * J[0] + J[2] * J[2] + l2;   // (JᵀJ + λ²I)₁₁
  const B = J[0] * J[1] + J[2] * J[3];        // (JᵀJ)₁₂
  const C = J[1] * J[1] + J[3] * J[3] + l2;   // (JᵀJ + λ²I)₂₂
  const r0 = J[0] * vx + J[2] * vy;           // (Jᵀv)₁
  const r1 = J[1] * vx + J[3] * vy;           // (Jᵀv)₂
  const det = A * C - B * B;                  // ≥ λ⁴ > 0

  return { u: [(r0 * C - B * r1) / det, (A * r1 - B * r0) / det], v: [vx, vy], dist: d };
}


// =====================================================================================
//  §5  THE RULE AND THE FILTER
//
//  The rule, one row per region:
//      ḣ = ∂h/∂p · ∂p/∂q · q̇ = ∇h(p)ᵀ J(q) u  ≥  −α h        ⇔   a·u ≥ b
//  with a = Jᵀ∇h and b = −αh. Note u appears only multiplied by matrices that depend on q:
//  ḣ is LINEAR in u, which is the whole reason the filter below is a quadratic program.
//
//  The filter:
//      u = argmin ½‖u − u_nom‖²   subject to every row
//  u lives in R², and there are at most two rows, so the optimum is one of four points:
//  u_nom itself, the projection onto either row alone, or the intersection of both.
//  Try all four, keep the feasible one of least cost. That is exact — the objective is
//  strictly convex, so the answer is the unique projection onto the feasible polygon, and
//  picking the cheapest feasible candidate finds it with no multiplier sign test.
// =====================================================================================

export function rows(q, regions, P) {
  const { ee } = fk(q);
  const J = jacobian(q);
  const out = [];
  for (const reg of regions) {
    if (!reg) continue;
    const { h, gx, gy } = barrier(reg, ee);
    out.push({
      a0: J[0] * gx + J[2] * gy,   // (Jᵀ∇h)₁
      a1: J[1] * gx + J[3] * gy,   // (Jᵀ∇h)₂
      b: -P.alpha * h,
      h, gx, gy, reg,
    });
  }
  return out;
}

const satisfies = (u, R) => R.every((r) =>
  r.a0 * u[0] + r.a1 * u[1] >= r.b - 1e-9 * (1 + Math.abs(u[0]) + Math.abs(u[1])));

export function filter(uNom, R) {
  if (R.length === 0) return { u: uNom.slice(), active: [], ok: true };

  const cand = [{ u: uNom.slice(), active: [] }];

  for (let i = 0; i < R.length; i++) {                       // project onto one row
    const r = R[i], nn = r.a0 * r.a0 + r.a1 * r.a1;
    if (nn < 1e-18) continue;                                // this h cannot be steered at all
    const s = (r.b - (r.a0 * uNom[0] + r.a1 * uNom[1])) / nn;
    if (s > 0) cand.push({ u: [uNom[0] + r.a0 * s, uNom[1] + r.a1 * s], active: [i] });
  }

  for (let i = 0; i < R.length; i++) {                       // meet two rows exactly
    for (let j = i + 1; j < R.length; j++) {
      const A = R[i], B = R[j];
      const cross = A.a0 * B.a1 - A.a1 * B.a0;
      if (Math.abs(cross) < 1e-9) continue;                  // parallel: the pair adds nothing
      cand.push({
        u: [(A.b * B.a1 - B.b * A.a1) / cross, (A.a0 * B.b - B.a0 * A.b) / cross],
        active: [i, j],
      });
    }
  }

  let best = null, bestCost = Infinity;
  for (const c of cand) {
    if (!satisfies(c.u, R)) continue;
    const dx = c.u[0] - uNom[0], dy = c.u[1] - uNom[1];
    const cost = dx * dx + dy * dy;
    if (cost < bestCost) { bestCost = cost; best = c; }
  }

  // No candidate feasible means the feasible set is provably empty. That can only happen
  // once some h is already negative: while every h ≥ 0 we have b ≤ 0, so u = 0 satisfies
  // every row. We stop and say so rather than relaxing the constraint.
  if (!best) return { u: [0, 0], active: [], ok: false };
  return { u: best.u, active: best.active, ok: true };
}

// One explicit Euler step of q̇ = u. Δt is fixed by the caller; αΔt ≤ 1 keeps the
// invariance argument intact across the discretisation.
export function step(S, dt) {
  const P = S.P;
  const nom = nominal(S.q, S.target, P);
  const R = rows(S.q, [S.obstacle, S.drawn], P);
  const f = P.filterOn ? filter(nom.u, R) : { u: nom.u, active: [], ok: true };

  S.q[0] += f.u[0] * dt;
  S.q[1] += f.u[1] * dt;

  return { u: f.u, uNom: nom.u, rows: R, active: f.active, ok: f.ok, dist: nom.dist };
}
