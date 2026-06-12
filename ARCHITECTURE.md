# Isoperimetric Planar Robot | Architecture Overview

**Author:** William Dakare  
**Branches:** `Will-c-v1` (numeric rectifier) → `Will-c-v2` (Baumgarte stabilization)

---

## What This Robot Does

A triangular truss robot whose nodes are connected by edges of fixed total perimeter. One node is commanded to a 2D setpoint while the shape deforms around it, preserving the total edge length (the isoperimetric constraint). The solver computes velocities for all nodes each timestep such that the task, shape, and structural constraints are satisfied simultaneously.

---

## High-Level Pipeline (Both Versions)

```
1. Initialize nodes 
2. Compute Jacobians
3. Stack constraint system 
4. Solve for velocities 
5. Integrate 
6. Repeat
```

Each iteration:
1. Read current node positions
2. Build the task Jacobian J (where should node 2 go?)
3. Build the perimeter Jacobian Jp (how does perimeter change with node motion?)
4. Stack all constraints into one linear system R * x_dt = L_dt
5. Solve via pseudoinverse for node velocities x_dt
6. Integrate: node_pst = node_pst + dt * x_dt

---

## Branch Comparison

| Feature | `Will-c-v1` | `Will-c-v2` |
|---|---|---|
| Perimeter constraint RHS | `0` (hold current rate) | `-k_b * (P_current - P0)` (correct drift) |
| Post-solve correction | `enforce_perimeter` centroid scaling, commented out | Not needed |
| Velocity smoothing | Adaptive `alpha * tanh(dist/0.05)` | Removed — Baumgarte handles stability |
| DOF constraints | Nodes `[1,2]` + `[1,2,6]` duplicated | `[1,2,6]` only |
| Convergence threshold | `5e-3` | `5e-2` relaxed, more robust |
| New gain | — | `k_b = 0.3` |
| `converged` flag | No | Yes — stops redrawing on arrival |

---

## Key Architectural Improvements in v2

**1. Perimeter drift is corrected inside the solver, not patched after it.**

In v1, the perimeter row told the solver "don't change perimeter right now." If drift had already accumulated from previous steps, the solver had no way to know — it just froze the drift in place. The commented-out `enforce_perimeter` was a post-hoc patch that moved even pinned nodes because it had no knowledge of the constraint structure. In v2, the right-hand side carries a corrective signal proportional to how far the perimeter has drifted, so the solver actively pulls it back each step through the same pseudoinverse that respects all other constraints.

**2. Smoothing removed.**

The adaptive `tanh` smoothing in v1 was compensating for instability caused by the perimeter constraint fighting the task. Once the constraint is properly stabilized with a restoring signal, the jitter disappears and smoothing is no longer needed — and its removal means the solver responds more directly to the error signal.

**3. Cleaner DOF pinning.**

v1 called `add_dof_constraints` twice — once with `[1,2]` and again with `[1,2,6]` — meaning DOFs 1 and 2 appeared twice as redundant rows in the stacked system. v2 calls it once with `[1,2,6]`.

**4. Convergence flag stops the animation.**

v1 would keep redrawing even after `break` was hit (or never hit). v2 uses a `converged` boolean to gate the `draw_robot` call, freezing the figure cleanly when the target is reached and setting the title to confirm arrival.

---

## File Structure

```
robot.m                        — Node class: coordinates, compute_perimeter, compute_edge
isoperimetric_planar_robot.m   — Main script: solver loop, drawing, constraints
```

---

## Parameters Quick Reference

| Parameter | Role | v1 | v2 |
|---|---|---|---|
| `dt` | Integration timestep | 0.05 | 0.05 |
| `kp` | Task proportional gain | 0.2 | 0.2 |
| `k_b` | Baumgarte restoring gain | — | 0.3 |
| `alpha` | Velocity smoothing factor | 0.35 | removed |
| `d_min` | Collision clearance distance | 1.0 | 1.0 |
| `N` | Max iterations | 10000 | 10000 |
