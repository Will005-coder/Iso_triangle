# Isoperimetric Planar Robot | Mathematical Overview

**Author:** William Dakare

**Versions**

* `Will-c-v1` — Rate-only perimeter constraint
* `Will-c-v2` — Baumgarte-stabilized perimeter constraint

---

# 1. State Representation

The robot consists of three nodes in 2D:

```text
x = [x1, y1, x2, y2, x3, y3]^T
```

The solver computes node velocities:

```text
x_dot
```

at each timestep.

---

# 2. Velocity-Level IK Formulation

All objectives and constraints are written as linear velocity equations:

```text
R x_dot = L_dot
```

where:

* `R` = stacked constraint Jacobians
* `L_dot` = desired task outputs

The system is solved using a pseudoinverse:

```matlab
x_dot = pinv(R) * L_dot;
```

This produces the minimum-norm velocity solution that best satisfies all rows simultaneously.

---

# 3. Target Tracking Task

The primary task is moving node 2 toward a target.

Error:

```text
e = x_target - x_node2
```

Desired velocity:

```text
v_task = kp * e
```

Task Jacobian:

```text
J =
[0 0 1 0 0 0
 0 0 0 1 0 0]
```

Constraint equation:

```text
J x_dot = kp e
```

The gain `kp` controls how aggressively the node moves toward the target.

---

# 4. Perimeter Constraint

The perimeter is

```text
P = l12 + l23 + l31
```

Its rate of change is

```text
dP/dt = Jp x_dot
```

where `Jp` is the perimeter Jacobian.

Each edge contributes according to its unit direction vector.

---

# 5. Version 1: Rate Constraint

The original formulation used

```text
Jp x_dot = 0
```

which only enforces:

```text
dP/dt = 0
```

This prevents perimeter changes during the current step but does not restore previously accumulated error.

As a result, numerical drift causes:

```text
P ≠ P0
```

over long simulations.

A post-processing perimeter rescaling step was added, but it violated DOF constraints because it modified node positions outside the solver.

---

# 6. Version 2: Baumgarte Stabilization

Define the perimeter error:

```text
Φ = P - P0
```

Instead of enforcing zero perimeter rate, v2 uses:

```text
dP/dt = -k_b (P - P0)
```

or

```text
Jp x_dot = -k_b (P - P0)
```

This creates a restoring velocity proportional to perimeter error.

The stacked system becomes:

```text
R =
[
J
Jp
]
```

```text
L_dot =
[
kp e
-k_b (P - P0)
]
```

Consequences:

| Property                       | v1          | v2                   |
| ------------------------------ | ----------- | -------------------- |
| Constraint                     | `dP/dt = 0` | `dP/dt = -k_b(P-P0)` |
| Drift correction               | No          | Yes                  |
| Long-term perimeter accuracy   | Poor        | Stable               |
| Post-solve correction required | Yes         | No                   |

Perimeter errors now decay continuously instead of accumulating.

---

# 7. Gain Selection

Two competing signals exist:

```text
Task magnitude       ~ kp |e|
Perimeter magnitude  ~ k_b |P-P0|
```

Guidelines:

* Large `kp` → faster target tracking
* Large `k_b` → stronger perimeter preservation
* Excessive `k_b` can overpower task motion

Typical ratio:

```text
k_b ≈ 2–5 × kp
```

Current values:

```text
kp = 0.2
k_b = 0.3
```

---

# 8. DOF Constraints

Fixed coordinates are enforced with identity rows:

```text
[0 ... 1 ... 0] x_dot = 0
```

Each row freezes a single coordinate velocity.

Current implementation pins:

```text
[1, 2, 6]
```

which corresponds to:

* Node 1 x
* Node 1 y
* Node 3 y

---

# 9. Collision Avoidance

For each node, distance to the opposite edge is monitored.

When the distance falls below:

```text
d_min = 1
```

a constraint is added that prevents further motion toward that edge.

This is implemented conservatively as a zero-normal-velocity condition.

---

# 10. Integration

Node positions are updated using explicit Euler integration:

```matlab
node_pst = node_pst + dt * reshape(x_dot,2,[]);
```

with

```text
dt = 0.05
```

Baumgarte stabilization compensates for the constraint drift introduced by Euler integration.

---

# 11. Convergence

Simulation terminates when node 2 reaches the target:

```text
||e|| < 5e-2
```

This threshold provides reliable convergence while avoiding excessive iterations near the goal.

---

# Key Insight

The main improvement from v1 to v2 is replacing a **rate-preserving perimeter constraint**

```text
dP/dt = 0
```

with a **restoring perimeter constraint**

```text
dP/dt = -k_b(P-P0)
```

which transforms perimeter preservation from a passive constraint into an active feedback controller and eliminates long-term drift without violating DOF constraints.
