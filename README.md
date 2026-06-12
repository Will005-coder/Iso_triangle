# Closed-Loop Path Planning of a Compliant, Constant-Perimeter Truss Robot

**Author:** William Dakare  
**Based on work by:** Nathan Usevitch [Untethered Truss](https://www.science.org/doi/10.1126/scirobotics.aaz0492)
**Branch:** `Will-c-v1` → `Will-c-v2`

---

## Overview

This project implements a velocity-level inverse kinematics controller for a compliant triangular truss robot operating under a constant perimeter (isoperimetric) constraint. A single node is commanded to a 2D Cartesian setpoint while the remaining nodes deform freely, preserving total edge length throughout motion.

The controller is grounded in differential kinematics — the same Jacobian-based framework used in serial manipulator IK — adapted here for a variable-geometry compliant structure whose Jacobian updates every timestep as the triangle deforms. Constraint drift, the fundamental failure mode of velocity-level integration, is suppressed using Baumgarte stabilization rather than post-hoc geometric correction.

---

## Core Principle

At every timestep the system solves:

```
R * x_dt = L_dt
```

Where `x_dt` is the 6×1 vector of node velocities, `R` is the stacked Jacobian encoding all active constraints, and `L_dt` is the desired output for each constraint row. The pseudoinverse finds the minimum-norm velocity that satisfies the task, perimeter, structural, and collision constraints simultaneously. Positions are then updated by explicit Euler integration.

---

## Documentation

For software architecture, branch comparison, and parameter reference see:
- [ARCHITECTURE.md](ARCHITECTURE.md)

For the full mathematical basis including IK formulation, Baumgarte derivation, and constraint stacking see:
- [MATH.md](MATH.md)

---

## Quick Start

```matlab
% Run the controller
isoperimetric_planar_robot.m
```

Requires MATLAB with the `nodes` class (`robot.m`) on the path. The figure window updates live and freezes on convergence.

---

## Key Parameters

| Parameter | Description | Default |
|---|---|---|
| `x_target` | 2D Cartesian setpoint for the controlled node | `[1, 1]` |
| `target_node` | Index of the node being controlled | `2` |
| `kp` | Proportional gain on task error | `0.2` |
| `k_b` | Baumgarte perimeter restoring gain | `0.3` |
| `dt` | Integration timestep | `0.05` |
| `d_min` | Minimum node-to-edge clearance before collision constraint activates | `1.0` |

---

## Related Work

- Usevith, N. | [Untethered Isoperimetric Truss](https://www.science.org/doi/10.1126/scirobotics.aaz0492)
- Baumgarte, J. (1972) | Stabilization of constraints and integrals of motion in dynamical systems
- Nakamura, Y. (1991) | Advanced Robotics: Redundancy and Optimization
