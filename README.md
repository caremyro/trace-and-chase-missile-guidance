# 3D Missile–Aircraft Pursuit Simulation

> Designed as a demonstration tool for (simplified) aerospace dynamics.  
> Contributions and scenario expansions are welcome.

---

## Overview

This repository contains a Python simulation and visualization of a missile pursuing an aircraft in 3D space. Aircraft movement includes distinct straight-line and curved segments, and the missile uses two guidance strategies to intercept the target. Results are visualized in an animated 3D plot with real-time position and distance tracking.

---

## Features

- **Multi-segment target trajectory** — Straight flight, 3D banked turn, and final straight segment
- **Two guidance strategies**
  - `TraceAndChase.py` — Pure pursuit: missile steers toward the aircraft's instantaneous position
  - `TraceAndChaseWithLOS.py` — Proportional Navigation: missile nullifies the rotation of the Line Of Sight vector, resulting in a predictive intercept trajectory
- **Randomized scenarios** — Aircraft altitude, heading, turn geometry, and missile launch position are randomized at each run
- **Configurable parameters** — Speeds, positions, kill distance, turn geometry, and timings
- **3D animated visualization** using Matplotlib
- **Real-time telemetry** — Instantaneous distance, speed, and intercept marker

---

## Guidance Law — Proportional Navigation (LOS-based)

The core of `TraceAndChaseWithLOS.py` is a classical **Proportional Navigation (PN)** guidance law, widely used in real missile systems.

### 1. Line Of Sight (LOS) vector

At each timestep, the LOS vector points from the missile toward the target:

$$\vec{u}_{los} = \frac{\vec{pos}_{target} - \vec{pos}_{missile}}{|\vec{pos}_{target} - \vec{pos}_{missile}|}$$

### 2. Closing velocity

The relative velocity of the missile with respect to the target:

$$\vec{v}_{approach} = \vec{v}_{missile} - \vec{v}_{target}$$

The closing velocity (rate at which distance decreases):

$$V_c = \vec{v}_{approach} \cdot \hat{u}_{los}$$

$V_c > 0$ means the missile is closing on the target.

### 3. LOS rotation rate

The angular velocity of the LOS vector is computed with the exact geometric formula — **no finite differences, no division by dt**:

$$\vec{\omega}_{los} = \frac{\hat{u}_{los} \times \vec{v}_{approach}}{|\vec{LOS}|} \quad \text{[rad/s]}$$

> Dividing by $dt$ is deliberately avoided. At $dt = 0.001$ s it would amplify numerical noise by a factor of 1000, causing the guidance to diverge.

The physical interpretation: if the target moves at speed $v_\perp$ perpendicular to the LOS at distance $R$, the LOS rotates at $\omega = v_\perp / R$.

### 4. Acceleration command

The PN law commands a lateral acceleration proportional to the LOS rotation rate and the closing velocity:

$$\vec{a}_{cmd} = N \cdot V_c \cdot (\hat{u}_{los} \times \vec{\omega}_{los})$$

where $N$ is the **navigation constant** (typically 3–5). The cross product ensures the acceleration is perpendicular to the LOS, correcting the missile's heading to drive $\vec{\omega}_{los} \to 0$.

> If the LOS is not rotating ($\vec{\omega}_{los} = 0$), the missile is already on a collision course — no correction needed. Any LOS rotation means the missile will miss, and the guidance corrects accordingly.

The command is clipped to a maximum lateral acceleration:

$$|\vec{a}_{cmd}| \leq a_{max}$$

### 5. Speed normalization

After applying the acceleration, the velocity vector is renormalized to maintain constant missile speed:

$$\vec{v}_{missile} \leftarrow \frac{\vec{v}_{missile}}{|\vec{v}_{missile}|} \cdot v_{miss}$$

### Why PN outperforms pure pursuit

| | Pure Pursuit | Proportional Navigation |
|---|---|---|
| Aims at | Current target position | Predicted intercept point |
| Trajectory | Curved, energy-inefficient | Near-straight |
| Against fast targets | Can fail entirely | Robust |
| LOS angle | Continuously changing | Held constant |

Pure pursuit always chases where the target **is**. PN implicitly predicts where the target **will be** by keeping the LOS angle constant — this is the geometrical condition for a collision course.

---

## Requirements
```
Python 3.7+
numpy
matplotlib
scipy
```

---

## Parameters

| Variable | Description | Unit |
|---|---|---|
| `Straight_time` | Duration of first straight segment | s |
| `curve_time` | Duration of the banked turn | s |
| `Straight_time2` | Duration of second straight segment | s |
| `targ_vel` | Aircraft speed | m/s |
| `miss_vel` | Missile speed | m/s |
| `turn_angle` | Heading change during the turn | rad |
| `yz_angle` | Out-of-plane tilt of the turn | rad |
| `kill_dist` | Intercept radius | m |
| `climb_rate_curve` | Altitude change rate during turn | — |
| `N` | Proportional navigation constant | — |
| `max_accel` | Maximum lateral acceleration | m/s² |

In the randomized version, aircraft start position, altitude (8 000–15 000 m), heading, turn geometry, and missile launch position (ground level, Z = 0) are randomized at each run via `numpy.random.default_rng()`. Set a fixed seed for reproducibility:
```python
rng = np.random.default_rng(42)
```

---

## Output

The simulation displays:

- Aircraft and missile flight paths in 3D
- Instantaneous position, speed, and distance
- Intercept point marked with a star if achieved
- Ground plane at Z = 0 to visually anchor the missile launch site

---

## File Structure
```
.
├── TraceAndChase.py            # Pure pursuit guidance
├── TraceAndChaseWithLOS.py     # Proportional Navigation (LOS-based)
└── README.md
```

---

> Documentation written with the assistance of an AI language model.  
> The simulation code, physics, and guidance logic were mainly developed by the author.