# ARL Autonomous Vehicle Control Track


### Student Information

| | |
|---|---|
| **Name** | Basant Aref Mahmoud |
| **Programme** | Mechatronics Engineering|
| **Institution** | Ain Shams University |


---

## Overview

This repository implements the complete control stack for a simulated autonomous vehicle in ROS 2 Humble. The vehicle is modelled as an **extended kinematic bicycle**, in which longitudinal velocity is a state rather than an input . the car accelerates and decelerates subject to rolling resistance and speed-squared aerodynamic drag, so a closed-loop speed regulator is required.

Three lateral controllers  were implemented and benchmarked against one another and against a human driver:

| Controller | Principle | Information used |
|---|---|---|
| **Lateral PID** | Reactive error feedback | Present cross-track and heading error |
| **Pure Pursuit** | Geometric preview | A single lookahead point on the path |
| **Extended Kinematic MPC** | Constrained optimal preview | A 10-step horizon plus a vehicle model |

**Headline result:** MPC tracks the reference path **1.7× more accurately than Pure Pursuit**, **11× more accurately than Lateral PID**, and **85× more accurately than a human driver** — but only when it is given the CPU it needs. That last qualification turned out to be the most interesting finding in the project, and is documented in [Real-Time Computation as a Design Constraint](#real-time-computation-as-a-design-constraint).

---

## System Architecture

The stack is three ROS 2 packages and six nodes, communicating over a small, deliberately flat topic graph.

```
  path_gen ───── /path ─────┐
                            ▼
  sim_node ──── /state ──► controller ──── /steer ────► sim_node
  (plant)          │        (10 Hz)   ──── /throttle ─► (plant)
                   │
                   ▼
              lap_analyzer
                   │
                   ├─► /telemetry/cte,  /telemetry/speed
                   ├─► /telemetry/heading_err_deg
                   ├─► /telemetry/lap_time
                   ├─► /lap/metrics         (JSON)
                   └─► /lap/visualization   (MarkerArray)
```

### Interfaces

| Topic | Type | Direction | Contents |
|---|---|---|---|
| `/state` | `nav_msgs/Odometry` | sim → controller, analyzer | pose `(x, y)`, orientation quaternion, `twist.linear.x = v` |
| `/path` | `nav_msgs/Path` | path_gen → controller, analyzer | 1001 centreline poses |
| `/throttle` | `std_msgs/Float32` | controller → sim | normalised effort in [−1, 1] |
| `/steer` | `std_msgs/Float32` | controller → sim | front-wheel angle in radians, positive left |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop keyboard → bridge | teleoperation only |
| `/telemetry/*` | `std_msgs/Float32` | analyzer → plotting | CTE, speed, heading error, lap time |
| `/lap/metrics` | `std_msgs/String` | analyzer → any | aggregated JSON |
| `/lap/visualization` | `visualization_msgs/MarkerArray` | analyzer → RViz | gate, CTE whisker, projection, 3D HUD |

### Two-tier control structure

Every autonomous mode decomposes the driving task into an independent lateral and longitudinal problem running in the same 10 Hz cycle:

**Tier 1  high-level.** The velocity profiler converts local path curvature into a speed target, and the selected lateral controller converts path geometry into a steering angle.

**Tier 2  low-level.** The longitudinal PID regulates actual speed onto the profiler's target by commanding normalised throttle, compensating for rolling resistance and drag.

MPC is the exception and the reason the architecture is worth describing. It replaces *both* tiers at once: steering and acceleration are jointly optimised decision variables, so the longitudinal PID is bypassed entirely in `mpc` mode. That coupling is why MPC holds the 7.500 m/s cap exactly where the other two overshoot it slightly . it is not chasing a setpoint through a separate loop, it is treating speed error as one term in a single cost function.

### Mode selection

`controller_node.py` reads the `control_mode` parameter at startup and dispatches to one of three branches in `control_loop()`. The path-geometry helpers (`path_window`, `path_tangent`, `path_curvature`, `compute_track_errors`, `build_mpc_reference`) are shared, which is what makes the benchmark a fair comparison: all three controllers see the same track, the same curvature estimates and the same speed policy.

---

## Vehicle and Environment Parameters

| Parameter | Value | Where |
|---|---|---|
| Wheelbase `L` | 1.25 m | `bicycle_model.py` |
| Max steering angle | ±35° (0.6109 rad) | `bicycle_model.py` |
| Throttle authority `k_a` | 4.0 m/s² | `bicycle_model.py` |
| Rolling resistance `c_roll` | 0.05 | `bicycle_model.py` |
| Aerodynamic drag `c_drag` | 0.005 | `bicycle_model.py` |
| Speed cap (profiler) | 7.5 m/s | `controller_node.py` |
| Lateral acceleration limit | 5.0 m/s² | `controller_node.py` |
| Control loop rate | 10 Hz (100 ms) | `controller_node.py` |
| Track | `centerline_0.csv`, 1001 waypoints, 528.20 m | `track_environment` |

The full model and every control law are derived in [Mathematical Formulations](#mathematical-formulations).

---

## Mathematical Formulations

### 1. Plant extended kinematic bicycle

State `x = [x, y, θ, v]ᵀ`, inputs `u = [u_throttle, δ]ᵀ`, reference point at the rear axle:

```
ẋ = v·cos θ
ẏ = v·sin θ
θ̇ = (v / L)·tan δ
v̇ = k_a·u_throttle − c_roll·v − c_drag·v²
```

Integrated by forward Euler at the simulator rate. Velocity is a **state**, not an input, which is what makes a closed-loop speed regulator necessary: commanding throttle does not command speed.

Terminal velocity for a constant throttle `u` follows from `v̇ = 0`:

```
c_drag·v² + c_roll·v − k_a·u = 0
    ⇒  v_∞ = ( −c_roll + √(c_roll² + 4·c_drag·k_a·u) ) / (2·c_drag)
```

### 2. Path geometry

**Arc-length window.** For waypoint index `i` and window `w`, walk backward and forward accumulating *segment* lengths (not radial distance) until `w` metres are covered in each direction, giving `(i_back, i_fwd)`.

**Jitter-rejecting tangent.** The centreline CSV contains sub-metre backtracking, so the bearing between consecutive waypoints is noisy. The tangent is taken as the chord across the arc-length window (`w = 1.5 m`):

```
ψ_path(i) = atan2( y[i_fwd] − y[i_back],  x[i_fwd] − x[i_back] )
```

**Menger curvature** through three arc-spaced points `a = p[i_back]`, `b = p[i]`, `c = p[i_fwd]` (`w = 2.0 m`):

```
κ = 2·|(b−a) × (c−a)| / ( |b−a| · |c−b| · |c−a| )
```

**Cross-track error.** The foot point `(p_x, p_y)` is the closest point on either segment adjacent to the nearest waypoint. The *signed* offset is then taken in the path frame, not from the segment, so the sign cannot flip at a backtracking joint:

```
e_lat = −(x − p_x)·sin ψ_path + (y − p_y)·cos ψ_path
        (positive = left of path)
e_ψ   = atan2( sin(θ − ψ_path), cos(θ − ψ_path) )
        (wrapped to [−π, π])
```

### 3. Velocity profiler

A point mass on a circular arc of curvature `κ` at speed `v` experiences lateral acceleration `a_lat = v²·κ`. Bounding it and clamping to the speed cap:

```
v_target(κ) = min( √(a_lat,max / |κ|),  v_cap )
        a_lat,max = 5.0 m/s²        v_cap = 7.5 m/s
```

### 4. Longitudinal PID with two-stage anti-windup

```
e = v_target − v
u = k_p·e + k_i·I + k_d·(e − e_prev)/Δt
```

Stage 1 conditional integration. Form the candidate `I⁺ = I + e·Δt`, evaluate the unsaturated effort with it, and accept the update only if it is not driving further into saturation:

```
if not ( (u_unsat >  u_max and e > 0)
      or (u_unsat < −u_max and e < 0) ):
    I ← clip(I⁺, −I_lim, +I_lim)   # stage 2 — hard clamp
```

Stage 2 alone is insufficient: a clamped integrator still winds to its limit during saturation and must then unwind before the output can respond. The conditional test prevents the accumulation in the first place.

### 5. Lateral PID

```
δ = −( k_p·e_lat + k_i·∫e_lat dt + k_d·ė_lat + k_yaw·e_ψ )
```

The derivative is taken **analytically, not by finite difference**. For a kinematic bicycle referenced at the rear axle the lateral rate is exactly

```
ė_lat = v·sin(e_ψ)
```

so no differentiation of a noisy estimate is required. Linearising the Frenet error dynamics for small errors,

```
ė_lat = v·e_ψ
ė_ψ   = (v/L)·tan δ − κ·v  ≈  −(v/L)·( k_p·e_lat + k_yaw,eff·e_ψ )
```

gives the second-order characteristic equation `s² + (v/L)·k_yaw,eff·s + (v²/L)·k_p = 0`, hence

```
ω_n = v·√(k_p / L)                    ζ = k_yaw,eff / ( 2·√(k_p·L) )
```

where `k_yaw,eff = k_yaw + k_d·v`, because the analytic derivative term acts on the same signal as the heading term. Damping is therefore **speed-adaptive**, scaling with `v` exactly as the plant gain does.

Steady-state offset on a constant-curvature arc, since a reactive controller must hold an error to generate the corner's steering:

```
e_ss ≈ atan(L·κ) / k_p
```

Third-order stability bound with the integrator included (Routh–Hurwitz):

```
k_i < (v/L)·k_yaw,eff·k_p
```

### 6. Pure Pursuit

Adaptive lookahead, then the arc through the goal point expressed in the vehicle frame:

```
L_d = clip( k_v·v + l_min,  l_min,  l_max )

y_local = −sin θ·(x_t − x) + cos θ·(y_t − y)

κ_arc = 2·y_local / L_d²              δ = atan( L·κ_arc )
```

`y_local` is the lateral offset of the target in the body frame; the curvature expression is the circle through the rear axle and the goal point, tangent to the current heading.

### 7. Extended Kinematic MPC

Decision vector over horizon `N = 10` at `Δt = 0.1 s`:

```
U = [ δ₀, a₀, δ₁, a₁, …, δ_{N−1}, a_{N−1} ]ᵀ  ∈ ℝ²ᴺ
```

Predicted states follow the same plant model as the simulator, drag terms included, so the optimiser anticipates deceleration rather than correcting for it:

```
x_{k+1} = x_k + v_k·cos θ_k·Δt
y_{k+1} = y_k + v_k·sin θ_k·Δt
θ_{k+1} = wrap( θ_k + (v_k/L)·tan δ_k·Δt )
v_{k+1} = clip( v_k + (a_k − c_roll·v_k
                           − c_drag·v_k²)·Δt,  0,  v_max )
```

Tracking error at each step is projected into the reference Frenet frame:

```
Δx = x_k − x_ref,k                    Δy = y_k − y_ref,k
e_long = Δx·cos ψ_ref,k + Δy·sin ψ_ref,k
e_lat  = −Δx·sin ψ_ref,k + Δy·cos ψ_ref,k
e_ψ    = wrap( θ_k − ψ_ref,k )        e_v = v_k − v_ref,k
```

The nonlinear program:

```
minimise  J(U) = Σ_{k=0}^{N−1} [
      w_lat·e_lat²  + w_long·e_long²  + w_yaw·e_ψ²
    + w_v·e_v²      + w_steer·δ_k²
    + w_dsteer·(δ_k − δ_{k−1})²       + w_accel·a_k² ]

subject to   |δ_k| ≤ δ_max = 35°            k = 0 … N−1
             |a_k| ≤ k_a   = 4.0 m/s²
```

with `δ_{−1} = ` the currently commanded steering angle, so the slew-rate term penalises discontinuity against the real actuator state rather than against zero.

Solved with SLSQP (`maxiter = 25`, `ftol = 1e-3`), warm-started by shifting the previous solution forward one step and holding the final control:

```
U_init = [ u*₁, u*₂, …, u*_{N−1},  u*_{N−1} ]
```

Only `(δ₀, a₀)` is applied; acceleration maps to normalised throttle as `u_throttle = a₀ / k_a`. The horizon is re-solved from the measured state at the next cycle — receding-horizon control.

**Reference generation.** The `k`-th reference pose is sampled at arc length `(k+1)·v·Δt` ahead of the nearest waypoint, with heading from the windowed tangent and `v_ref` from the velocity profiler evaluated at that point's curvature — so the MPC's speed references are consistent with the policy the other controllers follow.

---

## Quickstart

### Build

```bash
source /opt/ros/humble/setup.bash

sudo apt update && sudo apt install -y python3-colcon-common-extensions \
  python3-numpy python3-scipy ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro

cd ~/Control_Project
colcon build --symlink-install
source install/setup.bash
```

### Launch Modes

| Mode | Command |
|---|---|
| Base simulation | `ros2 launch bicycle_sim bicycle_sim.launch.py` |
| Keyboard teleoperation | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop` |
| Teleop + cruise control | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true` |
| Lateral PID | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid` |
| Pure Pursuit | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit` |
| Extended Kinematic MPC | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc` |


For teleoperation, start the keyboard driver in a second sourced terminal:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Telemetry

```bash
sudo apt install -y ros-humble-plotjuggler-ros rqt-plot

ros2 run rqt_plot rqt_plot /telemetry/cte/data
ros2 run rqt_plot rqt_plot /telemetry/speed/data
ros2 run plotjuggler plotjuggler
```

Published telemetry: `/telemetry/cte`, `/telemetry/speed`, `/telemetry/heading_err_deg`, `/telemetry/lap_time`, aggregated JSON on `/lap/metrics`, and RViz markers on `/lap/visualization`.

---

## Telemetry Benchmark Leaderboard

Each autonomous controller was run for **at least four complete laps**

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| **Manual Teleoperation** | 249.799 | 4.651 | 1.8217 | 13.9788 | 3.3659 | 1 / completed |
| **Lateral PID (Reactive)** | 66.217 | 7.538 | 0.2422 | 0.7952 | 0.2984 | 4 / completed |
| **Pure Pursuit (Preview)** | **65.001** | 7.537 | 0.0370 | 0.3341 | 0.0608 | 4 / completed |
| **Extended Kinematic MPC (Optimal)** | 66.062 | 7.500 | **0.0214** | 0.3725 | **0.0487** | 4 / completed |

Mean heading error: **MPC 1.74°**, **Pure Pursuit 1.68°**, **Lateral PID 3.84°**, **Teleoperation 12.65°**.

### Reading the table

**MPC wins on tracking accuracy**, which is the metric the benchmark is built around  0.0214 m mean CTE against Pure Pursuit's 0.0370 m. It also holds the speed cap exactly , because the optimiser treats the profiler's reference speed as a cost term rather than a setpoint to be chased by a separate PID.

**Pure Pursuit wins on lap time** by 1.06 s. It carries marginally more speed (6.83 vs 6.73 m/s mean) because it has no explicit speed-error cost pulling it back toward the profile. MPC trades that second of lap time for a 42% reduction in tracking error , a deliberate consequence of the cost weighting, not a deficiency.

**Lateral PID is an order of magnitude behind both**, and the reason is structural rather than a matter of tuning. See the analysis below.

**The human baseline is the one that puts the rest in perspective.** A 249.8 s lap with 1.82 m mean error and a 13.98 m worst-case excursion is what the same vehicle does without a controller at all.

---

## Why MPC > Pure Pursuit > Lateral PID

The ordering is not an accident of tuning. Each controller has access to strictly more information than the one below it.

### Lateral PID sees only the present

The controller computes a steering angle from the instantaneous cross-track error and heading error:

```
δ = −(k_p·e + k_i·∫e dt + k_d·ė + k_yaw·e_ψ)
```

It has no knowledge of what the path does ahead of the vehicle. On a curve it must therefore *generate* its steering from an error it has first committed, and two independent ceilings bind at once.

**Bandwidth is capped by the sample rate.** Linearising the Frenet error dynamics gives

```
ζ  = k_yaw / (2·√(k_p·L))        ω_n = v·√(k_p/L)
```

Raising `k_p` raises `ω_n` as `√k_p`, but the 100 ms zero-order hold contributes phase lag of roughly `ω_n·T/2`. Measured on this vehicle, `k_p = 0.6` (`ω_n = 4.64 rad/s`) produced a loop that diverged lap over lap — mean CTE climbing 0.326 → 0.405 → 0.466 m — despite a continuous-time damping estimate of ζ = 0.84. The empirical ceiling is **k_p ≈ 0.45**.

**Integral action is capped by curvature sign reversals.** On a constant-radius corner the car needs a standing steering angle of `atan(L·κ)`. At the profiler's cruise speed the cornering curvature is `κ = a_lat,max/v² = 5.0/6.7² = 0.111 1/m`, so with `k_p = 0.25` the predicted steady-state offset is `atan(1.25 × 0.111)/0.25 = 0.55 m` — against a measured 0.553 m. An integrator removes that offset, but only for a *constant* disturbance. Track curvature flips sign every few seconds, and each flip forces the accumulated integral to unwind through zero before it can build the opposite sign. That unwind is directly visible as the car pulling wide on corner entry.

Both limits bind simultaneously, which is why Lateral PID settles at **0.242 m** and no further tuning moves it.

### Pure Pursuit gets preview for free

The lookahead point *is* preview. Choosing a goal point `L_d = clip(k_v·v + l_min, l_min, l_max)` metres ahead and steering along the circular arc through it,

```
δ = atan(L · 2·y_local / L_d²)
```

the controller is implicitly responding to where the path *goes*, not only to where the car *is*. No standing error is required to generate corner steering, so the dominant error term in Lateral PID simply does not exist. The result is a **6.5× improvement** with a far simpler control law  0.0370 m against 0.2422 m.

The remaining error is geometric: Pure Pursuit tracks the chord to a single point, so it naturally cuts slightly inside tight corners, and the adaptive lookahead trades corner-cutting against straight-line stability.

### MPC previews with a model and optimises

MPC predicts the full state `[x, y, θ, v]` over a 10-step (1.0 s) horizon using the same extended kinematic model as the plant, including the drag terms, and minimises

```
J = Σₖ  w_lat·e_lat² + w_long·e_long² + w_yaw·e_ψ² + w_v·e_v²
        + w_steer·δ² + w_dsteer·(δₖ − δₖ₋₁)² + w_accel·a²
```

subject to `|δ| ≤ 35°` and `|a| ≤ k_a`, solved with SLSQP at every control cycle.

Three capabilities follow that the others cannot reach:

**It knows the plant.** The prediction includes `−c_roll·v − c_drag·v²`, so the optimiser anticipates deceleration rather than correcting for it afterwards.

**It sees many points, not one.** Where Pure Pursuit commits to a single lookahead target, MPC weighs ten reference poses at once and finds the steering sequence that best satisfies all of them — which is why it does not cut corners the way Pure Pursuit does.

**It handles its own constraints.** Steering and acceleration limits are bounds in the optimisation, not a clamp applied afterwards. The optimiser plans a feasible trajectory instead of discovering saturation after the fact. Steering saturation was measured at **0.15% of control cycles**.

The cost: roughly three orders of magnitude more computation per cycle. That is not free, and the next section is about what happens when it isn't paid.

---

## Real-Time Computation as a Design Constraint

The most instructive result in this project came from an experiment that was not planned.

All three controllers were run back to back on the same machine, in the same session, with and without RViz. RViz is a visualisation tool — it does not touch the control path, so nothing about the dynamics should change. The apparent variable was the visualiser; the real one, as the next section shows, was how much CPU was left for the controller.

| Controller | Mean CTE, RViz **off** | Mean CTE, RViz **on** | Change |
|---|---|---|---|
| Lateral PID | 0.2422 m | 0.2393 m | −1% |
| Pure Pursuit | 0.0370 m | 0.0372 m | +0.5% |
| **Extended Kinematic MPC(sometimes)** | **0.0211 m** | **0.9585 m** | **45× worse** |

MPC's other figures in that run: lap time 85.0 s against 66.0 s, mean heading error 33.3° against 1.66°, and a top speed of 7.833 m/s against a 7.500 m/s cap that it respects exactly when it has the CPU.

**Two controllers were completely indifferent. The third collapsed.**

### RViz is not the variable , headroom is

That first conclusion was wrong, and the way it was found to be wrong is itself the result.

After the collapse above, the machine was reset — every terminal and VS Code closed, then `wsl --shutdown`, then a fresh Ubuntu session — and MPC was launched again **with RViz2 open**, on the same build, with no gain changed. It tracked normally:

| MPC run | RViz | Machine | Mean CTE | Best lap | Heading err |
|---|---|---|---|---|---|
| Headless benchmark | off | fresh | 0.0214 m | 66.062 s | 1.74° |
| **After reset** | **on** | **fresh** | **0.0222 m** | 66.142 s | 1.81° |
| Before reset | on | loaded | 0.9585 m | 85.0 s | 33.3° |

With RViz open on a fresh instance the cost is **4%**, not a factor of 45 and the degradation was fully **reversible by restarting the host**, with the code untouched. That is what rules the visualiser out as the cause. RViz was the last straw on a machine whose WSL2 memory balloon had already grown and whose WSLg compositor had begun to stall, the state flagged by `robot_state_publisher`'s `Moved backwards in time` warning. On a machine with headroom, the same visualiser costs the same solver almost nothing.

The distinction matters for the conclusion. It is not "do not run RViz with MPC"; it is **MPC has no margin to give away, and how much margin a visualiser consumes depends on what else the machine is doing.** The same binary sits well inside its deadline or well outside it according to conditions that have nothing to do with the control problem. Pure Pursuit and Lateral PID are indifferent to all of it.

It is also a reminder about experimental method: the original comparison changed one variable in the launch command but not the one that actually mattered, and held constant a machine state that was silently drifting. The controlled version of the experiment required resetting the host, not editing the code.

The mechanism is the 100 ms deadline. Lateral PID and Pure Pursuit are closed-form arithmetic — a few dozen floating-point operations that will meet their deadline on any hardware. MPC must solve a 20-variable constrained nonlinear program inside every cycle. When the CPU is contended, SLSQP returns whatever partially converged iterate it has reached when the budget runs out, and the vehicle is steered by a solution to a problem it has already driven past.

Profiling the objective function confirmed the diagnosis. The original implementation allocated a NumPy array at every horizon step and recomputed `cos(ψ_ref)` and `sin(ψ_ref)` on every call  but the references are constant during a solve, and SLSQP evaluates the objective several hundred times per cycle. Hoisting those computations out of the inner loop and working in scalar floats:

**The engineering conclusion:** MPC's accuracy is purchased with computation, and the purchase has a hard real-time price. It is 1.7× more accurate than Pure Pursuit when it has the CPU, and 26× *worse* when it does not — and whether it does is not a property of the controller but of the whole machine at that moment. Pure Pursuit never flinches either way. On an embedded target with a fraction of this machine's compute, that trade-off would drive the controller choice far more than the accuracy figures in the leaderboard.

---

## Controller Tuning

### Longitudinal PID (Milestone 4)

`k_p = 1.2, k_i = 0.3, k_d = 0.05`, with two-stage anti-windup: conditional integration (the accumulator is frozen when the unsaturated effort would push further into saturation) followed by a hard clamp at ±2.0. Output saturates to [−1, 1], positive being throttle and negative braking.

### Velocity Profiler (Milestone 5.1)

`v_max(κ) = min(√(a_lat,max / |κ|), v_cap)` with `a_lat,max = 5.0 m/s²` and `v_cap = 7.5 m/s`. Curvature is estimated with the Menger three-point circle over an arc-length window of ±2.0 m.

### Lateral PID (Milestone 5.2)

`k_p = 0.45, k_i = 0.4, k_d = 0.1, k_yaw = 0.55`

Gains were derived from the target closed-loop dynamics rather than by trial and error. With the analytic derivative term (below), the total damping coefficient is `k_yaw + k_d·v = 0.55 + 0.1 × 6.8 = 1.23` at cruise, giving ζ = 0.80 and ω_n = 3.0 rad/s. The damping is therefore speed-adaptive, scaling with velocity exactly as the plant gain does.

> **Note on Ziegler–Nichols.** Z–N was evaluated and rejected. It is derived for self-regulating first-order-plus-dead-time process loops; this plant is a double integrator from steering to lateral position, its loop gain is speed-dependent, and the classic quarter-amplitude-decay rule targets ζ ≈ 0.4 — precisely the underdamped weaving the tuning was intended to eliminate. The damping-ratio method above is derived from the actual linearised Frenet error dynamics and every gain traces to a target ζ and ω_n.

### Pure Pursuit (Milestone 5.3)

`k_v = 0.25, l_min = 0.8 m, l_max = 2.5 m` — the assignment reference values. Raising `l_max` to 4.0 m was measured and found marginally worse.

### Extended Kinematic MPC (Milestone 5.4)

Horizon `N = 10`, `dt = 0.1 s`, SLSQP with `maxiter = 25`, `ftol = 1e-3`.

| Weight | Value | Role |
|---|---|---|
| `w_lat` | 50.0 | lateral cross-track error |
| `w_long` | 1.0 | along-path error |
| `w_yaw` | 10.0 | heading error |
| `w_v` | 5.0 | speed error |
| `w_steer` | 0.2 | steering magnitude |
| `w_dsteer` | 4.0 | steering rate (damps oscillation) |
| `w_accel` | 0.1 | acceleration magnitude |

`maxiter` and `ftol` are held at the assignment values deliberately: raising them pushes the solve past the 100 ms deadline, and the resulting stale commands destabilise the vehicle. An attempt at `maxiter = 50, ftol = 1e-4` produced a car that circled and reversed.

Raising `w_lat` to 150 and `w_v` to 20 was also tried, on the reasoning that heavier tracking penalties should tighten tracking. They did the opposite — mean CTE 0.72 m and a failed lap — because an ill-conditioned cost surface cannot be solved to useful accuracy in 25 iterations. **Within a fixed iteration budget, weight magnitudes trade against solvability.**

---

## Engineering Notes: Four Defects Found by Measurement

Four issues were diagnosed during development. Each was found by instrumenting and measuring rather than by inspection, and each is recorded here with the evidence.

### 1. Arc-length window walked the wrong pointer

`path_window()` accumulated backward distance using the *fixed* centre index instead of the *moving* backward pointer:

```python
seg = math.hypot(pts[idx][0]  - pts[prev_i][0], ...)   # wrong
seg = math.hypot(pts[back_idx][0] - pts[prev_i][0], ...)   # correct
```

This measured radial distance from one fixed point rather than arc length along the path, so the backward window terminated almost immediately and `path_tangent()` received a degenerate window. Because the tangent feeds the MPC's entire Frenet frame, the effect was severe: MPC mean CTE 0.72 m, heading error 39°, and visible circling.

### 2. MPC reference heading used raw waypoint yaw

`get_waypoint_at_distance()` returned the raw yaw stored on each waypoint. The centreline CSV contains sub-metre backtracking — **32.5% of waypoint joints bend by more than 20°** — so that angle is noisy. MPC builds its whole Frenet frame from it, corrupting both the lateral/longitudinal error split and the heading cost. Replacing it with the arc-length-windowed tangent reduced heading error from 4.59° to roughly 1.7°.

### 3. Cross-track error flipped sign 253 times per run

This one was invisible in the summary statistics and only appeared under instrumentation. Logging CTE, heading error, curvature and steering at 10 Hz to CSV and analysing 2,824 samples showed:

```
+1.2593 m  →  −1.1292 m   in one 100 ms tick (the car moved 0.70 m)
```

**253 of 2,774 samples (9.1%)** changed CTE by more than the vehicle could physically travel. All were sign flips with the magnitude preserved. The cause was taking the sign from an individual path segment: at the backtracking joints the two candidate segments point in substantially different directions, so whichever one won the distance comparison determined the sign — and the winner alternated between ticks.

The fix was to take the magnitude from the nearest segment's foot point but the **sign from the arc-length-windowed tangent**, projecting into the path frame:

```python
cte = −(x − p_x)·sin(ψ_path) + (y − p_y)·cos(ψ_path)
```

Because `ψ_path` is averaged over ±1.5 m, the sign is continuous from cycle to cycle.

### 4. The derivative term amplified that noise tenfold

Decomposing the steering command revealed that the D term was the *largest* contributor and almost entirely noise:

| term | mean &#124;·&#124; | max &#124;·&#124; |
|---|---|---|
| P (k_p·e) | 8.45° | 33.19° |
| I (k_i·∫e) | 6.99° | 22.92° |
| **D (k_d·ė)** | **11.43°** | **136.85°** |
| YAW (k_yaw·e_ψ) | 8.43° | 39.51° |

A 136° steering demand from a 35° actuator. The measured `d(CTE)/dt` peaked at **23.9 m/s** — 3.5× faster than the car's own 6.8 m/s — because a finite difference at 10 Hz multiplies the sign flips by `1/dt = 10`.

For a kinematic bicycle referenced at the rear axle the lateral rate is available in closed form, `ė = v·sin(e_ψ)`, so differentiating a noisy estimate is unnecessary:

```python
derivative_cte = v * math.sin(heading_err)
```

### Measured effect of fixes 3 and 4

| | before | after |
|---|---|---|
| Impossible CTE jumps | 253 (9.12%) | **0** |
| Large sign flips | 238 | **0** |
| Steering saturated | 501 cycles (18.06%) | **4 (0.15%)** |
| max &#124;d(CTE)/dt&#124; | 23.885 m/s | **3.071 m/s** |
| Median oscillation half-period | 0.20 s | **2.80 s** |
| Mean &#124;steer&#124; | 13.88° | **6.09°** |
| Mean CTE | 0.3272 m | **0.2424 m** |
| Mean heading error | 7.24° | **3.84°** |
| Distance per lap | 460.5 m | **453.3 m** |

Two of these deserve comment. The **0.20 s half-period** was two control samples — the Nyquist limit of a 10 Hz loop, meaning the controller was fighting its own sensor rather than oscillating dynamically; the designed ω_n of 3 rad/s would oscillate near 0.5 Hz. Afterwards it is 2.80 s, slower than the designed dynamics, so what remains is the car following the track's curvature rather than any control oscillation.

And **mean steering of 6.09°** should be compared with the steering the corners physically demand, `atan(L·κ̄) = atan(1.25 × 0.081) = 5.8°`. The controller is now doing almost exactly the work the track requires, instead of spending 14° per cycle rejecting noise it was generating itself.

---
## Reproduction Guide

Every figure in this report can be regenerated from a clean checkout. The procedure below is the one used to produce the leaderboard.

### Step 1 — Environment

ROS 2 Humble on Ubuntu 22.04 (tested under WSL2 on Windows 11).

```bash
source /opt/ros/humble/setup.bash
sudo apt update && sudo apt install -y python3-colcon-common-extensions \
  python3-numpy python3-scipy ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro ros-humble-teleop-twist-keyboard \
  ros-humble-plotjuggler-ros rqt-plot
```

### Step 2 — Build and verify

```bash
cd ~/Control_Project
colcon build --symlink-install
colcon test --packages-select bicycle_control bicycle_sim track_environment
colcon test-result --verbose
```

Expected: `26 tests, 0 errors, 0 failures, 3 skipped`. Do not proceed if the style tests fail — a build that passes but lints dirty indicates an edit was not saved.

### Step 3 — Prepare a quiet machine

**This step is not optional, and skipping it will not reproduce the MPC figures.**

```powershell
wsl --shutdown          # from PowerShell, not inside WSL
```

Reopen a plain Ubuntu terminal. Close VS Code and any other heavy application. WSL2's memory balloon grows with uptime and WSLg begins to stall; MPC is sensitive to this and the other controllers are not.

### Step 4 — Run each controller

```bash
cd ~/Control_Project && source install/setup.bash

ros2 launch bicycle_sim bicycle_sim.launch.py \
    controller:=lateral_pid rviz:=false
ros2 launch bicycle_sim bicycle_sim.launch.py \
    controller:=pure_pursuit rviz:=false
ros2 launch bicycle_sim bicycle_sim.launch.py \
    controller:=mpc
```

Let each run complete at least four `LAP COMPLETED` banners, then `Ctrl-C`.


### Step 5 — Manual teleoperation baseline

Terminal 1:
```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop
```

Terminal 2 (keep this window focused while driving):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Press `c` roughly ten times first. `teleop_twist_keyboard` sends `angular.z = ±1.0` by default and the bridge maps that to full 35° lock, which is undrivable; ten presses scale it to about 12°. Drive with `i` (forward), `u` (forward-left), `o` (forward-right), holding the key down — the bridge's 0.5 s watchdog zeroes the commands if no message arrives, which is the Milestone 3 safety feature working as specified.

### Step 6 — Aggregate

Take the mean of the settled laps (lap 2 onward) for Mean CTE, RMS CTE, mean speed and heading error. Take the minimum lap time for Best Lap and the maximum of `Max speed` across laps for Top Speed.

### Step 7 — Telemetry figures

With a controller running, in a second sourced terminal:

```bash
ros2 run rqt_plot rqt_plot /telemetry/cte/data
ros2 run rqt_plot rqt_plot /telemetry/speed/data
```

Start the plotter **after** lap 1 completes, or the standing-start transient dominates the autoscale. Fix the y-axis to −1.0 … +1.0 on all three CTE plots so they are directly comparable.

### Expected variation

The simulator is deterministic and settled laps repeat to within a few milliseconds:

| Controller | settled lap times (s) | spread |
|---|---|---|
| MPC | 66.075 / 66.062 / 66.164 | 102 ms |
| Pure Pursuit | 65.005 / 64.999 / 65.002 | 6 ms |
| Lateral PID | 66.500 / 66.401 | 99 ms |

Agreement with the table above to roughly ±0.005 m of CTE indicates a faithful reproduction. A large discrepancy in the **MPC row only** almost always means step 3 was skipped.

---

## Testing

```bash
colcon test --packages-select bicycle_control bicycle_sim track_environment
colcon test-result --verbose
```

**26 tests, 0 errors, 0 failures, 3 skipped** across all three packages, including `flake8`, `pep257` and copyright checks.

---

## Repository Structure

```
Control_Project/
├── bicycle_sim/                  # Simulator and vehicle model
│   └── bicycle_sim/
│       ├── bicycle_model.py      # M2: bicycle model + Euler
│       └── sim_node.py
├── bicycle_control/              # Controllers
│   └── bicycle_control/
│       ├── teleop_bridge.py      # M3: cmd_vel bridge + watchdog
│       ├── longitudinal_pid.py   # M4: speed PID + anti-windup
│       ├── velocity_profiler.py  # M5.1: curvature speed profile
│       ├── lateral_pid.py        # M5.2: reactive PID + heading
│       ├── pure_pursuit.py       # M5.3: adaptive lookahead
│       ├── mpc.py                # M5.4: SLSQP nonlinear MPC
│       └── controller_node.py    # modes, path geometry, references
└── track_environment/            # Track, path and analysis
    └── track_environment/
        ├── track.py
        ├── path_gen.py
        └── lap_analyzer.py       # M5.5: metrics, telemetry, HUD
```

---

## Milestone 6 — Free Exploration: Nav2 MPPI Control

Of the three options offered (four-wheel Ackermann kinematics, 3D simulation, Nav2 MPPI), **Nav2's Model Predictive Path Integral controller** was chosen because it sits directly opposite this project's own MPC: both are receding-horizon predictive controllers, and they differ in exactly the dimension this project measured by accident — how a predictive controller spends its compute budget.

All parameter values below are taken from the **ROS 2 Humble** reference, matching this project's distribution. Defaults have drifted between distributions (the current documentation quotes 100+ Hz and `wz_std = 0.2` where Humble quotes 50+ Hz and `0.4`), so version-matched sources matter. [1][2]

### 6.1 MPPI is MPC, with a different search

MPC is a *strategy* — predict, optimise a finite input sequence, apply the first input, re-plan — not a solver. SLSQP, interior-point methods, and MPPI are all ways of attacking that problem. The accurate comparison is therefore **gradient-based MPC versus sampling-based MPC**, not "MPC versus MPPI". [5]

MPPI maintains a nominal control sequence `U = [u₀ … u_{N−1}]`, and on each cycle:

**1. Sample.** Draw `K` perturbed sequences, `uₖ,ₜ = uₜ + εₖ,ₜ`, with `ε ~ N(0, Σ)`, clipped to the motion model's limits.

**2. Roll out.** Propagate each candidate through the motion model to obtain `K` predicted trajectories.

**3. Score.** Accumulate a cost `Sₖ = Σₜ c(xₖ,ₜ, uₖ,ₜ)` per candidate from the enabled critics.

**4. Weight.** Convert costs to path-integral weights, with the minimum subtracted for numerical stability:

```
wₖ = exp( −(Sₖ − S_min) / λ )  /  Σⱼ exp( −(Sⱼ − S_min) / λ )
```

`λ` (the `temperature`) controls selectivity: small `λ` concentrates weight on the best candidates, large `λ` spreads it across the batch.

**5. Update and apply.** `uₜ ← uₜ + Σₖ wₖ·εₖ,ₜ`, then emit `u₀`, shift the sequence forward, and warm-start the next cycle.

The decisive property is in step 3. **Nothing is differentiated**, so the cost need not be smooth or convex. A binary collision penalty, a costmap lookup, a piecewise rule — all admissible. That is precisely what a gradient solver cannot accept.

It is *not*, however, "simulate everything and pick the best". The update is a cost-weighted average of perturbations, and it carries no global-optimality guarantee: sampling can miss a useful region of the control space entirely, and a wrong motion model produces confidently wrong rollouts. [5][6]

### 6.2 How Nav2 packages it

`nav2_mppi_controller` is a `nav2_core::Controller` plugin running inside Nav2's Controller Server. It consumes a global plan, the robot pose and velocity, TF, and the local costmap, and publishes a velocity command. It is a *local* controller, not a navigation system — it depends on the rest of Nav2 around it. [1][2]

Two architectural choices distinguish it from a hand-written MPC objective:

**Motion models** — `DiffDrive`, `Omni`, `Ackermann` — encode which motions are feasible. The Ackermann model enforces a minimum turning radius.

**Critics are plugins**, selected and weighted by configuration rather than compiled into one cost function: `ConstraintCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`, `ObstaclesCritic`, `CostCritic`, `GoalCritic`, `GoalAngleCritic`, `PreferForwardCritic`, `TwirlingCritic`, `VelocityDeadbandCritic`. Behaviour is *composed* rather than rewritten. [1]

This project's MPC has its seven cost terms hard-coded in `objective()`. Adding obstacle avoidance would mean editing that function; in Nav2 it means enabling a critic and setting a weight. The flexibility is real, but it relocates the difficulty into critic selection, weight balancing, costmap quality and system integration.

### 6.3 Humble parameters and the compute budget

| Parameter | Humble default | Effect of increasing |
|---|---|---|
| `batch_size` | 1000 | better sampling coverage, linear cost increase |
| `time_steps` | 56 | longer preview, linear cost increase |
| `model_dt` | 0.05 s | 56 × 0.05 = **2.8 s horizon** |
| `iteration_count` | 1 | docs advise raising `batch_size` first |
| `temperature` (λ) | 0.3 | spreads weight across more candidates |
| `gamma` | 0.015 | control/update trade-off term |
| `vx_std`, `vy_std`, `wz_std` | 0.2, 0.2, 0.4 | wider exploration, more infeasible samples |
| `vx_max` | 0.5 m/s | a mobile-robot default, **not** a racing value |
| `motion_model` | DiffDrive | must match the base |
| `visualize` | false | debugging aid; measurably slows the controller |

The rollout workload is `batch_size × time_steps` trajectory-step evaluations per update:

```
1000 × 56 = 56,000 step evaluations per cycle, before critic evaluation
```

Compare this project's SLSQP MPC. With 20 decision variables, finite-difference gradients cost ≈21 objective evaluations per iteration, at `maxiter = 25`, each evaluating a 10-step horizon:

```
25 × 21 × 10 ≈ 5,250 step evaluations per cycle
```

**MPPI does roughly 10× more work per cycle — and runs 5× faster.** Nav2 reports 50+ Hz on a 4th-generation Intel i5; this project's MPC measured 23.8 ms (≈42 Hz ceiling) on considerably newer hardware. The difference is not algorithmic: Nav2's implementation is *"highly optimized CPU-only ... using vectorization and tensor operations"* in C++, where this MPC is interpreted Python calling SciPy. [1][2]

That comparison is worth stating plainly because it cuts against the obvious intuition. **Sampling-based control is not inherently more expensive than gradient-based control**; the implementation substrate dominates. It also corrects a common misreading: the well-known MPPI racing demonstrations come from AutoRally, which used CUDA — but Nav2's implementation explicitly targets CPU and needs no GPU. [4][8]

### 6.4 Compared with this project's SLSQP MPC

| | This project (SLSQP MPC) | Nav2 MPPI (Humble) |
|---|---|---|
| Search | gradient-based constrained optimisation | cost-weighted sampling of perturbations |
| Variables | front steering `δ`, acceleration `a` | motion-model command space (velocity) |
| Horizon | 10 × 0.1 s = 1.0 s | 56 × 0.05 s = 2.8 s |
| Cost | 7 fixed quadratic terms in `objective()` | composable critic plugins |
| Differentiability | required | **not required** |
| Obstacles | none modelled | costmap critics, when configured |
| Constraints | bounds in the decision vector | motion model + constraint critic |
| Output | `/steer` (rad) + `/throttle` ∈ [−1, 1] | `Twist` velocity command |
| Optimality | local, deterministic | no global guarantee, stochastic |
| Integration here | native | requires Nav2 stack + command adapter |

### 6.5 What integration would actually require

Nav2 MPPI is **not** a drop-in replacement for this controller, and the reason is the interface rather than the algorithm. It emits a `Twist`; this vehicle consumes a steering angle and a normalised powertrain effort. Bridging that needs:

**A command adapter.** For a kinematic bicycle, `ω = (v/L)·tan δ`, so `δ = atan(ω·L / v)` — with a guard near `v = 0`, where the mapping is singular, plus steering saturation and sign-convention handling. The linear velocity would become the longitudinal PID's setpoint.

**Ackermann configuration.** Minimum turning radius from this geometry:

```
R_min = L / tan(δ_max) = 1.25 / tan(35°) ≈ 1.79 m
```

**A closed-loop path.** Nav2 plans toward a finite goal. A racing lap has no terminal state, so lap looping must be handled deliberately or the controller will stop at the end of the plan.

**A local costmap**, if obstacle critics are to mean anything — plus correct TF, footprint and inflation.

That is an integration project in its own right, which is why this section documents MPPI as a reference architecture rather than claiming a result.

### 6.6 Would this project benefit?

On a fixed, obstacle-free centreline, probably not. The task is path tracking and curvature-aware speed management, which this project's MPC already does to 0.0214 m mean CTE. Obstacle critics would add computation and tuning burden for a hazard that does not exist, and MPPI's 2.8 s default horizon is tuned for mobile robots at 0.5 m/s, not a vehicle at 7.5 m/s.

MPPI becomes the right tool when the cost stops being differentiable: other vehicles on track, a blocked racing line, track limits as hard constraints, or an objective expressing preferences that no quadratic form captures. That is a different problem from the one benchmarked here.

### 6.7 What this project's own measurement contributes

This exploration connects directly to the compute finding documented earlier. This project measured a **45× degradation in MPC tracking accuracy** when CPU was contended, against **no measurable effect** on Pure Pursuit or Lateral PID. Predictive control's accuracy is bought with computation, and the purchase has a hard real-time price.

Nav2's design can be read as an answer to exactly that constraint. Its authors did not reduce the work — 56,000 rollout steps dwarfs this project's 5,250 — they made the work cheap, through vectorised CPU implementation, a single optimiser iteration by default, and the explicit advice to raise `batch_size` before `iteration_count`. Sampling parallelises; gradient descent does not. **An embarrassingly parallel algorithm with a 10× larger workload can be the cheaper engineering choice, because it maps onto hardware that a sequential one cannot use.**

That is the substantive lesson, and it is one this project stumbled into from the opposite direction: a deadline-missing Python SLSQP solver on a loaded laptop.

### 6.8 Scope boundary

Nav2 MPPI was **studied, not implemented**. No Nav2 stack was installed, no MPPI controller was run, and no MPPI figures appear in the benchmark. All numbers attributed to MPPI in this section are documented reference values from Nav2's Humble documentation, clearly distinguished from this project's own measurements.

### References

| | Source |
|---|---|
| [1] | [nav2_mppi_controller README — ROS 2 Humble API](https://api.nav2.org/nav2-humble/html/md_nav2_mppi_controller_README.html) |
| [2] | [nav2_mppi_controller source, Humble branch](https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller) |
| [3] | [Nav2 MPPI configuration guide](https://docs.nav2.org/configuration/packages/configuring-mppic.html) |
| [4] | [AutoRally MPPI project wiki (CUDA/GPU implementation)](https://github.com/AutoRally/autorally/wiki/Model-Predictive-Path-Integral-Controller-%28MPPI%29) |
| [5] | [Williams et al. (2017), *Information Theoretic MPC: Theory and Applications to Autonomous Driving*](https://arxiv.org/abs/1707.02342) |
| [6] | [Kazim et al. (2023), *Recent Advances in Path Integral Control for Trajectory Optimization*](https://arxiv.org/abs/2309.12566) |
| [7] | [Yin et al. (2023), *Shield Model Predictive Path Integral*](https://arxiv.org/abs/2302.11719) |
| [8] | [Macenski, *On Use of Nav2 MPPI Controller*, ROSCon 2023](https://vimeo.com/879001391) |
| [9] | [Nav2 MPPI release announcement and maintainer discussion](https://discourse.ros.org/t/nav2-model-predictive-path-integral-mppi-controller-now-available/30054) |
| [10] | [nav2_mppi_controller package index](https://index.ros.org/p/nav2_mppi_controller/) |

---

## Milestone 8 — Video Walkthrough

**Video:** [3-5 minute technical walkthrough](https://drive.google.com/drive/folders/1-0gAdsuxdqZRbql0gahbdlFq7_0ZDuLo)

