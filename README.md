# H₂-Electric Propulsion Optimizer
### Optimal Energy Management of a Hydrogen–Electric Aircraft Propulsion System
**Colorado State University — M.Eng Aerospace Engineering — Dec 2025**

---

## Project Overview

This project formulates and solves an optimal energy management problem for a 
simplified hydrogen-electric aircraft powertrain using stiff ODE modeling, 
direct collocation, and nonlinear programming. A lumped-parameter model of a 
PEM fuel cell stack and Thevenin battery is developed as a 3-state stiff ODE 
system and simulated over a representative 30-minute regional mission profile.

The core question: **how should power be split between the fuel cell and battery 
to minimize hydrogen consumption while protecting battery health?**

| Parameter | Value |
|---|---|
| Mission duration | 1800 s (30 min) |
| State variables | SOC(t), v_RC(t), P_fc(t) |
| Control input | u(t) ∈ [0,1] — fuel cell power fraction |
| ODE solver | MATLAB ode15s (BDF/variable-order, stiff) |
| Optimization method | Direct trapezoidal collocation + fmincon (interior-point) |
| Collocation segments | N = 40 |
| Decision variables | 4(N+1) = 164 |
| Implementation | MATLAB (single-file, fully reproducible) |

---

## Mission Profile

| Segment | Time (s) | Duration (s) | Power (kW) |
|---|---|---|---|
| Climb | 0 – 300 | 300 | 80 |
| Cruise | 300 – 1200 | 900 | 50 |
| Descent | 1200 – 1500 | 300 | 30 |
| Approach | 1500 – 1800 | 300 | 40 |

---

## Physical Model

### System Architecture
```
Hydrogen Tank → PEM Fuel Cell Stack → Power Split Controller → Electric Motor
                                              ↕
                              Li-ion Battery (Thevenin Model)
```

### State Variables
```
x(t) = [SOC(t)    — battery state of charge
        v_RC(t)   — RC branch voltage (transient polarization)  
        P_fc(t)]  — fuel cell electrical power (first-order lag)
```

### Key Equations

**Fuel cell power dynamics (first-order lag):**
```
τ_fc · dP_fc/dt = -P_fc(t) + P_fc,cmd(t)
```

**Battery SOC dynamics:**
```
dSOC/dt = -I_bat(t) / C_bat
```

**RC voltage dynamics:**
```
τ_RC · dv_RC/dt = -v_RC(t) + R1 · I_bat(t)
```

**Power balance:**
```
P_bat(t) = [P_prop(t) - P_fc(t)] / η_bat
```

**Hydrogen mass flow (power-based estimate):**
```
ṁ_H2(t) = P_fc(t) / (η_stack · LHV_H2)
```

---

## Optimal Control Formulation

**Objective functional** (hydrogen consumption + battery current penalty):
```
J[u(·)] = ∫₀ᵀ [w_H · ṁ_H2(t) + w_I · I²_bat(t)] dt
```

With weights: w_H = 1.0, w_I = 1×10⁻⁴

**Constraints:**
- Dynamic: ẋ(t) = f(x(t), u(t), t) with x(0) = x₀
- Path: SOC_min ≤ SOC(t) ≤ SOC_max, |I_bat(t)| ≤ I_bat,max
- Terminal: SOC(T) ≥ SOC_target = 0.6
- Control: u(t) ∈ [0, 1]

---

## Collocation Implementation

The time interval [0, T] is partitioned into N = 40 uniform segments. 
State and control are approximated at nodes {t_k}, giving a decision 
vector of dimension 4(N+1) = 164.

**Trapezoidal collocation defect** (enforced as equality constraint):
```
x_{k+1} - x_k - (Δt/2)[f(x_k, u_k, t_k) + f(x_{k+1}, u_{k+1}, t_{k+1})] = 0
```

This is mathematically equivalent to the Crank–Nicolson / trapezoid 
time-stepping method — a second-order A-stable scheme.

---

## Key Results

### Baseline vs Optimal Comparison

| Metric | Baseline (u₀ = 0.6) | Collocation Optimal | Change |
|---|---|---|---|
| Total H₂ consumed | 0.751 kg | 1.062 kg | +41% |
| Minimum SOC | 0.135 | 0.555 | +311% |
| Peak battery current | 84.4 A | 66.1 A | -22% |
| Terminal SOC | ~0.135 | ~0.555 | ✅ Constraint met |

### Core Trade-off

The baseline strategy is hydrogen-efficient but drives the battery to 
dangerous depth-of-discharge (SOC = 0.135 vs constraint 0.20). The 
optimal collocation strategy burns ~40% more hydrogen but keeps the 
battery in a healthy operating window throughout the entire mission — 
directly addressing the fundamental hydrogen consumption vs battery 
health trade-off.

### Solver Performance (fmincon Interior-Point)

| Parameter | Value |
|---|---|
| Decision variables | 164 |
| Equality constraints (defects) | 123 |
| Inequality constraints | 83 |
| Major iterations | 19 |
| Function evaluations | 3,135 |
| Reported objective at termination | J ≈ 1.14 × 10² |
| Termination reason | Max function evaluations reached |

---

## Sensitivity Analysis

The cost functional structure makes qualitative sensitivity clear:

- **Increase w_H** → penalizes hydrogen more → optimizer leans toward 
  battery → lower H₂, deeper SOC swing, higher peak current
- **Increase w_I** → penalizes battery current → more conservative 
  battery operation → higher H₂, flatter SOC trajectories
- **Tighten SOC_target** → forces more fuel cell usage → higher H₂, 
  better battery protection
- **Loosen SOC_target** → expands feasible set → optimizer can trade 
  more depth-of-discharge for H₂ savings

---

## Tools & Methods

| Tool | Application |
|---|---|
| **MATLAB ode15s** | Stiff ODE integration (BDF/variable-order, A-stable) |
| **MATLAB fmincon** | Nonlinear programming (interior-point algorithm) |
| **Direct trapezoidal collocation** | Optimal control discretization |
| **Thevenin battery model** | Li-ion battery dynamics |
| **PEM fuel cell polarization model** | Fuel cell V-I characteristics |

---

## How to Run

The entire project is implemented in a single MATLAB script for 
full reproducibility.
```matlab
% Place MECH568_final_project.m in your MATLAB working directory
% Run from the command window:
MECH568_final_project
```

The script will:
1. Run the baseline simulation (ode15s, u₀ = 0.6)
2. Solve the collocation optimal control problem (fmincon)
3. Re-simulate with optimal power split
4. Print comparison metrics to the command window
5. Generate Figure 1 (baseline) and Figure 2 (baseline vs optimal)

---

## Skills Demonstrated

`MATLAB` `Stiff ODE Solving` `ode15s` `Direct Collocation`
`Nonlinear Programming` `fmincon` `Optimal Control` `Energy Management`
`PEM Fuel Cell Modeling` `Thevenin Battery Model` `Hydrogen Propulsion`
`Trapezoidal Collocation` `Sensitivity Analysis` `Aerospace Propulsion`

---

## Files

| File | Description |
|---|---|
| `MECH568_Final_Project.pdf` | Full technical report with complete MATLAB code (Appendix A) |

---

## References

1. Soleymani et al. *Hydrogen propulsion systems for aircraft: A review.* 
   Int. J. Hydrogen Energy, 91:137–165, 2024.
2. Onori, Serrao, Rizzoni. *Hybrid Electric Vehicles: Energy Management 
   Strategies.* Springer, 2016.
3. Ascher & Petzold. *Computer Methods for ODEs and DAEs.* SIAM, 1998.
4. Betts. *Practical Methods for Optimal Control.* SIAM, 2010.

---

## Author

**Faisal Zohad Sayyed**
M.Eng Aerospace Engineering — Colorado State University
[LinkedIn](https://www.linkedin.com/in/faisalzsayyed/) |
[GitHub](https://github.com/faisalzsayyed)
