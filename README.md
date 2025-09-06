# Sliding Mode Control (SMC) for PX4 Multicopter Rate Control

## Motivation

Boundary- Layer Sliding Mode Control (SMC) is a robust control strategy designed to handle nonlinear systems with uncertainties and disturbances. For multicopters, SMC offers:

- Strong robustness to modeling errors and external disturbances (e.g., wind gusts).  
- Reduced sensitivity to parameter variations in motor dynamics.  
- Ability to enforce desired angular rate tracking with fast convergence.  

The goal of this implementation is to replace the traditional PID-based inner-loop rate controller with a **Sliding Mode Controller**, ensuring faster, robust, and precise angular rate tracking.

---

## Features

- **Sliding Mode Control (SMC) for roll, pitch, and yaw axes**.  
- **Boundary layer implementation** to reduce chattering.  
- Configurable **gain parameters** for sliding surface (`L`) and switching term (`k`).  
- **Torque saturation limits** to ensure safe operation in SITL and real drones.  
- Auto-zero torque when the drone is landed to prevent sudden impulses.

---

## Mathematical Formulation

### 1. Tracking Error

The angular rate tracking error is defined as:

\[
\mathbf{e} = \mathbf{\omega}_{\text{sp}} - \mathbf{\omega}
\]

Where:  
- \(\mathbf{\omega}_{\text{sp}}\) — desired angular rates `[rad/s]`  
- \(\mathbf{\omega}\) — measured angular rates `[rad/s]`  

---

### 2. Sliding Surface

The sliding surface is constructed using a proportional term:

\[
\mathbf{s} = \mathbf{L} \odot \mathbf{e}
\]

Where:  
- \(\mathbf{L} = [L_{\text{roll}}, L_{\text{pitch}}, L_{\text{yaw}}]\) — sliding surface gains.  
- \(\odot\) — element-wise multiplication.  

---

### 3. Sliding Mode Control Law

The SMC torque command is given by:

\[
\boldsymbol{\tau} = \mathbf{L} \odot \mathbf{e} - \mathbf{k} \odot \text{sign}(\mathbf{s})
\]

Where:  
- \(\mathbf{k} = [k_{\text{roll}}, k_{\text{pitch}}, k_{\text{yaw}}]\) — switching term gains.  
- `sign(s)` is applied element-wise.  
- To reduce chattering, a **boundary layer** around \(s = 0\) can be introduced:

\[
\text{sign}(s) \approx 
\begin{cases} 
\frac{s}{\phi} & \text{if } |s| < \phi \\
\text{sign}(s) & \text{otherwise}
\end{cases}
\]

Where \(\phi\) is the boundary layer thickness.  

---

### 4. Safety and Limits

To prevent instability, the torque command is **saturated**:

\[
\tau_i = \text{constrain}(\tau_i, -\tau_{\text{max}}, \tau_{\text{max}})
\]

Where \(\tau_{\text{max}}\) is defined based on the UAV type (e.g., `Iris`).

---

## Parameters

| Parameter             | Description                                                   | Typical Value |
|-----------------------|---------------------------------------------------------------|---------------|
| `MC_USE_SMC`          | Enable or disable SMC controller                               | 0 or 1        |
| `MC_SMC_L_ROLL`       | Sliding surface gain for roll axis                              | 1.0           |
| `MC_SMC_L_PITCH`      | Sliding surface gain for pitch axis                             | 1.0           |
| `MC_SMC_L_YAW`        | Sliding surface gain for yaw axis                               | 1.0           |
| `MC_SMC_K_ROLL`       | Switching term gain for roll axis                               | 0.1           |
| `MC_SMC_K_PITCH`      | Switching term gain for pitch axis                              | 0.1           |
| `MC_SMC_K_YAW`        | Switching term gain for yaw axis                                | 0.1           |
| Torque saturation      | Maximum allowable torque output per axis                        | UAV-specific  |

> **Note:** Gains should be tuned carefully; too high leads to flipping, too low leads to sluggish response.  

---

## Getting Started with SMC in SITL

1. **Enable SMC:**  
   ```shell
   param set MC_USE_SMC 1
## 2. Set Initial Gains
Start with low values for L and k to avoid aggressive reactions on takeoff.

## 3. Set Torque Limits
Adjust torque saturation to realistic values for your vehicle. For Iris:
- **Roll/Pitch:** ±0.25–0.3
- **Yaw:** ±0.15–0.2

## 4. Takeoff Carefully
Observe initial behavior and gradually increase gains. Avoid maximum values initially.

## 5. Tune Iteratively
- Increase L for faster error convergence
- Increase k for stronger switching action
- Ensure torque limits remain within safe bounds

## 6. Monitor Stability
- If the drone flips, reduce k first, then L
- Check for oscillations in hover; may require slight reduction of L

## Recommended Tuning Workflow

1. Set `MC_USE_SMC = 1` and small initial gains (0.5–0.8)
2. Start SITL and slowly increase L for each axis while monitoring angular rates
3. Introduce k gradually to add robustness against disturbances
4. Adjust torque limits to allow sufficient control authority without saturation
5. Iterate until smooth hover and stable maneuvers are achieved
