# PX4 Sliding Mode Control (SMC) – Altitude Hold

## Project Overview

This project replaces the default PX4 altitude PID controller with a **Sliding Mode Control (SMC)** law for robust altitude stabilization of multirotor UAVs.  

**Key Features:**

- Robust against **disturbances** (wind gusts, sensor noise).  
- Handles **payload variations** and modeling uncertainties.  
- Implemented entirely in **PX4 SITL** for safe testing.  

---

## Motivation

Standard PX4 altitude controllers use cascaded PID loops, which may perform poorly under:

- Sudden payload changes  
- Strong wind disturbances  
- Sensor noise or modeling errors  

Sliding Mode Control ensures the UAV maintains its target altitude with **robust and reliable performance**, even in uncertain conditions.

---

## Features

- Altitude stabilization using SMC.  
- Adjustable sliding surface parameters.  
- Optional tuning of **chattering reduction** via super-twisting.  
- Compatible with PX4 multicopters in **SITL (Gazebo or jmavsim)**.  
- Logging and plotting of altitude response and control inputs.

---
