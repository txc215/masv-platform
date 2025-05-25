# Sensor Fusion Design – Extended Kalman filter for GNSS + IMU + Magnetometer

## 1. Overview

This document outlines the design assumptions and initial structure for an Extended Kalman Filter (EKF) used to fuse single-antenna GNSS, IMU (accelerometer + gyroscope) + magnetometer, assuming all sensors and antenna are co-located and time-synchronized.

---

## 2. Assumptions

| Category         | Assumption |
|------------------|------------|
| **GNSS**         | Single-antenna GNSS providing 2D global position (x, y) and optional heading via velocity vector |
| **IMU**          | Contains gyroscope, accelerometer, and magnetometer |
| **Magnetometer** | Used for heading estimation, assuming horizontal magnetic field with tilt compensation |
| **Sensor Layout**| All sensors and GNSS antenna are located at the same physical point (no lever arm offset) |
| **Motion Model** | 2D bicycle model or constant velocity model |
| **Coordinate Frame** | NED (North-East-Down) or ENU depending on GNSS output |

---

## 3. EKF State Vector

```text
x = [ pos_x, pos_y, vel_x, vel_y, heading, gyro_bias ]
