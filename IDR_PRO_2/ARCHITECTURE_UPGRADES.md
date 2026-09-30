# Architecture Upgrades: IDR-PoC vs KF-GINS

This document outlines the areas where the current Inertial Dead Reckoning Proof of Concept (`idr-poc`) can be upgraded to match the mathematical rigor and precision of an industrial-grade Error-State Extended Kalman Filter, taking inspiration from systems like [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS).

While `idr-poc` shares the same core philosophy (closed-loop Error-State EKF feeding back estimated biases and navigation errors), it currently employs mathematical shortcuts suitable for a PoC. The following improvements are required for a production-grade system:

## 1. Expanding the State Vector (Sensor Scale-Factor Errors)
**Current:** A 15-state filter (3 Position, 3 Velocity, 3 Attitude, 3 Accel Bias, 3 Gyro Bias) assuming perfectly orthogonal IMU axes and a strict 1.0 scale factor.
**Upgrade:** Expand to a **21-state EKF** by adding 6 states to estimate Accelerometer and Gyroscope scale factors and cross-axis misalignments. Cheap smartphone IMUs suffer heavily from scale errors, which cause rapid drift during high-acceleration maneuvers.

## 2. Rigorous State Transition Matrix ($F$-Matrix)
**Current:** The error-state propagation uses a highly simplified $F$-matrix. For instance, the mapping of attitude error to velocity error uses hardcoded gravitational constants (`g * dt`) and bypasses coordinate transformations.
**Upgrade:** Implement the full continuous-time inertial navigation error equations. Utilize the Direction Cosine Matrix (DCM) or Quaternions to project body-frame specific forces (accelerometer readings) into the navigation frame (ENU). This includes calculating the proper skew-symmetric matrices for rigorous error state transitions.

## 3. Earth Rotation and Coriolis Effects
**Current:** Navigation is modeled on a flat, non-rotating local ENU frame.
**Upgrade:** Account for the Earth's rotation rate ($\omega_{ie}$) and the transport rate (the rotation of the ENU frame as you travel over the Earth's curved surface, $\omega_{en}$). These rotation rates must be subtracted from the raw gyroscope readings during the INS prediction step to prevent long-term heading and position drift.

## 4. Lever Arm Compensation
**Current:** The GNSS antenna and IMU sensor are assumed to occupy the exact same point in space.
**Upgrade:** Introduce a "Lever Arm Compensation" algorithm in the Measurement Matrix ($H$-matrix). When a vehicle rotates, the GNSS antenna moves even if the IMU does not translate. The algorithm mathematically translates the GNSS measurement precisely to the IMU's center of mass using the known physical offset vector and current attitude.

## 5. IMU Noise Modeling via Allan Variance
**Current:** The $Q$-matrix process noise parameters (e.g., `0.1 * dt`) are manually tuned scalars padded to cover unmodeled systemic errors.
**Upgrade:** Derive $Q$ parameters directly from **Allan Variance** analysis (Angle Random Walk, Velocity Random Walk, Bias Instability). Profiling the specific smartphone's IMU using Allan Variance tools allows for the calculation of exact, physically-proven process noise covariances.

## 6. Initial Alignment Phase
**Current:** The EKF initiates dead-reckoning immediately with a rough or zeroed attitude.
**Upgrade:** Implement a rigorous "Coarse Alignment" followed by a "Fine Alignment" phase before navigation begins. This uses static gravity to lock pitch/roll, and gyro-compassing (or moving GNSS vectors) to converge yaw, ensuring the covariance matrix ($P$) is highly constrained before integrating motion.
