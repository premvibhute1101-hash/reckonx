# IDR-PoC: Mathematics & Algorithms Study Guide

This document explains the core mathematics, sensor fusion algorithms, and architectural decisions used in the IDR-PoC (Inertial Dead Reckoning) project. It serves as a study guide for understanding how the system maintains highly accurate navigation during GNSS blackouts.

---

## 1. The Error-State Extended Kalman Filter (ES-EKF)

Unlike standard Kalman Filters that try to track the total position and velocity directly, this project uses an **Error-State EKF**. 

### Why Error-State?
Tracking total state directly with an EKF is non-linear and suffers from gimbal lock and rapid divergence. An Error-State EKF tracks the *errors* in the system instead. The high-frequency INS (Inertial Navigation System) integrates the physics to find the total position, while the low-frequency EKF predicts how fast errors are accumulating, and then mathematically subtracts those errors from the INS.

### The 15-State Vector ($\delta x$)
Our system tracks 15 specific error states:
1. **Position Errors** (3 states: $\delta p_x, \delta p_y, \delta p_z$)
2. **Velocity Errors** (3 states: $\delta v_x, \delta v_y, \delta v_z$)
3. **Attitude Errors** (3 states: $\delta \theta_{roll}, \delta \theta_{pitch}, \delta \theta_{yaw}$)
4. **Accelerometer Biases** (3 states: $\delta b_{ax}, \delta b_{ay}, \delta b_{az}$)
5. **Gyroscope Biases** (3 states: $\delta b_{gx}, \delta b_{gy}, \delta b_{gz}$)

---

## 2. INS Mechanization & Physics

At 10Hz, raw IMU data arrives. Before the EKF runs, we do physical Dead Reckoning (`InsMechanization.ts`):

### Attitude Integration
We integrate the gyroscope (rad/s) over time ($dt$) to find Euler angles (Roll, Pitch, Yaw).

### Coordinate Transformation (Direction Cosine Matrix)
Smartphones measure acceleration in the **Body Frame** (the phone's physical axes). We must rotate this to the **Navigation Frame** (East, North, Up) to track movement across the map.

We build the Rotation Matrix $C_b^n$ using the Euler angles:
$$
C_b^n = R_z(yaw) \cdot R_y(pitch) \cdot R_x(roll)
$$
We multiply the raw acceleration by $C_b^n$ to get the specific force in the ENU frame ($f^n$).

### Gravity Subtraction & Double Integration
We subtract Earth's gravity ($9.81 m/s^2$) from the Z-axis of $f^n$. We then integrate acceleration to velocity ($v = v + a \cdot dt$), and velocity to position ($p = p + v \cdot dt$).

---

## 3. The State Transition Matrix ($F$-Matrix)

This is where the EKF predicts how errors grow over time. We use a rigorous $F$-matrix that applies the physics of rotation.

### Velocity Error from Attitude Error
If the phone's pitch is off by 1 degree, it thinks some of gravity is actually forward acceleration. The F-Matrix couples attitude error to velocity error using the **Skew-Symmetric Matrix** of the specific force ($[f^n \times]$):
$$
\delta \dot{v}^n = -[f^n \times] \delta \theta + C_b^n \delta b_a
$$
In code, the matrix $[f^n \times]$ looks like this:
```math
[f^n \times] = \begin{bmatrix} 0 & -f_z & f_y \\ f_z & 0 & -f_x \\ -f_y & f_x & 0 \end{bmatrix}
```
This guarantees that if the user turns a corner during a GPS blackout, the EKF projects error corrections along the curve, rather than just straight ahead.

---

## 4. Process Noise Covariance ($Q$)

The $Q$ matrix represents our uncertainty in the physical sensors. We tune $Q$ to reflect the actual drift physics of the smartphone:
- $Q_{vel} = 1.0 \cdot dt$: We set velocity noise relatively high to model the fact that uncalibrated attitude initialization leaks gravity into horizontal acceleration. 
- $Q_{pos} = 0.1 \cdot dt$: Position noise accounts for systemic unmodeled biases.

By setting $Q$ correctly, the Prediction Covariance Matrix ($P$) grows exactly proportional to how far the user actually physically drifts during a blackout.

---

## 5. GNSS Measurement Updates ($R$ and $K$)

When a GNSS (GPS) fix arrives (1Hz), we calculate the **Innovation** (the difference between the INS predicted position and the GNSS actual position).

### The Kalman Gain ($K$)
$$
K = \frac{P}{P + R}
$$
$K$ dictates who to trust. 
- If $P$ (our internal error) is high and $R$ (GNSS noise) is low, $K \approx 1$. We trust the GNSS fully.
- If $P$ is low and $R$ is high, $K \approx 0$. We trust our INS and ignore the GNSS.

### Dynamic R-Matrix (GNSS State Machine)
The `GnssQualityStateMachine.ts` categorizes the signal into `GOOD`, `DEGRADED`, or `LOST`.
For a `GOOD` state, we use $R = RawGnssAccuracy^2$ (e.g., $3.0^2 = 9.0$). Because we properly tuned $Q$, $P$ will grow to match this, balancing the filter and completely eliminating "sawtooth" divergence patterns.

---

## 6. Mahalanobis Distance (Outlier Gating)

When driving out of a tunnel, the first GPS fix is usually a massive reflection (multipath error), causing a "teleport" effect. We filter this out using the **Mahalanobis Distance**:
$$
D^2 = z^T (H P H^T + R)^{-1} z
$$
This measures how many "standard deviations" the GNSS reading is from our predicted position. If $D^2 > 12.6$ (the Chi-Square threshold for 6 degrees of freedom), we mathematically reject the GNSS reading as an impossible hallucination. 

---

## 7. Initial Stationary Alignment

If you start dead reckoning while moving, forward acceleration is permanently logged as gravitational tilt, causing runaway velocity (e.g., drifting 3000m in a minute).

We solve this using **Stationary Variance Analysis**:
1. We buffer 20 samples (2 seconds) of IMU data.
2. We calculate the statistical variance ($\sigma^2$) of the accelerometer and gyro.
3. If $\sigma_{accel}^2 < 0.5$ and $\sigma_{gyro}^2 < 0.05$, the user is holding the phone still.
4. We average the 20 samples to filter out noise, extract the perfect gravity vector, and initialize pitch and roll perfectly before the EKF starts running.

---

## 8. AI Motion Model (CNN-LSTM)

During a GNSS `LOST` state, standard dead reckoning drifts exponentially due to the double-integration of accelerometer noise.

We invoke a TensorFlow Lite AI Model (`ins_error_model_best.tflite`). 
- **Input:** 200 time-steps of raw IMU data.
- **Architecture:** Convolutional Neural Networks (CNN) extract spatial movement patterns (like a walking gait or car engine vibration), while Long Short-Term Memory (LSTM) layers track how those patterns evolve over time.
- **Output:** The model predicts the pseudo-velocity (the actual speed of the user). We feed this pseudo-velocity back into the EKF as a measurement update to aggressively drag the INS velocity back to reality, bounded by the AI's known variance.
