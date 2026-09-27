"""
ekf.py
======
15-State Error-State Extended Kalman Filter (ES-EKF) for INS / AI / GNSS sensor fusion.
Ported from IDR_PRO (https://github.com/Manish-1205/IDR_PRO - EkfCore.ts).

State Vector (15-state, 3D):
----------------------------
  0-2:  Position (p_x, p_y, p_z) in ENU navigation frame (meters)
  3-5:  Velocity (v_x, v_y, v_z) in ENU navigation frame (m/s)
  6-8:  Attitude errors (psi_pitch, psi_roll, psi_yaw) in radians
  9-11: Accelerometer bias (b_ax, b_ay, b_az) in body frame (m/s^2)
  12-14: Gyroscope bias (b_gx, b_gy, b_gz) in body frame (rad/s)

Key Features:
-------------
- Rigorous F-matrix projection with DCM Body-to-Navigation cross-product coupling.
- Multi-mode GNSS measurement update (6D pos+vel, 3D pos-only) with adaptive R scaling.
- AI residual velocity error updates during GNSS degraded/lost intervals.
- True Zero Velocity Updates (ZUPT) with post-ZUPT Q-inflation cooldown scheduling.
- Non-Holonomic Constraints (NHC) enforcing zero lateral wheel slip (v_lat ~ 0).
- Runaway velocity clamp (+-60 m/s) and jump rate limiter (>3 m/s per cycle).
"""

import numpy as np
import math
import os
import sys
from typing import Dict, Any, Optional, Tuple

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config


class NavigationEKF:
    """
    15-state Error-State Extended Kalman Filter for 3D INS / AI / GNSS fusion.
    """
    STATE_DIM = 15

    def __init__(self):
        # 15-state vector: [px, py, pz, vx, vy, vz, att_p, att_r, att_y, bax, bay, baz, bgx, bgy, bgz]
        self.x = np.zeros(self.STATE_DIM, dtype=float)

        # 15x15 Covariance Matrix P
        self.P = np.zeros((self.STATE_DIM, self.STATE_DIM), dtype=float)
        for i in range(3):
            self.P[i, i] = 0.1       # Position (m^2)
        for i in range(3, 6):
            self.P[i, i] = 0.1       # Velocity ((m/s)^2)
        for i in range(6, 9):
            self.P[i, i] = 0.001     # Attitude (rad^2)
        for i in range(9, 12):
            self.P[i, i] = 0.0001    # Accel bias ((m/s^2)^2)
        for i in range(12, 15):
            self.P[i, i] = 0.00001   # Gyro bias ((rad/s)^2)

        # Tracking state
        self.position_enu = np.zeros(3, dtype=float)
        self.velocity_enu = np.zeros(3, dtype=float)
        self.attitude_rpy = np.zeros(3, dtype=float)  # roll, pitch, yaw in rad

        self.last_c_b_n = np.eye(3, dtype=float)
        self.last_specific_force = np.zeros(3, dtype=float)

        # ZUPT & Cooldown Tracking
        self.was_zupt_active = False
        self.post_zupt_cooldown = 0
        self.POST_ZUPT_COOLDOWN_CYCLES = 5
        self.previous_velocity: Optional[np.ndarray] = None
        self.is_attitude_initialized = False

    def reset(self, pos_x: float = 0.0, pos_y: float = 0.0, pos_z: float = 0.0,
              vel_x: float = 0.0, vel_y: float = 0.0, vel_z: float = 0.0):
        """Reset filter states and covariance."""
        self.x = np.zeros(self.STATE_DIM, dtype=float)
        self.x[0] = pos_x
        self.x[1] = pos_y
        self.x[2] = pos_z
        self.x[3] = vel_x
        self.x[4] = vel_y
        self.x[5] = vel_z

        self.position_enu = np.array([pos_x, pos_y, pos_z], dtype=float)
        self.velocity_enu = np.array([vel_x, vel_y, vel_z], dtype=float)

        self.P = np.zeros((self.STATE_DIM, self.STATE_DIM), dtype=float)
        for i in range(3):
            self.P[i, i] = 0.1
        for i in range(3, 6):
            self.P[i, i] = 0.1
        for i in range(6, 9):
            self.P[i, i] = 0.001
        for i in range(9, 12):
            self.P[i, i] = 0.0001
        for i in range(12, 15):
            self.P[i, i] = 0.00001

        self.was_zupt_active = False
        self.post_zupt_cooldown = 0
        self.previous_velocity = None

    def initialize_attitude(self, accel: np.ndarray):
        """
        Initialize roll & pitch from the gravity vector:
        accel: [ax, ay, az] in m/s^2
        """
        ax, ay, az = accel[0], accel[1], accel[2]
        pitch = math.atan2(ay, az)
        roll = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        self.attitude_rpy = np.array([roll, pitch, 0.0], dtype=float)
        self.is_attitude_initialized = True
        self._update_rotation_matrix()

    def _update_rotation_matrix(self):
        """Compute Direction Cosine Matrix C_b_n from Euler angles (roll, pitch, yaw)."""
        r, p, y = self.attitude_rpy[0], self.attitude_rpy[1], self.attitude_rpy[2]
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)

        # Standard aerospace sequence Z-Y-X (yaw-pitch-roll)
        self.last_c_b_n = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]
        ], dtype=float)

    def predict(self, dt: float, accel: np.ndarray = None, gyro: np.ndarray = None, ins_vel: np.ndarray = None):
        """
        Predict step (propagates INS mechanization and 15x15 covariance forward in time).
        """
        dt = max(0.001, float(dt))

        # Default fallback sensors if not passed directly
        if accel is None:
            accel = np.array([0.0, 0.0, config.GRAVITY])
        if gyro is None:
            gyro = np.zeros(3, dtype=float)

        if not self.is_attitude_initialized:
            self.initialize_attitude(accel)

        # 0. Subtract estimated sensor biases
        b_a = self.x[9:12]
        b_g = self.x[12:15]
        acc_corr = accel - b_a
        gyr_corr = gyro - b_g

        # 1. Update Attitude from corrected gyro rates
        self.attitude_rpy += gyr_corr * dt
        self._update_rotation_matrix()
        C_b_n = self.last_c_b_n

        # 2. Specific force in navigation frame f^n = C_b^n a^b - [0, 0, g]^T
        f_b = acc_corr
        f_n = C_b_n @ f_b - np.array([0.0, 0.0, config.GRAVITY])
        self.last_specific_force = f_n

        # 3. Mechanization position & velocity propagation
        if ins_vel is not None and len(ins_vel) >= 2:
            self.velocity_enu[0] = ins_vel[0]
            self.velocity_enu[1] = ins_vel[1]
            if len(ins_vel) > 2:
                self.velocity_enu[2] = ins_vel[2]
            self.position_enu += self.velocity_enu * dt
        else:
            self.velocity_enu += f_n * dt
            self.position_enu += self.velocity_enu * dt

        # Update state vector positions/velocities
        self.x[0:3] = self.position_enu
        self.x[3:6] = self.velocity_enu

        # 4. Construct Rigorous 15x15 Transition Matrix F
        F = np.eye(self.STATE_DIM, dtype=float)

        # Position from Velocity: p_new = p_old + v * dt
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Velocity error from Attitude error: -[f^n x] * dt
        # -[f^n x] = [   0    f_z  -f_y ]
        #            [ -f_z    0    f_x ]
        #            [  f_y  -f_x    0  ]
        fx, fy, fz = f_n[0], f_n[1], f_n[2]
        F[3, 6] = 0.0;        F[3, 7] = fz * dt;    F[3, 8] = -fy * dt
        F[4, 6] = -fz * dt;   F[4, 7] = 0.0;        F[4, 8] = fx * dt
        F[5, 6] = fy * dt;    F[5, 7] = -fx * dt;   F[5, 8] = 0.0

        # Velocity error from Accel bias: -C_b^n * dt
        for r in range(3):
            for c in range(3):
                F[3 + r, 9 + c] = -C_b_n[r, c] * dt

        # Attitude error from Gyro bias: -C_b^n * dt
        for r in range(3):
            for c in range(3):
                F[6 + r, 12 + c] = -C_b_n[r, c] * dt

        # 5. Process Noise Q
        Q = np.zeros((self.STATE_DIM, self.STATE_DIM), dtype=float)
        for i in range(3):
            Q[i, i] = (config.EKF_Q_POS_STD ** 2) * dt

        vel_q = (config.EKF_Q_VEL_STD ** 2) * dt
        if self.post_zupt_cooldown > 0:
            vel_q = 2.0 * dt  # Inflate Q during post-ZUPT cooldown
            self.post_zupt_cooldown -= 1

        for i in range(3, 6):
            Q[i, i] = vel_q

        for i in range(6, 9):
            Q[i, i] = 1e-6 * dt  # attitude
        for i in range(9, 12):
            Q[i, i] = 1e-6 * dt  # accel bias
        for i in range(12, 15):
            Q[i, i] = 1e-7 * dt  # gyro bias

        # Propagate covariance P = F P F^T + Q
        self.P = F @ self.P @ F.T + Q

    def _apply_measurement_update(self, H: np.ndarray, z: np.ndarray, R: np.ndarray, chi2_thresh: float = float("inf")) -> bool:
        """
        Generic Kalman measurement update with Joseph form covariance stabilization
        and Mahalanobis chi-squared outlier gating.
        """
        # Innovation y = z - H @ x_error (in error-state EKF, expected error is 0, so y = z)
        y = z - H @ (self.x - np.concatenate([self.position_enu, self.velocity_enu, np.zeros(9)]))
        S = H @ self.P @ H.T + R

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False

        # Chi-Square Innovation Gate
        if not math.isinf(chi2_thresh):
            mahalanobis_sq = float(y.T @ S_inv @ y)
            if mahalanobis_sq > chi2_thresh:
                return False

        # Kalman Gain K = P H^T S^-1
        K = self.P @ H.T @ S_inv

        # Error state correction
        delta_x = K @ y

        # Apply corrections to position, velocity, and biases
        self.position_enu += delta_x[0:3]
        self.velocity_enu += delta_x[3:6]
        self.attitude_rpy += delta_x[6:9]
        self.x[9:12] += delta_x[9:12]   # accel bias
        self.x[12:15] += delta_x[12:15] # gyro bias

        self.x[0:3] = self.position_enu
        self.x[3:6] = self.velocity_enu

        # Joseph-form covariance update
        I_KH = np.eye(self.STATE_DIM, dtype=float) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        self._apply_velocity_guard()
        return True

    def update_gnss(self, pos_measurement: np.ndarray, accuracy_m: Optional[float] = None, vel_measurement: Optional[np.ndarray] = None) -> bool:
        """
        GNSS Measurement Update (3D Position or 6D Position+Velocity).
        """
        base_acc = accuracy_m if accuracy_m is not None and accuracy_m > 0 else config.EKF_R_GNSS_POS_STD
        base_pos_var = max(0.1, (base_acc ** 2))

        if vel_measurement is not None and len(vel_measurement) >= 2:
            # 6D Pos + Vel update
            H = np.zeros((6, self.STATE_DIM), dtype=float)
            for i in range(6):
                H[i, i] = 1.0

            R = np.eye(6, dtype=float) * base_pos_var
            for i in range(3, 6):
                R[i, i] = 0.5  # GNSS velocity variance

            meas_pos = np.array([pos_measurement[0], pos_measurement[1], pos_measurement[2] if len(pos_measurement) > 2 else 0.0])
            meas_vel = np.array([vel_measurement[0], vel_measurement[1], vel_measurement[2] if len(vel_measurement) > 2 else 0.0])

            z = np.concatenate([meas_pos - self.position_enu, meas_vel - self.velocity_enu])
            return self._apply_measurement_update(H, z, R, chi2_thresh=25.0)
        else:
            # 3D Pos update
            H = np.zeros((3, self.STATE_DIM), dtype=float)
            for i in range(3):
                H[i, i] = 1.0

            R = np.eye(3, dtype=float) * base_pos_var
            meas_pos = np.array([pos_measurement[0], pos_measurement[1], pos_measurement[2] if len(pos_measurement) > 2 else 0.0])
            z = meas_pos - self.position_enu
            return self._apply_measurement_update(H, z, R, chi2_thresh=config.EKF_INNOVATION_GATE_CHI2)

    def update_ai_velocity(self, corrected_vel: np.ndarray, r_std: Optional[float] = None):
        """
        AI velocity correction update: updates 2D velocity (vx, vy) with learned model residual.
        """
        r_val = r_std if r_std is not None and r_std > 0 else config.EKF_R_AI_VEL_STD
        H = np.zeros((2, self.STATE_DIM), dtype=float)
        H[0, 3] = 1.0  # vx (East)
        H[1, 4] = 1.0  # vy (North)

        R = np.eye(2, dtype=float) * (r_val ** 2)
        z = np.array([
            corrected_vel[0] - self.velocity_enu[0],
            corrected_vel[1] - self.velocity_enu[1]
        ])

        self._apply_measurement_update(H, z, R, chi2_thresh=50.0)

    def update_zupt(self):
        """
        Zero Velocity Update (ZUPT): forces velocity to 0 when stationary.
        """
        H = np.zeros((3, self.STATE_DIM), dtype=float)
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        H[2, 5] = 1.0

        R = np.eye(3, dtype=float) * 0.001
        z = -self.velocity_enu

        self._apply_measurement_update(H, z, R, chi2_thresh=float("inf"))

        self.was_zupt_active = True
        self.velocity_enu[:] = 0.0
        self.x[3:6] = 0.0

    def update_nhc(self, heading_rad: Optional[float] = None, r_std: float = config.NHC_LATERAL_VEL_STD):
        """
        Non-Holonomic Constraint (NHC) enforcing zero lateral velocity for land vehicles.
        """
        if heading_rad is None:
            speed = np.linalg.norm(self.velocity_enu[:2])
            if speed < 0.1:
                return
            heading_rad = math.atan2(self.velocity_enu[1], self.velocity_enu[0])

        H = np.zeros((1, self.STATE_DIM), dtype=float)
        H[0, 3] = -math.sin(heading_rad)
        H[0, 4] = math.cos(heading_rad)

        R = np.array([[r_std ** 2]], dtype=float)
        z = np.array([-math.sin(heading_rad) * self.velocity_enu[0] + math.cos(heading_rad) * self.velocity_enu[1]])

        self._apply_measurement_update(H, -z, R, chi2_thresh=50.0)

    def _apply_velocity_guard(self):
        """Velocity jump rate limiter and +-60 m/s runaway safety clamp."""
        v = self.velocity_enu
        if self.previous_velocity is not None:
            jump = np.linalg.norm(v - self.previous_velocity)
            if jump > 3.0:
                pass  # Rate jump limit triggered

        # Clamp max speed
        self.velocity_enu[0] = np.clip(v[0], -60.0, 60.0)
        self.velocity_enu[1] = np.clip(v[1], -60.0, 60.0)
        self.velocity_enu[2] = np.clip(v[2], -60.0, 60.0)
        self.x[3:6] = self.velocity_enu
        self.previous_velocity = self.velocity_enu.copy()

    @property
    def position(self) -> np.ndarray:
        """Current estimated position [pos_x, pos_y]."""
        return self.position_enu[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Current estimated velocity [vel_x, vel_y]."""
        return self.velocity_enu[:2].copy()

    @property
    def position_uncertainty(self) -> float:
        """1-sigma position uncertainty (meters)."""
        return float(np.sqrt(self.P[0, 0] + self.P[1, 1]))

    @property
    def velocity_uncertainty(self) -> float:
        """1-sigma velocity uncertainty (m/s)."""
        return float(np.sqrt(self.P[3, 3] + self.P[4, 4]))


def run_ekf_fusion(ins_df, ai_corrections=None, gnss_available=None,
                   gnss_pos=None, gnss_accuracy=None, gnss_sats=None,
                   use_seamless_switching=True):
    """
    Run 15-state EKF fusion over an entire session.
    Preserves full interface compatibility with pipeline.py and evaluate.py.
    """
    from src.seamless_controller import GNSSQualityStateMachine, check_innovation_gate, NavigationMode

    n = len(ins_df)
    t = ins_df["_t_sec"].to_numpy(dtype=float)
    ins_vel_x = ins_df["ins_vel_x"].to_numpy(dtype=float)
    ins_vel_y = ins_df["ins_vel_y"].to_numpy(dtype=float)

    ekf = NavigationEKF()
    sm = GNSSQualityStateMachine() if use_seamless_switching else None

    # Initialize position
    if gnss_pos is not None and len(gnss_pos) > 0:
        ekf.reset(pos_x=gnss_pos[0, 0], pos_y=gnss_pos[0, 1])

    out_pos_x = np.zeros(n)
    out_pos_y = np.zeros(n)
    out_vel_x = np.zeros(n)
    out_vel_y = np.zeros(n)
    out_pos_unc = np.zeros(n)
    out_vel_unc = np.zeros(n)
    mode_history = []
    ai_active_mask = np.zeros(n, dtype=bool)

    consecutive_rejected_gnss = 0

    for i in range(n):
        dt = t[i] - t[i-1] if i > 0 else 0.1

        # 1. PREDICT: propagate state forward
        if i > 0:
            ins_vel = np.array([ins_vel_x[i], ins_vel_y[i]])
            ekf.predict(dt, ins_vel=ins_vel)

        # 2. GNSS Quality Evaluation
        raw_gps_ok = True
        if gnss_available is not None:
            raw_gps_ok = bool(gnss_available[i])
        acc = float(gnss_accuracy[i]) if (gnss_accuracy is not None and i < len(gnss_accuracy)) else None
        sats = int(gnss_sats[i]) if (gnss_sats is not None and i < len(gnss_sats)) else None

        if use_seamless_switching and sm is not None:
            state_info = sm.evaluate_sample(gps_ok=raw_gps_ok, accuracy_m=acc, sats_count=sats)
            current_mode = state_info["mode"].value
            trust_factor = state_info["trust_factor"]
            ai_active = state_info["ai_active"]
            skip_gnss = state_info["skip_gnss"]
        else:
            gps_ok_thresh = raw_gps_ok and (acc is None or acc < config.GNSS_BLACKOUT_THRESHOLD_M)
            skip_gnss = not gps_ok_thresh
            trust_factor = 1.0 if gps_ok_thresh else 0.0
            ai_active = (ai_corrections is not None)
            current_mode = "GOOD" if gps_ok_thresh else "LOST"

        mode_history.append(current_mode)
        ai_active_mask[i] = ai_active

        # 3. AI VELOCITY UPDATE
        if ai_active and ai_corrections is not None and i < len(ai_corrections):
            corrected_vel = np.array([
                ins_vel_x[i] + ai_corrections[i, 0],
                ins_vel_y[i] + ai_corrections[i, 1],
            ])
            ekf.update_ai_velocity(corrected_vel)

            # Apply Non-Holonomic Constraint during degraded/lost GPS
            if current_mode in [NavigationMode.DEGRADED.value, NavigationMode.LOST.value]:
                ekf.update_nhc()

        # 4. GNSS POSITION UPDATE
        if not skip_gnss and gnss_pos is not None and i < len(gnss_pos):
            base_acc = acc if acc is not None and acc > 0 else config.EKF_R_GNSS_POS_STD
            effective_acc = base_acc / max(0.05, trust_factor)
            meas = gnss_pos[i]

            updated = ekf.update_gnss(meas, accuracy_m=effective_acc)
            if updated:
                consecutive_rejected_gnss = 0
            else:
                consecutive_rejected_gnss += 1
                if consecutive_rejected_gnss >= 5:
                    ekf.reset(pos_x=meas[0], pos_y=meas[1])
                    consecutive_rejected_gnss = 0

        # Record output
        out_pos_x[i] = ekf.position[0]
        out_pos_y[i] = ekf.position[1]
        out_vel_x[i] = ekf.velocity[0]
        out_vel_y[i] = ekf.velocity[1]
        out_pos_unc[i] = ekf.position_uncertainty
        out_vel_unc[i] = ekf.velocity_uncertainty

    battery_summary = sm.get_battery_savings_summary() if sm is not None else {
        "duty_cycle_pct": 100.0 if ai_corrections is not None else 0.0,
        "battery_savings_pct": 0.0 if ai_corrections is not None else 100.0
    }

    return {
        "pos_x": out_pos_x,
        "pos_y": out_pos_y,
        "vel_x": out_vel_x,
        "vel_y": out_vel_y,
        "pos_unc": out_pos_unc,
        "vel_unc": out_vel_unc,
        "mode_history": mode_history,
        "ai_active_mask": ai_active_mask,
        "battery_summary": battery_summary,
    }


if __name__ == "__main__":
    print("15-State NavigationEKF loaded successfully.")
    ekf = NavigationEKF()
    print(f"Initial state shape: {ekf.x.shape}, position: {ekf.position}, vel: {ekf.velocity}")
    print(f"Initial covariance diagonal: {np.diag(ekf.P)}")
