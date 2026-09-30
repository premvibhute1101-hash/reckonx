import { Matrix, inverse as mlInverse } from 'ml-matrix';
import { InsMechanization } from './InsMechanization';
import { GnssQualityStateMachine } from './GnssQualityStateMachine';
import { AiMotionModel } from './AiMotionModel';

export class EkfCore {
  // 15-state vector:
  // 0-2: Position (x,y,z)
  // 3-5: Velocity (x,y,z)
  // 6-8: Attitude errors (pitch, roll, yaw)
  // 9-11: Accel bias (x,y,z)
  // 12-14: Gyro bias (x,y,z)
  private x: Matrix;
  private P: Matrix;

  private ins: InsMechanization;
  private gnssState: GnssQualityStateMachine;
  private aiModel: AiMotionModel;

  // ── IDR_PRO_2 feature flags ────────────────────────────────────────────────
  /** Enable phone-to-vehicle yaw offset estimation from GNSS course vs INS heading. */
  public enableYawAlignment: boolean = false;
  /** Enable Non-Holonomic Constraints (lateral + vertical velocity = 0) in WEAK_LOST mode. */
  public enableNHC: boolean = false;
  /** Enable bump/pothole gate: inflate Q and suppress AI during high-vibration events. */
  public enableBumpGate: boolean = false;
  /** Running count of detected bump events (diagnostic). */
  public bumpGateCount: number = 0;

  /** Estimated yaw offset between phone forward axis and vehicle forward axis (radians). */
  private yawOffsetRad: number = 0;
  /** True once yawOffsetRad has been initialised from at least one moving GNSS update. */
  private yawOffsetActive: boolean = false;
  // ──────────────────────────────────────────────────────────────────────────

  constructor() {
    this.x = Matrix.zeros(15, 1);
    this.P = Matrix.zeros(15, 15);
    for (let i = 0; i < 3; i++) this.P.set(i, i, 0.1);       // Position
    for (let i = 3; i < 6; i++) this.P.set(i, i, 0.1);       // Velocity
    for (let i = 6; i < 9; i++) this.P.set(i, i, 0.001);     // Attitude
    for (let i = 9; i < 12; i++) this.P.set(i, i, 0.0001);   // Accel bias
    for (let i = 12; i < 15; i++) this.P.set(i, i, 0.00001); // Gyro bias

    this.ins = new InsMechanization();
    this.gnssState = new GnssQualityStateMachine();
    this.aiModel = new AiMotionModel();
  }

  // ZUPT state tracking for Q scheduling
  private wasZuptActive = false;
  private postZuptCooldown = 0;
  private readonly POST_ZUPT_COOLDOWN_CYCLES = 40; // ~2.0s at 20Hz / 4.0s at 10Hz to safely cover 1Hz GNSS updates
  private previousVelocity: { x: number; y: number; z: number } | null = null;
  private lastRawAccel: number[] = [0, 0, 9.81];
  private lastVelQ: number = 0.01;
  private lastAiCorrection: number[] | null = null;
  // Set to true once FusionRuntime completes stationary attitude initialisation.
  // NHC must NOT fire before this: an identity C_b_n projects full gravity into
  // navigation-frame lateral/vertical velocity, instantly saturating the ±60 guard.
  private isAttitudeInitialized = false;
  private isPositionInitialized = false;

  public getIns() {
    return this.ins;
  }

  public getGnssState() {
    return this.gnssState;
  }

  public getAiModel() {
    return this.aiModel;
  }

  private currentTime: number = 0;

  /**
   * Predict step (runs constantly at IMU rate)
   */
  public predict(dt: number, accel: number[], gyro: number[]) {
    this.currentTime += dt;
    this.lastRawAccel = [...accel];

    // Guard: if initializeAttitude() has not been called yet, auto-initialize using a
    // neutral level orientation [0, 0, g]. Using the raw accel sample would create a
    // slightly non-zero attitude from any off-axis transient (e.g. the startup
    // acceleration bump in GnssVelocityFallback), which then leaks gravity back into
    // horizontal velocity for all subsequent level samples.
    // FusionRuntime overwrites this with a proper stationary-window init before predict
    // is called for real, so this neutral fallback only matters in tests that skip
    // initializeAttitude() entirely.
    if (this.ins.getFilteredAccel() === null) {
      this.ins.initializeAttitude({ x: 0, y: 0, z: 9.81 });
      this.isAttitudeInitialized = true;
    } else if (!this.isAttitudeInitialized) {
      // initializeAttitude() was called externally (e.g., by a test); sync the flag.
      this.isAttitudeInitialized = true;
    }

    // 0. Subtract estimated biases from raw IMU measurements
    const accX = accel[0] - this.x.get(9, 0);
    const accY = accel[1] - this.x.get(10, 0);
    const accZ = accel[2] - this.x.get(11, 0);

    const gyrX = gyro[0] - this.x.get(12, 0);
    const gyrY = gyro[1] - this.x.get(13, 0);
    const gyrZ = gyro[2] - this.x.get(14, 0);

    // 1. Advance INS Mechanization with bias-corrected inputs
    this.ins.predict(
      dt, 
      { x: accX, y: accY, z: accZ }, 
      { x: gyrX, y: gyrY, z: gyrZ }
    );

    // 2. Propagate Covariance P = F * P * F^T + Q
    const F = Matrix.eye(15);
    
    // Position depends on velocity: p_new = p_old + v * dt
    F.set(0, 3, dt);
    F.set(1, 4, dt);
    F.set(2, 5, dt);

    // Rigorous F-matrix construction (Body to Navigation frame projection)
    const C_b_n = this.ins.lastRotationMatrix;
    const f_n = this.ins.lastSpecificForce;

    // Velocity error from Attitude error: -[f^n x] * dt
    F.set(3, 6, 0);                 F.set(3, 7, f_n.z * dt);       F.set(3, 8, -f_n.y * dt);
    F.set(4, 6, -f_n.z * dt);       F.set(4, 7, 0);                F.set(4, 8, f_n.x * dt);
    F.set(5, 6, f_n.y * dt);        F.set(5, 7, -f_n.x * dt);      F.set(5, 8, 0);

    // Velocity error from Accelerometer bias: -C_b^n * dt (since f_true = f_meas - b_a)
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        F.set(3 + r, 9 + c, -C_b_n[r][c] * dt);
      }
    }

    // Attitude error from Gyroscope bias: -C_b^n * dt
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        F.set(6 + r, 12 + c, -C_b_n[r][c] * dt);
      }
    }

    // Process noise Q (tunable — calibrate against actual IMU noise characteristics)
    const Q = Matrix.zeros(15, 15);
    for (let i = 0; i < 3; i++) Q.set(i, i, 0.1 * dt);         // position

    // Bump / Pothole Gate: detect high-acceleration or high-rotation events
    // and inflate velocity + attitude Q to reduce trust in IMU during vibration.
    let bumpDetected = false;
    if (this.enableBumpGate) {
      const accelMag = Math.sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
      const gyroMag  = Math.sqrt(gyro[0]*gyro[0]  + gyro[1]*gyro[1]  + gyro[2]*gyro[2]);
      if (Math.abs(accelMag - 9.81) > 3.0 || gyroMag > 1.0) {
        bumpDetected = true;
        this.bumpGateCount++;
      }
    }

    // Velocity Q: inflate during post-ZUPT cooldown to allow graceful transition.
    // Also inflate massively if a bump/pothole is detected.
    let velQ = 0.25 * dt;
    if (this.postZuptCooldown > 0) {
      velQ = 2.0 * dt;
      this.postZuptCooldown--;
    }
    if (bumpDetected) {
      velQ = Math.max(velQ, 5.0 * dt);
    }
    this.lastVelQ = velQ;
    for (let i = 3; i < 6; i++) Q.set(i, i, velQ);

    for (let i = 6; i < 9; i++) Q.set(i, i, bumpDetected ? 0.001 * dt : 0.000001 * dt);  // attitude
    for (let i = 9; i < 12; i++) Q.set(i, i, 0.00001 * dt);    // accel bias
    for (let i = 12; i < 15; i++) Q.set(i, i, 0.000001 * dt);  // gyro bias

    this.P = F.mmul(this.P).mmul(F.transpose()).add(Q);

    this.applyVelocityGuard();
  }

  /**
   * Applies a generalized Kalman measurement update in Joseph form with Mahalanobis gating.
   * Feeds back errors to INS mechanization (closed-loop EKF) and resets error states 0-8.
   */
  public applyMeasurementUpdate(H: Matrix, z: Matrix, R: Matrix, chiSquareThreshold: number = 6.0): boolean {
    const S = H.mmul(this.P).mmul(H.transpose()).add(R);
    const S_inv = inverse(S);
    if (!S_inv) return false;

    const mahalanobisSq = z.transpose().mmul(S_inv).mmul(z).get(0, 0);
    if (mahalanobisSq >= chiSquareThreshold) {
      return false; // Outlier rejected
    }

    const K = this.P.mmul(H.transpose()).mmul(S_inv);
    const dx = K.mmul(z);
    this.x = this.x.add(dx);

    // Joseph-form covariance update
    const I = Matrix.eye(15);
    const IKH = I.sub(K.mmul(H));
    this.P = IKH.mmul(this.P).mmul(IKH.transpose()).add(K.mmul(R).mmul(K.transpose()));

    // Closed-loop state feedback to INS
    this.ins.position.x += this.x.get(0, 0);
    this.ins.position.y += this.x.get(1, 0);
    this.ins.position.z += this.x.get(2, 0);
    this.ins.velocity.x += this.x.get(3, 0);
    this.ins.velocity.y += this.x.get(4, 0);
    this.ins.velocity.z += this.x.get(5, 0);

    // If device is confirmed stationary (ZUPT active), prevent position-error covariance from leaking non-zero velocity
    if (this.wasZuptActive) {
      this.ins.velocity.x = 0;
      this.ins.velocity.y = 0;
      this.ins.velocity.z = 0;
      this.x.set(3, 0, 0);
      this.x.set(4, 0, 0);
      this.x.set(5, 0, 0);
    }

    // Project Nav-frame attitude error delta_theta^n to Body frame delta_theta^b = (C_b^n)^T * delta_theta^n
    const dThetaE = this.x.get(6, 0);
    const dThetaN = this.x.get(7, 0);
    const dThetaU = this.x.get(8, 0);
    const C_b_n = this.ins.lastRotationMatrix;

    const dRollRaw = C_b_n[0][0] * dThetaE + C_b_n[1][0] * dThetaN + C_b_n[2][0] * dThetaU;
    const dPitchRaw = C_b_n[0][1] * dThetaE + C_b_n[1][1] * dThetaN + C_b_n[2][1] * dThetaU;
    const dYawRaw = C_b_n[0][2] * dThetaE + C_b_n[1][2] * dThetaN + C_b_n[2][2] * dThetaU;

    // Clamp attitude error correction to max ~3 degrees (0.05 rad) per update to prevent tilt runaway
    const maxAttJump = 0.05;
    const dRoll = Math.max(-maxAttJump, Math.min(maxAttJump, dRollRaw));
    const dPitch = Math.max(-maxAttJump, Math.min(maxAttJump, dPitchRaw));
    const dYaw = Math.max(-maxAttJump, Math.min(maxAttJump, dYawRaw));

    this.ins.attitude.roll += dRoll;
    this.ins.attitude.pitch += dPitch;
    this.ins.attitude.yaw += dYaw;

    // Reset error state (since we fed it back)
    for (let i = 0; i < 9; i++) {
      this.x.set(i, 0, 0);
    }
    return true;
  }

  /**
   * Non-Holonomic Constraints (NHC)
   * Constrains lateral (y) and vertical (z) body-frame velocity to ~0 for land vehicles.
   * Only applied during WEAK_LOST / INS-only mode.
   */
  public applyNhc(R_lat: number = 0.05, R_vert: number = 0.01) {
    const C_b_n = this.ins.lastRotationMatrix;
    const vel = this.ins.velocity;

    // Body-frame velocity components: v^b = (C_b^n)^T * v^n
    // Body frame axes: x = lateral (cross-track), y = longitudinal (forward), z = vertical (up)
    const v_lat = C_b_n[0][0] * vel.x + C_b_n[1][0] * vel.y + C_b_n[2][0] * vel.z;
    const v_vert = C_b_n[0][2] * vel.x + C_b_n[1][2] * vel.y + C_b_n[2][2] * vel.z;

    const H = Matrix.zeros(2, 15);
    // Row 0: lateral velocity constraint (body x)
    H.set(0, 3, C_b_n[0][0]);
    H.set(0, 4, C_b_n[1][0]);
    H.set(0, 5, C_b_n[2][0]);

    // Row 1: vertical velocity constraint
    H.set(1, 3, C_b_n[0][2]);
    H.set(1, 4, C_b_n[1][2]);
    H.set(1, 5, C_b_n[2][2]);

    const z = new Matrix([
      [-v_lat],
      [-v_vert]
    ]);

    const R = Matrix.zeros(2, 2);
    R.set(0, 0, R_lat);
    R.set(1, 1, R_vert);

    this.applyMeasurementUpdate(H, z, R, 12.0);
    this.applyVelocityGuard();
  }

  private lastGnssPos: number[] | null = null;
  private lastGnssUpdateTime: number = 0;
  private lastGnssRejected: boolean = false;

  public isLastGnssRejected(): boolean {
    return this.lastGnssRejected;
  }

  /**
   * Update step with GNSS or AI (runs when data is available)
   * Includes IMU-corroboration gate: when ZUPT is active and GNSS reports motion,
   * ZUPT is only released if isImuMotionCorroborated or isUncorroboratedEscape is true.
   */
  public updateGnss(
    gnssPos: number[],
    gnssVel: number[] | null,
    accuracy: number | null,
    recentImuWindow: number[][] = [],
    isImuMotionCorroborated: boolean = true,
    isUncorroboratedEscape: boolean = false,
    hdop?: number | null,
    satCount?: number | null
  ): boolean {
    // 1. Determine GNSS Quality
    const rScale = this.gnssState.updateState(accuracy, hdop, satCount);
    const state = this.gnssState.getState();

    const gnssReportedSpeed = gnssVel !== null
      ? Math.sqrt(gnssVel[0] * gnssVel[0] + gnssVel[1] * gnssVel[1] + (gnssVel[2] || 0) * (gnssVel[2] || 0))
      : 0;

    // Bootstrap Seeding: The very first GNSS fix unconditionally initializes position
    // regardless of accuracy gating, so the filter has a valid origin to track from.
    if (!this.isPositionInitialized) {
      this.ins.position = { x: gnssPos[0], y: gnssPos[1], z: gnssPos[2] };
      if (gnssVel !== null && gnssReportedSpeed >= 0.4) {
        this.ins.velocity = { x: gnssVel[0], y: gnssVel[1], z: gnssVel[2] };
        this.wasZuptActive = false;
      }
      this.lastGnssPos = [...gnssPos];
      this.lastGnssUpdateTime = this.currentTime;
      this.isPositionInitialized = true;
    }

    let z: Matrix; // Measurement residual
    let H: Matrix; // Observation matrix
    let R: Matrix; // Measurement noise covariance

    let effectiveVel = gnssVel;
    const dxInnov = gnssPos[0] - this.ins.position.x;
    const dyInnov = gnssPos[1] - this.ins.position.y;
    const posInnovDist = Math.sqrt(dxInnov * dxInnov + dyInnov * dyInnov);

    const allowMotionRelease = isImuMotionCorroborated || isUncorroboratedEscape;

    if (this.wasZuptActive) {
      if (gnssVel !== null && gnssReportedSpeed >= 0.4 && allowMotionRelease) {
        // Genuine vehicle motion corroborated by IMU or sustained escape rule — release ZUPT
        this.notifyZuptReleased();
      } else {
        // When stationary (ZUPT active) or uncorroborated, velocity is 0 — never let GPS jitter/multipath inject false motion
        effectiveVel = null; // Position-only update

        // IMU-corroboration gate: reject position jumps or uncorroborated high speed while device is still
        if (!allowMotionRelease && (posInnovDist > 6.0 || gnssReportedSpeed >= 0.4)) {
          this.lastGnssRejected = true;
          return false; // Discard uncorroborated multipath jump
        }
      }
    } else if (effectiveVel === null && this.lastGnssPos !== null && (state === 'GOOD' || state === 'DEGRADED')) {
      const dtGnss = this.lastGnssUpdateTime > 0
        ? Math.min(Math.max(this.currentTime - this.lastGnssUpdateTime, 0.1), 3.0)
        : 0;
      if (dtGnss >= 0.1 && dtGnss <= 3.0) {
        const dx = gnssPos[0] - this.lastGnssPos[0];
        const dy = gnssPos[1] - this.lastGnssPos[1];
        const dz = gnssPos[2] - this.lastGnssPos[2];
        const disp = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const vx = dx / dtGnss;
        const vy = dy / dtGnss;
        const vz = dz / dtGnss;
        const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);

        // Noise gate: only derive velocity if displacement exceeds GPS jitter threshold
        const minDispThreshold = state === 'DEGRADED' ? Math.max(3.0, (accuracy || 20) * 0.25) : 0.8;
        if (disp >= minDispThreshold && speed >= 0.8 && speed < 40) {
          effectiveVel = [vx, vy, vz];
        } else {
          effectiveVel = null; // Position-only update
        }
      }
    }
    if (state === 'GOOD' || state === 'DEGRADED') {
      this.lastGnssPos = [...gnssPos];
      this.lastGnssUpdateTime = this.currentTime;

      // Yaw alignment: estimate phone-to-vehicle yaw offset from GNSS course vs INS heading.
      // Only active when enableYawAlignment = true and vehicle is moving (speed > 3 m/s).
      if (this.enableYawAlignment && effectiveVel !== null) {
        const speed = Math.sqrt(effectiveVel[0]*effectiveVel[0] + effectiveVel[1]*effectiveVel[1]);
        if (speed > 3.0) {
          const gnssCourse = Math.atan2(effectiveVel[0], effectiveVel[1]); // atan2(E, N)
          const C = this.ins.lastRotationMatrix;
          const insCourse = Math.atan2(C[0][1], C[1][1]); // phone-Y in nav frame
          let diff = gnssCourse - insCourse;
          while (diff > Math.PI) diff -= 2 * Math.PI;
          while (diff < -Math.PI) diff += 2 * Math.PI;
          if (!this.yawOffsetActive) {
            this.yawOffsetRad = diff;
            this.yawOffsetActive = true;
          } else {
            this.yawOffsetRad = 0.95 * this.yawOffsetRad + 0.05 * diff; // α=0.05 LPF
          }
        }
      }
    }

    if (state === 'GOOD' || state === 'DEGRADED') {
      // Dynamic base variance scaled to reported GNSS accuracy (or standard ~1m if unknown)
      let basePosVar = accuracy !== null && accuracy > 0 ? Math.max(0.1, (accuracy * accuracy) / 9.0) : 1.0;

      // Inflate R if ZUPT is active (suspect stationary period) or under sustained uncorroborated escape mode
      if (isUncorroboratedEscape) {
        basePosVar *= 10.0;
      } else if (this.wasZuptActive && !isImuMotionCorroborated) {
        basePosVar *= 10.0;
      }

      // 1. 3D Position Update
      const H_pos = Matrix.zeros(3, 15);
      for (let i = 0; i < 3; i++) {
        H_pos.set(i, i, 1);
      }

      let R_pos = Matrix.eye(3).mul(basePosVar);
      if (state === 'DEGRADED') {
        R_pos = R_pos.mul(rScale);
      }

      const z_pos = new Matrix([
        [gnssPos[0] - this.ins.position.x],
        [gnssPos[1] - this.ins.position.y],
        [gnssPos[2] - this.ins.position.z],
      ]);
      const isUncorroboratedStationary = this.wasZuptActive && !isImuMotionCorroborated && !isUncorroboratedEscape;
      const posJump = Math.sqrt(z_pos.get(0, 0) * z_pos.get(0, 0) + z_pos.get(1, 0) * z_pos.get(1, 0));

      const currentSpeed = Math.sqrt(this.ins.velocity.x * this.ins.velocity.x + this.ins.velocity.y * this.ins.velocity.y);
      const dtGap = this.lastGnssUpdateTime > 0 ? Math.min(Math.max(this.currentTime - this.lastGnssUpdateTime, 0.1), 15.0) : 1.0;

      let posUpdated = false;
      if (isUncorroboratedStationary && posJump > 2.0) {
        // While uncorroborated stationary: reject position jumps (> 2.0m)
        posUpdated = false;
      } else {
        // Dynamic moving threshold: expand innovation gate gracefully during fast motion and multi-second GPS gaps
        const dynamicMovingThreshold = Math.max(120.0, 45.0 + currentSpeed * dtGap * 4.0);
        const posThreshold = this.postZuptCooldown > 0 ? 150.0 : (this.wasZuptActive ? 6.0 : dynamicMovingThreshold);
        posUpdated = this.applyMeasurementUpdate(H_pos, z_pos, R_pos, posThreshold);
      }

      // 2. 3D Velocity Update (when available)
      let velUpdated = false;
      if (effectiveVel !== null && !isUncorroboratedStationary) {
        const H_vel = Matrix.zeros(3, 15);
        for (let i = 0; i < 3; i++) {
          H_vel.set(i, 3 + i, 1);
        }

        const baseVelVar = isUncorroboratedEscape ? 2.5 : (gnssVel !== null ? 0.01 : 1.0);
        let R_vel = Matrix.eye(3).mul(baseVelVar);
        if (state === 'DEGRADED') {
          R_vel = R_vel.mul(rScale);
        }

        const z_vel = new Matrix([
          [effectiveVel[0] - this.ins.velocity.x],
          [effectiveVel[1] - this.ins.velocity.y],
          [effectiveVel[2] - this.ins.velocity.z],
        ]);
        const velThreshold = this.postZuptCooldown > 0 ? 150.0 : (isUncorroboratedEscape ? 60.0 : 60.0);
        velUpdated = this.applyMeasurementUpdate(H_vel, z_vel, R_vel, velThreshold);
      }

      this.lastGnssRejected = effectiveVel !== null ? (!posUpdated && !velUpdated) : !posUpdated;
    } else {
      // WEAK_LOST State: Use AI pseudo-measurement and NHC
      const aiCorrection = this.aiModel.predictError(
        state, 
        recentImuWindow, 
        [this.ins.velocity.x, this.ins.velocity.y]
      );
      this.lastAiCorrection = aiCorrection ? [...aiCorrection] : null;

      H = Matrix.zeros(2, 15);
      H.set(0, 3, 1); // z[0] is East velocity error -> maps to state 3 (vx)
      H.set(1, 4, 1); // z[1] is North velocity error -> maps to state 4 (vy)

      R = Matrix.eye(2).mul(0.5);

      if (aiCorrection) {
        z = new Matrix([
          [aiCorrection[0]],
          [aiCorrection[1]]
        ]);
      } else {
        z = Matrix.zeros(2, 1);
        R = Matrix.eye(2).mul(1000); // Discard
      }

      this.applyMeasurementUpdate(H, z, R, 6.0);

      if (this.isAttitudeInitialized) {
        this.applyNhc(0.05, 0.01);
      }
    }

    this.applyVelocityGuard();
    return !this.lastGnssRejected;
  }

  /**
   * Zero Velocity Update (ZUPT)
   * Forces the EKF to correct velocity drift when stationary.
   */
  public updateZupt() {
    const H = Matrix.zeros(3, 15);
    H.set(0, 3, 1);
    H.set(1, 4, 1);
    H.set(2, 5, 1);

    // High confidence that velocity is 0
    const R = Matrix.eye(3).mul(0.001);

    const z = new Matrix([
      [-this.ins.velocity.x],
      [-this.ins.velocity.y],
      [-this.ins.velocity.z]
    ]);

    this.applyMeasurementUpdate(H, z, R, Infinity);

    this.wasZuptActive = true;

    // Hard override velocity to exactly 0 to eliminate any micro-jitter
    this.ins.velocity.x = 0;
    this.ins.velocity.y = 0;
    this.ins.velocity.z = 0;

    // Reset velocity error in the EKF state vector
    this.x.set(3, 0, 0);
    this.x.set(4, 0, 0);
    this.x.set(5, 0, 0);

    // Bound estimated biases to realistic physical ranges
    for (let i = 9; i < 12; i++) {
      const b = this.x.get(i, 0);
      if (Math.abs(b) > 1.5) this.x.set(i, 0, Math.sign(b) * 1.5);
    }
    for (let i = 12; i < 15; i++) {
      const b = this.x.get(i, 0);
      if (Math.abs(b) > 0.1) this.x.set(i, 0, Math.sign(b) * 0.1);
    }

    this.applyVelocityGuard();
  }

  public getPosition() {
    return this.ins.position;
  }

  public getBiases() {
    return {
      accel: { x: this.x.get(9, 0), y: this.x.get(10, 0), z: this.x.get(11, 0) },
      gyro: { x: this.x.get(12, 0), y: this.x.get(13, 0), z: this.x.get(14, 0) }
    };
  }

  private applyVelocityGuard() {
    const v = this.ins.velocity;

    if (this.previousVelocity === null) {
      this.previousVelocity = { x: v.x, y: v.y, z: v.z };
      return;
    }

    const MAX_VELOCITY_JUMP = 5.0; // max ~18 km/h velocity change per cycle
    const dx = v.x - this.previousVelocity.x;
    const dy = v.y - this.previousVelocity.y;
    const dz = v.z - this.previousVelocity.z;
    const jumpMag = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (jumpMag > MAX_VELOCITY_JUMP) {
      console.warn(
        `[Velocity Jump Guard] Warning: |velocity| changed by ${jumpMag.toFixed(2)} m/s in single cycle (> ${MAX_VELOCITY_JUMP} m/s). Clamping.\n` +
        `State Dump:\n` +
        `  Raw Accel: [${this.lastRawAccel.map(n => n.toFixed(3)).join(', ')}]\n` +
        `  Filtered Accel: ${JSON.stringify(this.ins.getFilteredAccel())}\n` +
        `  Vel Q: ${this.lastVelQ.toFixed(4)}\n` +
        `  ZUPT Active: ${this.wasZuptActive}, Post-ZUPT Cooldown: ${this.postZuptCooldown}\n` +
        `  AI Correction: ${JSON.stringify(this.lastAiCorrection)}\n` +
        `  Current Velocity: [${v.x.toFixed(3)}, ${v.y.toFixed(3)}, ${v.z.toFixed(3)}]`
      );
      const scale = MAX_VELOCITY_JUMP / jumpMag;
      v.x = this.previousVelocity.x + dx * scale;
      v.y = this.previousVelocity.y + dy * scale;
      v.z = this.previousVelocity.z + dz * scale;
    }

    // Absolute velocity clamp (runaway safety guard)
    if (Math.abs(v.x) > 60 || Math.abs(v.y) > 60 || Math.abs(v.z) > 60) {
      console.warn(`[Sanity Guard] Runaway velocity: x=${v.x.toFixed(2)}, y=${v.y.toFixed(2)}, z=${v.z.toFixed(2)} m/s! Clamping.`);
      v.x = Math.max(-60, Math.min(60, v.x));
      v.y = Math.max(-60, Math.min(60, v.y));
      v.z = Math.max(-60, Math.min(60, v.z));
    }

    this.previousVelocity = { x: v.x, y: v.y, z: v.z };
  }

  public notifyAttitudeInitialized() {
    this.isAttitudeInitialized = true;
  }

  public notifyZuptReleased() {
    if (this.wasZuptActive) {
      this.postZuptCooldown = this.POST_ZUPT_COOLDOWN_CYCLES;
      this.wasZuptActive = false;
      for (let i = 0; i < 3; i++) {
        this.P.set(i, i, Math.max(this.P.get(i, i), 5.0));
      }
      for (let i = 3; i < 6; i++) {
        this.P.set(i, i, Math.max(this.P.get(i, i), 5.0));
      }
    }
  }
}

function inverse(m: Matrix): Matrix | null {
  try {
    return mlInverse(m);
  } catch {
    return null;
  }
}
