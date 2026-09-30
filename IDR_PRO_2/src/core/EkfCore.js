"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.EkfCore = void 0;
const ml_matrix_1 = require("ml-matrix");
const InsMechanization_1 = require("./InsMechanization");
const GnssQualityStateMachine_1 = require("./GnssQualityStateMachine");
const AiMotionModel_1 = require("./AiMotionModel");
class EkfCore {
    // 15-state vector:
    // 0-2: Position (x,y,z)
    // 3-5: Velocity (x,y,z)
    // 6-8: Attitude errors (pitch, roll, yaw)
    // 9-11: Accel bias (x,y,z)
    // 12-14: Gyro bias (x,y,z)
    x;
    P;
    enableYawAlignment = false;
    enableNHC = false;
    enableBumpGate = false;
    bumpGateCount = 0;
    yawOffsetRad = 0; // The angle between the phone's forward axis and the vehicle's forward axis
    yawOffsetActive = false; // Whether we have a valid estimate
    ins;
    gnssState;
    aiModel;
    constructor() {
        this.x = ml_matrix_1.Matrix.zeros(15, 1);
        this.P = ml_matrix_1.Matrix.zeros(15, 15);
        for (let i = 0; i < 3; i++)
            this.P.set(i, i, 0.1); // Position
        for (let i = 3; i < 6; i++)
            this.P.set(i, i, 0.1); // Velocity
        for (let i = 6; i < 9; i++)
            this.P.set(i, i, 0.001); // Attitude
        for (let i = 9; i < 12; i++)
            this.P.set(i, i, 0.0001); // Accel bias
        for (let i = 12; i < 15; i++)
            this.P.set(i, i, 0.00001); // Gyro bias
        this.ins = new InsMechanization_1.InsMechanization();
        this.gnssState = new GnssQualityStateMachine_1.GnssQualityStateMachine();
        this.aiModel = new AiMotionModel_1.AiMotionModel();
    }
    // ZUPT state tracking for Q scheduling
    wasZuptActive = false;
    postZuptCooldown = 0;
    POST_ZUPT_COOLDOWN_CYCLES = 5; // ~0.5s at 10Hz
    previousVelocity = null;
    lastRawAccel = [0, 0, 9.81];
    lastVelQ = 0.01;
    lastAiCorrection = null;
    // Set to true once FusionRuntime completes stationary attitude initialisation.
    // NHC must NOT fire before this: an identity C_b_n projects full gravity into
    // navigation-frame lateral/vertical velocity, instantly saturating the ±60 guard.
    isAttitudeInitialized = false;
    getIns() {
        return this.ins;
    }
    getGnssState() {
        return this.gnssState;
    }
    getAiModel() {
        return this.aiModel;
    }
    currentTime = 0;
    /**
     * Predict step (runs constantly at IMU rate)
     */
    predict(dt, accel, gyro) {
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
        }
        else if (!this.isAttitudeInitialized) {
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
        const posBefore = { ...this.ins.position };
        this.ins.predict(dt, { x: accX, y: accY, z: accZ }, { x: gyrX, y: gyrY, z: gyrZ });
        // console.log(`[PREDICT] dt=${dt.toFixed(3)}, posBefore: ${posBefore.x.toFixed(3)}, ${posBefore.y.toFixed(3)}, posAfter: ${this.ins.position.x.toFixed(3)}, ${this.ins.position.y.toFixed(3)})`);
        // 2. Propagate Covariance P = F * P * F^T + Q
        const F = ml_matrix_1.Matrix.eye(15);
        // Position depends on velocity: p_new = p_old + v * dt
        F.set(0, 3, dt);
        F.set(1, 4, dt);
        F.set(2, 5, dt);
        // Rigorous F-matrix construction (Body to Navigation frame projection)
        const C_b_n = this.ins.lastRotationMatrix;
        const f_n = this.ins.lastSpecificForce;
        // Velocity error from Attitude error: -[f^n x] * dt
        // [f^n x] = [   0   -f_z   f_y ]
        //           [  f_z    0   -f_x ]
        //           [ -f_y   f_x    0  ]
        // So -[f^n x] = [   0    f_z  -f_y ]
        //               [ -f_z    0    f_x ]
        //               [  f_y  -f_x    0  ]
        F.set(3, 6, 0);
        F.set(3, 7, f_n.z * dt);
        F.set(3, 8, -f_n.y * dt);
        F.set(4, 6, -f_n.z * dt);
        F.set(4, 7, 0);
        F.set(4, 8, f_n.x * dt);
        F.set(5, 6, f_n.y * dt);
        F.set(5, 7, -f_n.x * dt);
        F.set(5, 8, 0);
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
        // Process noise Q (tunable placeholders - calibrate against actual IMU noise characteristics)
        const Q = ml_matrix_1.Matrix.zeros(15, 15);
        for (let i = 0; i < 3; i++)
            Q.set(i, i, 0.1 * dt); // position
        // Phase 4: Bump / Pothole Gate
        let bumpDetected = false;
        if (this.enableBumpGate) {
            // Simple high-pass proxy: deviation of raw accel magnitude from 1g, and high gyro magnitude
            const accelMag = Math.sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
            const gyroMag = Math.sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
            const accelDeviation = Math.abs(accelMag - 9.81);
            // Thresholds: e.g. > 3 m/s^2 deviation or > 1.0 rad/s rotation
            if (accelDeviation > 3.0 || gyroMag > 1.0) {
                bumpDetected = true;
                this.bumpGateCount++;
            }
        }
        // Velocity Q: inflate during post-ZUPT cooldown to allow graceful transition
        // Also inflate if bump/pothole detected to reduce trust in IMU during vibration
        let velQ = 0.1 * dt;
        if (this.postZuptCooldown > 0) {
            velQ = 2.0 * dt; // 2x inflation during transition (cooldown)
            this.postZuptCooldown--;
        }
        if (bumpDetected) {
            velQ = Math.max(velQ, 5.0 * dt); // Massive inflation during bumps
        }
        this.lastVelQ = velQ;
        for (let i = 3; i < 6; i++)
            Q.set(i, i, velQ);
        for (let i = 6; i < 9; i++)
            Q.set(i, i, bumpDetected ? 0.001 * dt : 0.000001 * dt); // attitude (gyro noise ~1e-6)
        for (let i = 9; i < 12; i++)
            Q.set(i, i, 0.00001 * dt); // accel bias (slow drift)
        for (let i = 12; i < 15; i++)
            Q.set(i, i, 0.000001 * dt); // gyro bias (very slow drift)
        this.P = F.mmul(this.P).mmul(F.transpose()).add(Q);
        this.applyVelocityGuard();
    }
    /**
     * Applies a generalized Kalman measurement update in Joseph form with Mahalanobis gating.
     * Feeds back errors to INS mechanization (closed-loop EKF) and resets error states 0-8.
     */
    applyMeasurementUpdate(H, z, R, chiSquareThreshold = 6.0) {
        const S = H.mmul(this.P).mmul(H.transpose()).add(R);
        const S_inv = inverse(S);
        if (!S_inv)
            return false;
        const mahalanobisSq = z.transpose().mmul(S_inv).mmul(z).get(0, 0);
        if (mahalanobisSq >= chiSquareThreshold) {
            return false; // Outlier rejected
        }
        const K = this.P.mmul(H.transpose()).mmul(S_inv);
        const dx = K.mmul(z);
        this.x = this.x.add(dx);
        // Joseph-form covariance update
        const I = ml_matrix_1.Matrix.eye(15);
        const IKH = I.sub(K.mmul(H));
        this.P = IKH.mmul(this.P).mmul(IKH.transpose()).add(K.mmul(R).mmul(K.transpose()));
        // Closed-loop state feedback to INS
        this.ins.position.x += this.x.get(0, 0);
        this.ins.position.y += this.x.get(1, 0);
        this.ins.position.z += this.x.get(2, 0);
        this.ins.velocity.x += this.x.get(3, 0);
        this.ins.velocity.y += this.x.get(4, 0);
        this.ins.velocity.z += this.x.get(5, 0);
        // Clamp attitude error correction to max ~3 degrees (0.05 rad) per update to prevent tilt runaway
        const maxAttJump = 0.05;
        const dPitch = Math.max(-maxAttJump, Math.min(maxAttJump, this.x.get(6, 0)));
        const dRoll = Math.max(-maxAttJump, Math.min(maxAttJump, this.x.get(7, 0)));
        const dYaw = Math.max(-maxAttJump, Math.min(maxAttJump, this.x.get(8, 0)));
        this.ins.attitude.pitch += dPitch;
        this.ins.attitude.roll += dRoll;
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
    applyNhc(R_lat = 0.05, R_vert = 0.01) {
        const C_b_n = this.ins.lastRotationMatrix;
        const vel = this.ins.velocity;
        // Adjust the rotation matrix for the phone-to-vehicle yaw offset
        // C_b_v = Rz(-yawOffsetRad)
        // Then C_v_n = C_b_n * C_v_b = C_b_n * Rz(yawOffsetRad)
        // The lateral vehicle axis in the navigation frame is the 2nd column of C_v_n
        const cosY = Math.cos(this.yawOffsetRad);
        const sinY = Math.sin(this.yawOffsetRad);
        // C_v_n column 0 (forward): 
        // fx = C00*cosY + C01*sinY
        // fy = C10*cosY + C11*sinY
        // fz = C20*cosY + C21*sinY
        // C_v_n column 1 (lateral):
        // lx = -C00*sinY + C01*cosY
        // ly = -C10*sinY + C11*cosY
        // lz = -C20*sinY + C21*cosY
        // C_v_n column 2 (vertical) remains C_b_n column 2 because we only rotate yaw (around z)
        // Actually, phone might be pitched, so vehicle vertical is not exactly phone vertical, but
        // usually pitch/roll are estimated via gravity. Assuming phone Z is vehicle Z for now.
        const l_x = -C_b_n[0][0] * sinY + C_b_n[0][1] * cosY;
        const l_y = -C_b_n[1][0] * sinY + C_b_n[1][1] * cosY;
        const l_z = -C_b_n[2][0] * sinY + C_b_n[2][1] * cosY;
        const v_x = C_b_n[0][2];
        const v_y = C_b_n[1][2];
        const v_z = C_b_n[2][2];
        const v_lat = l_x * vel.x + l_y * vel.y + l_z * vel.z;
        const v_vert = v_x * vel.x + v_y * vel.y + v_z * vel.z;
        const H = ml_matrix_1.Matrix.zeros(2, 15);
        // Row 0: lateral velocity constraint
        H.set(0, 3, l_x);
        H.set(0, 4, l_y);
        H.set(0, 5, l_z);
        // Row 1: vertical velocity constraint
        H.set(1, 3, v_x);
        H.set(1, 4, v_y);
        H.set(1, 5, v_z);
        const z = new ml_matrix_1.Matrix([
            [-v_lat],
            [-v_vert]
        ]);
        const R = ml_matrix_1.Matrix.zeros(2, 2);
        R.set(0, 0, R_lat);
        R.set(1, 1, R_vert);
        this.applyMeasurementUpdate(H, z, R, 12.0);
        this.applyVelocityGuard();
    }
    lastGnssPos = null;
    lastGnssUpdateTime = 0;
    /**
     * Update step with GNSS or AI (runs when data is available)
     */
    updateGnss(gnssPos, gnssVel, accuracy, recentImuWindow = []) {
        // 1. Determine GNSS Quality
        const rScale = this.gnssState.updateState(accuracy);
        const state = this.gnssState.getState();
        let z; // Measurement residual
        let H; // Observation matrix
        let R; // Measurement noise covariance
        let threshold = 6.0;
        // Derived velocity fallback from GNSS position delta if velocity is null.
        // Use this.currentTime (accumulated from predict() dt values) rather than Date.now(),
        // because in test environments Date.now() is nearly constant across a synchronous
        // loop, making dtGnss ≈ 0 (clamped to 0.1s) and producing 10×-inflated velocity
        // estimates that are then rejected by the speed < 40 guard.
        let effectiveVel = gnssVel;
        if (effectiveVel === null && this.lastGnssPos !== null && (state === 'GOOD' || state === 'DEGRADED')) {
            const dtGnss = this.lastGnssUpdateTime > 0
                ? Math.min(Math.max(this.currentTime - this.lastGnssUpdateTime, 0.1), 3.0)
                : 0; // No valid prior time — cannot derive velocity
            if (dtGnss >= 0.1 && dtGnss <= 3.0) {
                const vx = (gnssPos[0] - this.lastGnssPos[0]) / dtGnss;
                const vy = (gnssPos[1] - this.lastGnssPos[1]) / dtGnss;
                const vz = (gnssPos[2] - this.lastGnssPos[2]) / dtGnss;
                const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
                if (speed < 40) {
                    effectiveVel = [vx, vy, vz];
                }
            }
        }
        if (state === 'GOOD' || state === 'DEGRADED') {
            this.lastGnssPos = [...gnssPos];
            this.lastGnssUpdateTime = this.currentTime;
            // Yaw alignment logic
            if (this.enableYawAlignment && effectiveVel !== null) {
                const speed = Math.sqrt(effectiveVel[0] * effectiveVel[0] + effectiveVel[1] * effectiveVel[1]);
                if (speed > 3.0) { // e.g. 3 m/s threshold
                    // GNSS Course
                    const gnssCourse = Math.atan2(effectiveVel[0], effectiveVel[1]); // Atan2(E, N)
                    // INS Forward Course (phone Y axis)
                    const C_b_n = this.ins.lastRotationMatrix;
                    const insY_E = C_b_n[0][1];
                    const insY_N = C_b_n[1][1];
                    const insCourse = Math.atan2(insY_E, insY_N);
                    let angleDiff = gnssCourse - insCourse;
                    // Normalize to -PI to PI
                    while (angleDiff > Math.PI)
                        angleDiff -= 2 * Math.PI;
                    while (angleDiff < -Math.PI)
                        angleDiff += 2 * Math.PI;
                    // Low-pass filter the offset
                    if (!this.yawOffsetActive) {
                        this.yawOffsetRad = angleDiff;
                        this.yawOffsetActive = true;
                    }
                    else {
                        const alpha = 0.05; // Tunable smoothing factor
                        this.yawOffsetRad = (1 - alpha) * this.yawOffsetRad + alpha * angleDiff;
                    }
                }
            }
        }
        if (state === 'GOOD' || state === 'DEGRADED') {
            // Dynamic base variance scaled to reported GNSS accuracy (or standard ~1m if unknown)
            const basePosVar = accuracy !== null && accuracy > 0 ? Math.max(0.1, (accuracy * accuracy) / 9.0) : 1.0;
            if (effectiveVel !== null) {
                // GNSS Update (Position & Velocity)
                // Observation: 6D (pos x,y,z, vel x,y,z)
                H = ml_matrix_1.Matrix.zeros(6, 15);
                for (let i = 0; i < 6; i++) {
                    H.set(i, i, 1);
                }
                R = ml_matrix_1.Matrix.eye(6).mul(basePosVar);
                for (let i = 3; i < 6; i++) {
                    R.set(i, i, gnssVel !== null ? 0.1 : 5.0);
                }
                if (state === 'DEGRADED') {
                    R = R.mulColumnVector(ml_matrix_1.Matrix.columnVector(Array(6).fill(rScale)));
                }
                // Residual z = GNSS - INS
                z = new ml_matrix_1.Matrix([
                    [gnssPos[0] - this.ins.position.x],
                    [gnssPos[1] - this.ins.position.y],
                    [gnssPos[2] - this.ins.position.z],
                    [effectiveVel[0] - this.ins.velocity.x],
                    [effectiveVel[1] - this.ins.velocity.y],
                    [effectiveVel[2] - this.ins.velocity.z],
                ]);
                threshold = 25.0;
            }
            else {
                // GNSS Update (Position Only)
                // Observation: 3D (pos x,y,z)
                H = ml_matrix_1.Matrix.zeros(3, 15);
                for (let i = 0; i < 3; i++) {
                    H.set(i, i, 1);
                }
                R = ml_matrix_1.Matrix.eye(3).mul(basePosVar);
                if (state === 'DEGRADED') {
                    R = R.mulColumnVector(ml_matrix_1.Matrix.columnVector(Array(3).fill(rScale)));
                }
                // Residual z = GNSS - INS
                z = new ml_matrix_1.Matrix([
                    [gnssPos[0] - this.ins.position.x],
                    [gnssPos[1] - this.ins.position.y],
                    [gnssPos[2] - this.ins.position.z],
                ]);
                threshold = 7.8;
            }
            this.applyMeasurementUpdate(H, z, R, threshold);
        }
        else {
            // WEAK_LOST State: Use AI pseudo-measurement and NHC
            // Phase 4: Bump / Pothole Gate for AI
            // Check the recent IMU window for large magnitude deviations (bumps)
            let windowHasBump = false;
            if (this.enableBumpGate) {
                for (const sample of recentImuWindow) {
                    const accMag = Math.sqrt(sample[0] * sample[0] + sample[1] * sample[1] + sample[2] * sample[2]);
                    const gyrMag = Math.sqrt(sample[3] * sample[3] + sample[4] * sample[4] + sample[5] * sample[5]);
                    if (Math.abs(accMag - 9.81) > 3.0 || gyrMag > 1.0) {
                        windowHasBump = true;
                        break;
                    }
                }
            }
            // 1. Ask AI for velocity error correction
            let aiCorrection = null;
            if (!windowHasBump) {
                aiCorrection = this.aiModel.predictError(state, recentImuWindow, [this.ins.velocity.x, this.ins.velocity.y]);
            }
            else {
                console.warn("[Bump Gate] Skipping AI update due to severe vibration/bump in IMU window.");
            }
            this.lastAiCorrection = aiCorrection ? [...aiCorrection] : null;
            // We'll update 2D velocity (vx = East, vy = North) from AI
            H = ml_matrix_1.Matrix.zeros(2, 15);
            H.set(0, 3, 1); // z[0] is East velocity error -> maps to state 3 (vx)
            H.set(1, 4, 1); // z[1] is North velocity error -> maps to state 4 (vy)
            // AI confidence is lower than GNSS, so base R is higher
            R = ml_matrix_1.Matrix.eye(2).mul(0.5);
            if (aiCorrection) {
                // AI predicts the *error* in INS velocity.
                // So residual z = aiCorrection
                z = new ml_matrix_1.Matrix([
                    [aiCorrection[0]],
                    [aiCorrection[1]]
                ]);
            }
            else {
                // Fallback if AI fails: zero-update (no correction)
                z = ml_matrix_1.Matrix.zeros(2, 1);
                R = ml_matrix_1.Matrix.eye(2).mul(1000); // Discard
            }
            this.applyMeasurementUpdate(H, z, R, 6.0);
            // 2. Apply Non-Holonomic Constraints (NHC) in WEAK_LOST mode
            // Guard: only apply NHC after attitude is initialized.
            if (this.enableNHC && this.isAttitudeInitialized && this.yawOffsetActive) {
                this.applyNhc(0.05, 0.01);
            }
        }
        this.applyVelocityGuard();
    }
    /**
     * Zero Velocity Update (ZUPT)
     * Forces the EKF to correct velocity drift when stationary.
     */
    updateZupt() {
        const H = ml_matrix_1.Matrix.zeros(3, 15);
        H.set(0, 3, 1);
        H.set(1, 4, 1);
        H.set(2, 5, 1);
        // High confidence that velocity is 0
        const R = ml_matrix_1.Matrix.eye(3).mul(0.001);
        // Residual z = 0 - INS Velocity
        const z = new ml_matrix_1.Matrix([
            [-this.ins.velocity.x],
            [-this.ins.velocity.y],
            [-this.ins.velocity.z]
        ]);
        this.applyMeasurementUpdate(H, z, R, Infinity);
        // Track ZUPT state for Q scheduling: when ZUPT stops, start cooldown
        this.wasZuptActive = true;
        // Hard override velocity to exactly 0 to eliminate any micro-jitter (0.02 - 0.1 m/s)
        this.ins.velocity.x = 0;
        this.ins.velocity.y = 0;
        this.ins.velocity.z = 0;
        // Reset velocity error in the EKF state vector to prevent accumulation
        this.x.set(3, 0, 0);
        this.x.set(4, 0, 0);
        this.x.set(5, 0, 0);
        this.applyVelocityGuard();
    }
    getPosition() {
        return this.ins.position;
    }
    getBiases() {
        return {
            accel: { x: this.x.get(9, 0), y: this.x.get(10, 0), z: this.x.get(11, 0) },
            gyro: { x: this.x.get(12, 0), y: this.x.get(13, 0), z: this.x.get(14, 0) }
        };
    }
    applyVelocityGuard() {
        const v = this.ins.velocity;
        if (this.previousVelocity === null) {
            this.previousVelocity = { x: v.x, y: v.y, z: v.z };
            return;
        }
        // Per-step velocity-jump rate limiter / monitor:
        // Log a warning if |velocity| changes by > 3 m/s in a single fusion cycle (e.g. 100ms)
        // and dump the full state (raw accel, filtered accel, Q at that step, ZUPT state, AI correction).
        const MAX_VELOCITY_JUMP = 3.0; // m/s per cycle
        const dx = v.x - this.previousVelocity.x;
        const dy = v.y - this.previousVelocity.y;
        const dz = v.z - this.previousVelocity.z;
        const jumpMag = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (jumpMag > MAX_VELOCITY_JUMP) {
            console.warn(`[Velocity Jump Guard] Warning: |velocity| changed by ${jumpMag.toFixed(2)} m/s in single cycle (> ${MAX_VELOCITY_JUMP} m/s).\n` +
                `State Dump:\n` +
                `  Raw Accel: [${this.lastRawAccel.map(n => n.toFixed(3)).join(', ')}]\n` +
                `  Filtered Accel: ${JSON.stringify(this.ins.getFilteredAccel())}\n` +
                `  Vel Q: ${this.lastVelQ.toFixed(4)}\n` +
                `  ZUPT Active: ${this.wasZuptActive}, Post-ZUPT Cooldown: ${this.postZuptCooldown}\n` +
                `  AI Correction: ${JSON.stringify(this.lastAiCorrection)}\n` +
                `  Current Velocity: [${v.x.toFixed(3)}, ${v.y.toFixed(3)}, ${v.z.toFixed(3)}]`);
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
    /**
     * Called by FusionRuntime once the initial stationary alignment is complete and
     * the rotation matrix C_b_n is valid. Enables NHC and other attitude-dependent guards.
     */
    notifyAttitudeInitialized() {
        this.isAttitudeInitialized = true;
    }
    /**
     * Called by FusionRuntime when ZUPT was active last cycle but is no longer.
     * Triggers post-ZUPT cooldown for Q scheduling.
     */
    notifyZuptReleased() {
        if (this.wasZuptActive) {
            this.postZuptCooldown = this.POST_ZUPT_COOLDOWN_CYCLES;
            this.wasZuptActive = false;
        }
    }
    /**
     * Applies a soft map-matched position constraint.
     * @param pos Navigation-frame position [x, y, z] (usually z is unused/0)
     * @param R_pos Measurement noise covariance (e.g. 5.0 for 5m confidence)
     */
    updateMapMatch(pos, R_pos = 5.0) {
        const H = ml_matrix_1.Matrix.zeros(2, 15);
        H.set(0, 0, 1);
        H.set(1, 1, 1);
        const z = new ml_matrix_1.Matrix([
            [pos[0] - this.ins.position.x],
            [pos[1] - this.ins.position.y]
        ]);
        const R = ml_matrix_1.Matrix.eye(2).mul(R_pos);
        // Mahalanobis gate around 15.0 to accept gentle corrections but reject huge jumps
        this.applyMeasurementUpdate(H, z, R, 15.0);
        this.applyVelocityGuard();
    }
}
exports.EkfCore = EkfCore;
// Helper for Matrix Inverse (ml-matrix inverse() can fail on singular)
function inverse(m) {
    try {
        return (0, ml_matrix_1.inverse)(m);
    }
    catch (e) {
        return null;
    }
}
