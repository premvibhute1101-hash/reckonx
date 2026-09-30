export class InsMechanization {
  // State variables
  public position: { x: number; y: number; z: number } = { x: 0, y: 0, z: 0 }; // ENU frame
  public velocity: { x: number; y: number; z: number } = { x: 0, y: 0, z: 0 }; // ENU frame
  public attitude: { roll: number; pitch: number; yaw: number } = { roll: 0, pitch: 0, yaw: 0 };

  // Exposed for rigorous EKF F-Matrix propagation
  public lastRotationMatrix: number[][] = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
  ];
  public lastSpecificForce: { x: number; y: number; z: number } = { x: 0, y: 0, z: 0 }; // f^n

  // Constants
  private gravity = 9.81;

  // 2nd-order Butterworth LPF state (cutoff: 2.0 Hz)
  private accelXHist: number[] = [0, 0];
  private accelYHist: number[] = [0, 0];
  private accelZHist: number[] = [0, 0];
  private filteredXHist: number[] = [0, 0];
  private filteredYHist: number[] = [0, 0];
  private filteredZHist: number[] = [0, 0];
  private filteredAccel: { x: number; y: number; z: number } | null = null;

  // Running estimate of body-frame gravity vector (Anti-Gravity tracking)
  public estimatedGravity: { x: number; y: number; z: number } = { x: 0, y: 0, z: 9.81 };

  public setState(
    position: { x: number; y: number; z: number },
    velocity: { x: number; y: number; z: number },
    attitude: { roll: number; pitch: number; yaw: number }
  ) {
    this.position = { ...position };
    this.velocity = { ...velocity };
    this.attitude = { ...attitude };
  }

  public initializeAttitude(accel: { x: number; y: number; z: number }) {
    const mag = Math.sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    this.estimatedGravity = { x: accel.x, y: accel.y, z: accel.z };
    if (mag > 3.0) {
      // Specific force reading with gravity baseline:
      // Roll (rotation around body X-axis) = atan2(ay, az)
      // Pitch (rotation around body Y-axis) = atan2(-ax, sqrt(ay^2 + az^2))
      this.attitude.roll = Math.atan2(accel.y, accel.z);
      this.attitude.pitch = Math.atan2(-accel.x, Math.sqrt(accel.y * accel.y + accel.z * accel.z));
    } else {
      // Pure linear acceleration reading (gravity already removed)
      this.attitude.roll = 0;
      this.attitude.pitch = 0;
    }
    // Also seed the LPF history with the initial reading to prevent startup transients
    this.accelXHist = [accel.x, accel.x];
    this.accelYHist = [accel.y, accel.y];
    this.accelZHist = [accel.z, accel.z];
    this.filteredXHist = [accel.x, accel.x];
    this.filteredYHist = [accel.y, accel.y];
    this.filteredZHist = [accel.z, accel.z];
    this.filteredAccel = { ...accel };
  }

  public getFilteredAccel(): { x: number; y: number; z: number } | null {
    return this.filteredAccel ? { ...this.filteredAccel } : null;
  }

  /**
   * Applies a 2nd-order Butterworth digital low-pass filter to accelerometer data.
   * Cutoff = 2.0Hz to attenuate heel-strike transients while preserving walking dynamics.
   * Coefficients are dynamically calculated based on dt via bilinear transform.
   */
  private filterAccel(dt: number, raw: { x: number; y: number; z: number }): { x: number; y: number; z: number } {
    if (this.filteredAccel === null) {
      this.accelXHist = [raw.x, raw.x];
      this.accelYHist = [raw.y, raw.y];
      this.accelZHist = [raw.z, raw.z];
      this.filteredXHist = [raw.x, raw.x];
      this.filteredYHist = [raw.y, raw.y];
      this.filteredZHist = [raw.z, raw.z];
      this.filteredAccel = { ...raw };
      return { ...raw };
    }

    const fc = 2.0; // Cutoff frequency (Hz)
    // Protection against dt = 0 or extremely small
    const safeDt = Math.max(dt, 0.001); 
    const w0 = 2 * Math.PI * fc * safeDt;
    const alpha = Math.sin(w0) / Math.SQRT2;

    const a0 = 1 + alpha;
    const b0 = ((1 - Math.cos(w0)) / 2) / a0;
    const b1 = (1 - Math.cos(w0)) / a0;
    const b2 = ((1 - Math.cos(w0)) / 2) / a0;
    const a1 = (-2 * Math.cos(w0)) / a0;
    const a2 = (1 - alpha) / a0;

    const processAxis = (x: number, xHist: number[], yHist: number[]) => {
      const y = b0 * x + b1 * xHist[0] + b2 * xHist[1] - a1 * yHist[0] - a2 * yHist[1];
      xHist[1] = xHist[0];
      xHist[0] = x;
      yHist[1] = yHist[0];
      yHist[0] = y;
      return y;
    };

    this.filteredAccel = {
      x: processAxis(raw.x, this.accelXHist, this.filteredXHist),
      y: processAxis(raw.y, this.accelYHist, this.filteredYHist),
      z: processAxis(raw.z, this.accelZHist, this.filteredZHist),
    };
    return { ...this.filteredAccel };
  }

  /**
   * Predicts the next state given IMU readings.
   * @param dt Time step in seconds (e.g., 0.1 for 10Hz)
   * @param accel Accelerometer reading in body frame (m/s^2)
   * @param gyro Gyroscope reading in body frame (rad/s)
   */
  public predict(
    dt: number,
    accel: { x: number; y: number; z: number },
    gyro: { x: number; y: number; z: number }
  ) {
    // 0. Low-pass filter accelerometer to attenuate heel-strike impact transients
    const fAccel = this.filterAccel(dt, accel);

    // 1. Update attitude (Euler angles integration: gyro.x=roll, gyro.y=pitch, gyro.z=yaw)
    let newRoll = this.attitude.roll + gyro.x * dt;
    let newPitch = this.attitude.pitch + gyro.y * dt;
    this.attitude.yaw += gyro.z * dt;

    // 2. Transform body frame acceleration to navigation frame (ENU) using gyro-integrated attitude
    let cRoll = Math.cos(newRoll);
    let sRoll = Math.sin(newRoll);
    let cPitch = Math.cos(newPitch);
    let sPitch = Math.sin(newPitch);
    const cYaw = Math.cos(this.attitude.yaw);
    const sYaw = Math.sin(this.attitude.yaw);

    // Standard Z-Y-X (Yaw-Pitch-Roll) Rotation Matrix R_b^n (Body to Nav ENU)
    let R11 = cYaw * cPitch;
    let R12 = cYaw * sPitch * sRoll - sYaw * cRoll;
    let R13 = cYaw * sPitch * cRoll + sYaw * sRoll;

    let R21 = sYaw * cPitch;
    let R22 = sYaw * sPitch * sRoll + cYaw * cRoll;
    let R23 = sYaw * sPitch * cRoll - cYaw * sRoll;

    let R31 = -sPitch;
    let R32 = cPitch * sRoll;
    let R33 = cPitch * cRoll;

    const init_a_n_x = R11 * fAccel.x + R12 * fAccel.y + R13 * fAccel.z;
    const init_a_n_y = R21 * fAccel.x + R22 * fAccel.y + R23 * fAccel.z;
    const horizSpecificForce = Math.sqrt(init_a_n_x * init_a_n_x + init_a_n_y * init_a_n_y);

    // 3. Dynamic Gravity Extraction (Anti-Gravity) & Tilt Correction
    // ONLY update the gravity estimate when strictly stationary:
    // horizontal specific force < 0.05 m/s^2, rotation rate < 0.05 rad/s, and total accel either near 9.81 m/s^2 (+/- 0.3 m/s^2) or near 0 m/s^2 (< 0.05 m/s^2).
    const accelMag = Math.sqrt(fAccel.x * fAccel.x + fAccel.y * fAccel.y + fAccel.z * fAccel.z);
    const gyroMag = Math.sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
    const isStrictlyStationary = horizSpecificForce < 0.05 && gyroMag < 0.05 && (Math.abs(accelMag - this.gravity) < 0.3 || accelMag < 0.05);

    if (isStrictlyStationary) {
      // Low-pass filter update for estimated body-frame gravity vector (alpha = 0.05)
      const alphaG = 0.05;
      this.estimatedGravity.x = (1 - alphaG) * this.estimatedGravity.x + alphaG * fAccel.x;
      this.estimatedGravity.y = (1 - alphaG) * this.estimatedGravity.y + alphaG * fAccel.y;
      this.estimatedGravity.z = (1 - alphaG) * this.estimatedGravity.z + alphaG * fAccel.z;

      // Continuous tilt leveling towards tracked gravity vector if gravity is present
      if (accelMag > 3.0) {
        const rollAcc = Math.atan2(this.estimatedGravity.y, this.estimatedGravity.z);
        const pitchAcc = Math.atan2(-this.estimatedGravity.x, Math.sqrt(this.estimatedGravity.y * this.estimatedGravity.y + this.estimatedGravity.z * this.estimatedGravity.z));

        const alphaAtt = Math.min(0.2, 0.4 * dt);
        newPitch = newPitch + alphaAtt * (pitchAcc - newPitch);
        newRoll = newRoll + alphaAtt * (rollAcc - newRoll);

        cRoll = Math.cos(newRoll);
        sRoll = Math.sin(newRoll);
        cPitch = Math.cos(newPitch);
        sPitch = Math.sin(newPitch);

        R11 = cYaw * cPitch;
        R12 = cYaw * sPitch * sRoll - sYaw * cRoll;
        R13 = cYaw * sPitch * cRoll + sYaw * sRoll;

        R21 = sYaw * cPitch;
        R22 = sYaw * sPitch * sRoll + cYaw * cRoll;
        R23 = sYaw * sPitch * cRoll - cYaw * sRoll;

        R31 = -sPitch;
        R32 = cPitch * sRoll;
        R33 = cPitch * cRoll;
      }
    }

    this.attitude.roll = newRoll;
    this.attitude.pitch = newPitch;

    this.lastRotationMatrix = [
      [R11, R12, R13],
      [R21, R22, R23],
      [R31, R32, R33]
    ];

    // Subtract dynamically tracked gravity in body frame to isolate pure linear acceleration
    const a_lin_bx = fAccel.x - this.estimatedGravity.x;
    const a_lin_by = fAccel.y - this.estimatedGravity.y;
    const a_lin_bz = fAccel.z - this.estimatedGravity.z;

    const raw_a_n_x = R11 * a_lin_bx + R12 * a_lin_by + R13 * a_lin_bz;
    const raw_a_n_y = R21 * a_lin_bx + R22 * a_lin_by + R23 * a_lin_bz;
    const raw_a_n_z = R31 * a_lin_bx + R32 * a_lin_by + R33 * a_lin_bz;

    // Specific force for EKF F-matrix tracking (includes gravity and true un-clamped dynamics)
    this.lastSpecificForce = {
      x: raw_a_n_x,
      y: raw_a_n_y,
      z: raw_a_n_z + (accelMag > 3.0 ? this.gravity : 0)
    };

    let a_n_x = raw_a_n_x;
    let a_n_y = raw_a_n_y;
    let a_n_z = raw_a_n_z;

    // 4. Deadband Thresholding:
    // If linear acceleration magnitude on the horizontal plane is sub-threshold (< 0.08 m/s²),
    // clamp to exactly 0.0 to prevent noise integration during still or sub-threshold periods.
    const horizLinearMag = Math.sqrt(a_n_x * a_n_x + a_n_y * a_n_y);
    if (horizLinearMag < 0.08) {
      a_n_x = 0;
      a_n_y = 0;
    }

    // 5. Integrate linear acceleration into velocity
    this.velocity.x += a_n_x * dt;
    this.velocity.y += a_n_y * dt;
    this.velocity.z += a_n_z * dt;

    // 6. Integrate velocity into position
    this.position.x += this.velocity.x * dt;
    this.position.y += this.velocity.y * dt;
    this.position.z += this.velocity.z * dt;
  }

  public getLastNavAccel(): { x: number; y: number; z: number } {
    return {
      x: this.lastSpecificForce ? this.lastSpecificForce.x : 0,
      y: this.lastSpecificForce ? this.lastSpecificForce.y : 0,
      z: this.lastSpecificForce ? this.lastSpecificForce.z - this.gravity : 0,
    };
  }
}
