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
    // Standard gravity vector alignment:
    // Roll (rotation around body X-axis) = atan2(ay, az)
    // Pitch (rotation around body Y-axis) = atan2(-ax, sqrt(ay^2 + az^2))
    this.attitude.roll = Math.atan2(accel.y, accel.z);
    this.attitude.pitch = Math.atan2(-accel.x, Math.sqrt(accel.y * accel.y + accel.z * accel.z));
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

    // 2. Dynamic Gravity Leveling / Continuous Tilt Correction
    // When the phone is tilted (or stationary), the accelerometer measures the true gravity vector.
    // Driving estimated roll/pitch toward the accelerometer gravity vector eliminates gravity leakage
    // into horizontal velocity during small handheld tilts or static inclination.
    const accelMag = Math.sqrt(fAccel.x * fAccel.x + fAccel.y * fAccel.y + fAccel.z * fAccel.z);
    const gravityDiff = Math.abs(accelMag - this.gravity);

    if (gravityDiff < 2.5) {
      const rollAcc = Math.atan2(fAccel.y, fAccel.z);
      const pitchAcc = Math.atan2(-fAccel.x, Math.sqrt(fAccel.y * fAccel.y + fAccel.z * fAccel.z));

      // Dynamic gain: rapid convergence (~5.0 rad/s) when near 1g, gently tapering during high linear dynamic loads
      const gainWeight = Math.max(0, 1 - gravityDiff / 2.5);
      const alpha = Math.min(1.0, 5.0 * gainWeight * dt);

      newRoll = newRoll + alpha * (rollAcc - newRoll);
      newPitch = newPitch + alpha * (pitchAcc - newPitch);
    }

    this.attitude.roll = newRoll;
    this.attitude.pitch = newPitch;

    // 3. Transform body frame acceleration to navigation frame (ENU)
    const cRoll = Math.cos(this.attitude.roll);
    const sRoll = Math.sin(this.attitude.roll);
    const cPitch = Math.cos(this.attitude.pitch);
    const sPitch = Math.sin(this.attitude.pitch);
    const cYaw = Math.cos(this.attitude.yaw);
    const sYaw = Math.sin(this.attitude.yaw);

    // Standard Z-Y-X (Yaw-Pitch-Roll) Rotation Matrix R_b^n (Body to Nav ENU)
    const R11 = cYaw * cPitch;
    const R12 = cYaw * sPitch * sRoll - sYaw * cRoll;
    const R13 = cYaw * sPitch * cRoll + sYaw * sRoll;

    const R21 = sYaw * cPitch;
    const R22 = sYaw * sPitch * sRoll + cYaw * cRoll;
    const R23 = sYaw * sPitch * cRoll - cYaw * sRoll;

    const R31 = -sPitch;
    const R32 = cPitch * sRoll;
    const R33 = cPitch * cRoll;

    this.lastRotationMatrix = [
      [R11, R12, R13],
      [R21, R22, R23],
      [R31, R32, R33]
    ];

    let a_n_x = R11 * fAccel.x + R12 * fAccel.y + R13 * fAccel.z;
    let a_n_y = R21 * fAccel.x + R22 * fAccel.y + R23 * fAccel.z;
    let a_n_z = R31 * fAccel.x + R32 * fAccel.y + R33 * fAccel.z;

    this.lastSpecificForce = { x: a_n_x, y: a_n_y, z: a_n_z };

    // 4. Subtract gravity (assuming Z is up in ENU)
    const a_n_z_no_g = a_n_z - this.gravity;

    // 5. Horizontal acceleration deadband to eliminate sub-threshold micro-jitter (< 0.12 m/s²)
    const HORIZONTAL_ACCEL_DEADBAND = 0.12;
    if (Math.abs(a_n_x) < HORIZONTAL_ACCEL_DEADBAND) a_n_x = 0;
    if (Math.abs(a_n_y) < HORIZONTAL_ACCEL_DEADBAND) a_n_y = 0;

    // 6. Integrate acceleration into velocity
    this.velocity.x += a_n_x * dt;
    this.velocity.y += a_n_y * dt;
    this.velocity.z += a_n_z_no_g * dt;

    // 7. Integrate velocity into position
    this.position.x += this.velocity.x * dt;
    this.position.y += this.velocity.y * dt;
    this.position.z += this.velocity.z * dt;
  }
}
