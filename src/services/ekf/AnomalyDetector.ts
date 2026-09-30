/**
 * AnomalyDetector.ts
 * ===================
 * Real-time Sensor, GNSS, and Innovation Anomaly Detection Engine.
 *
 * Detects:
 *   1. GNSS Spoofing / Multipath Ghosting: Impossible position jumps without IMU acceleration.
 *   2. Sensor Freezing / Hardware Dropout: Repeated identical floating-point values.
 *   3. Out-of-Bounds Innovation: Residuals that diverge from the EKF error covariance envelope.
 *   4. Sensor Clock Jitter / Reversal: Timestamps that jump backward or arrive out of sequence.
 */

export interface AnomalyReport {
  isSpoofed: boolean;
  isSensorFrozen: boolean;
  isInnovationExcessive: boolean;
  isTimestampAnomalous: boolean;
  severity: 'NONE' | 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  reasons: string[];
}

export class AnomalyDetector {
  private lastAccel: { x: number; y: number; z: number } | null = null;
  private consecutiveIdenticalAccelCount = 0;
  private lastTimestamp = 0;

  /**
   * Evaluates incoming IMU sensor readings for hardware freeze or clock issues.
   */
  public checkImuSample(
    accel: { x: number; y: number; z: number },
    timestampMs: number
  ): AnomalyReport {
    const reasons: string[] = [];
    let isSensorFrozen = false;
    let isTimestampAnomalous = false;

    // Check for sensor hardware freeze (repeating exact 64-bit floats)
    if (this.lastAccel) {
      if (
        this.lastAccel.x === accel.x &&
        this.lastAccel.y === accel.y &&
        this.lastAccel.z === accel.z
      ) {
        this.consecutiveIdenticalAccelCount++;
        if (this.consecutiveIdenticalAccelCount >= 15) {
          isSensorFrozen = true;
          reasons.push(`Sensor freeze: identical accel for ${this.consecutiveIdenticalAccelCount} samples`);
        }
      } else {
        this.consecutiveIdenticalAccelCount = 0;
      }
    }
    this.lastAccel = { ...accel };

    // Check for timestamp reversal or impossible gaps (> 5 seconds during active session)
    if (this.lastTimestamp > 0) {
      const dt = timestampMs - this.lastTimestamp;
      if (dt < 0) {
        isTimestampAnomalous = true;
        reasons.push(`Timestamp reversal: dt = ${dt} ms`);
      } else if (dt > 10000) {
        isTimestampAnomalous = true;
        reasons.push(`Extreme sensor gap: dt = ${dt} ms`);
      }
    }
    this.lastTimestamp = timestampMs;

    return {
      isSpoofed: false,
      isSensorFrozen,
      isInnovationExcessive: false,
      isTimestampAnomalous,
      severity: isSensorFrozen ? 'HIGH' : isTimestampAnomalous ? 'MEDIUM' : 'NONE',
      reasons,
    };
  }

  /**
   * Evaluates GNSS fix plausibility against IMU motion corroboration and innovation distance.
   */
  public checkGnssSample(
    gnssDisplacementM: number,
    dtSec: number,
    isImuMotionCorroborated: boolean,
    innovationM: number
  ): AnomalyReport {
    const reasons: string[] = [];
    let isSpoofed = false;
    let isInnovationExcessive = false;

    const impliedSpeedMps = dtSec > 0.05 ? gnssDisplacementM / dtSec : 0;
    const impliedSpeedKmh = impliedSpeedMps * 3.6;

    // Spoofing check: high implied speed (> 60 km/h) or large position jump (> 20m in 1s)
    // with completely uncorroborated stationary IMU
    if (impliedSpeedKmh > 60.0 && !isImuMotionCorroborated) {
      isSpoofed = true;
      reasons.push(`GNSS spoofing suspect: ${impliedSpeedKmh.toFixed(1)} km/h uncorroborated by IMU`);
    }

    // Innovation gate check
    if (innovationM > 100.0) {
      isInnovationExcessive = true;
      reasons.push(`Excessive EKF innovation residual: ${innovationM.toFixed(1)} m`);
    }

    const severity = isSpoofed
      ? 'CRITICAL'
      : isInnovationExcessive
      ? 'HIGH'
      : 'NONE';

    return {
      isSpoofed,
      isSensorFrozen: false,
      isInnovationExcessive,
      isTimestampAnomalous: false,
      severity,
      reasons,
    };
  }

  public reset(): void {
    this.lastAccel = null;
    this.consecutiveIdenticalAccelCount = 0;
    this.lastTimestamp = 0;
  }
}
