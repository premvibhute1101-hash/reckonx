/**
 * MotionClassifier.ts
 * ====================
 * Real-time Motion & Activity Classifier for Seamless GNSS-IDR Navigation.
 *
 * Distinguishes between:
 *   - 'STATIONARY'  (Vehicle at stoplight, phone resting on desk)
 *   - 'PEDESTRIAN'  (Walking, jogging with step cadence)
 *   - 'AUTOMOTIVE'  (Car, bus, truck with vehicle dynamics & smooth cruise)
 *   - 'CYCLING'     (Bicycle with rhythmic pedal frequency)
 */

export type MotionActivity = 'STATIONARY' | 'PEDESTRIAN' | 'AUTOMOTIVE' | 'CYCLING';

export interface MotionFeatures {
  varA: number;          // Acceleration total variance (m²/s⁴)
  varG: number;          // Gyroscope total variance (rad²/s²)
  lpHorizAccel: number;  // Low-pass filtered horizontal acceleration (m/s²)
  speedKmh: number;      // Current speed (from GNSS or EKF)
  dominantFreqHz?: number; // Step/cadence frequency if FFT/zero-crossing available
}

export interface MotionClassificationResult {
  activity: MotionActivity;
  confidence: number;    // 0.0 to 1.0
  features: MotionFeatures;
}

export class MotionClassifier {
  private history: MotionActivity[] = [];
  private readonly DEBOUNCE_WINDOW = 5;

  /**
   * Classifies motion activity from current IMU statistical features and speed.
   */
  public classify(features: MotionFeatures): MotionClassificationResult {
    const { varA, varG, lpHorizAccel, speedKmh } = features;
    let activity: MotionActivity = 'STATIONARY';
    let confidence = 0.8;

    // 1. Stationary Detection
    if (speedKmh < 0.8 && varA < 0.035 && varG < 0.02 && lpHorizAccel < 0.06) {
      activity = 'STATIONARY';
      confidence = 0.95;
    }
    // 2. Pedestrian Detection (speed < 9 km/h, stride variance pattern)
    else if (speedKmh <= 9.0 && (varA >= 0.035 && varA <= 0.8) && (varG >= 0.01 && varG <= 0.5)) {
      activity = 'PEDESTRIAN';
      confidence = speedKmh <= 6.0 ? 0.90 : 0.75;
    }
    // 3. Cycling Detection (speed 8 - 35 km/h, rhythmic gyro variance)
    else if (speedKmh > 8.0 && speedKmh <= 35.0 && varG > 0.04 && varA < 0.3) {
      activity = 'CYCLING';
      confidence = 0.70;
    }
    // 4. Automotive Detection (speed > 10 km/h or engine idle vibration)
    else if (speedKmh > 10.0 || (varA >= 0.15 && varA <= 0.65 && varG <= 0.08)) {
      activity = 'AUTOMOTIVE';
      confidence = speedKmh > 20.0 ? 0.95 : 0.80;
    }
    // Default fallback
    else if (speedKmh > 0.8) {
      activity = speedKmh > 15.0 ? 'AUTOMOTIVE' : 'PEDESTRIAN';
      confidence = 0.60;
    }

    // Debounce state transitions over rolling history window
    this.history.push(activity);
    if (this.history.length > this.DEBOUNCE_WINDOW) {
      this.history.shift();
    }

    // Majority vote
    const counts = new Map<MotionActivity, number>();
    for (const act of this.history) {
      counts.set(act, (counts.get(act) || 0) + 1);
    }
    let debouncedActivity = activity;
    let maxCount = 0;
    for (const [act, count] of counts.entries()) {
      if (count > maxCount) {
        maxCount = count;
        debouncedActivity = act;
      }
    }

    return {
      activity: debouncedActivity,
      confidence,
      features,
    };
  }

  public reset(): void {
    this.history = [];
  }
}
