import { EkfCore } from './EkfCore';
import { InsMechanization } from './InsMechanization';
import { OutputStabilizer } from './OutputStabilizer';
import type { StabilizerInput } from './OutputStabilizer';
import type { GnssState } from './GnssQualityStateMachine';
import { MotionClassifier, type MotionActivity } from './MotionClassifier';
import { AnomalyDetector, type AnomalyReport } from './AnomalyDetector';
import { EARTH_RADIUS_METERS } from '../../constants/geodesy';

export interface FusedState {
  latitude: number | null;
  longitude: number | null;
  accuracy: number | null;
  velocity: { x: number; y: number; z: number };
  heading: number;
  timestamp: number;
  sourceMode: 'GNSS' | 'IDR';
  gnssState: GnssState;
  pureInsLatitude: number | null;
  pureInsLongitude: number | null;
  accelBias: { x: number; y: number; z: number };
  gyroBias: { x: number; y: number; z: number };
  isAligned: boolean;
  // Activity & Anomaly Telemetry
  motionActivity?: MotionActivity;
  anomalyReport?: AnomalyReport;
  // IMU & Motion Corroboration Diagnostic Telemetry
  imuVarA?: number;
  imuVarG?: number;
  gnssSpeedRawKmH?: number;
  gnssAccuracyRaw?: number | null;
  zuptState?: 'LOCKED' | 'RELEASED';
  gnssRejected?: boolean;
  gnssUncorroborated?: boolean;
  gnssAppliedThisTick?: boolean;
  lpHorizAccel?: number;
  stabilization?: {
    wasClamped: boolean;
    wasSmoothed: boolean;
    clampEvent?: any;
  };
}

export interface ImuSample {
  accel: { x: number; y: number; z: number };
  gyro: { x: number; y: number; z: number };
  timestamp?: number;
}

export interface GnssSample {
  latitude: number;
  longitude: number;
  accuracy?: number | null;
  speed?: number | null;
  heading?: number | null;
  hdop?: number | null;
  satellites?: number | null;
  timestamp?: number;
}

export class FusionRuntime {
  private ekf: EkfCore;
  private pureIns: InsMechanization;
  private outputStabilizer: OutputStabilizer;
  private motionClassifier: MotionClassifier;
  private anomalyDetector: AnomalyDetector;
  private lastAnomalyReport: AnomalyReport | undefined = undefined;
  public isRunning = false;

  private imuWindow: number[][] = [];
  private lastImuTimestamp: number = 0;
  private lastGnssTimestamp: number = 0;
  
  private initialLat: number | null = null;
  private initialLon: number | null = null;
  private hasReceivedGnssFix = false;
  private isAttitudeInitialized = false;
  private isInitialHeadingSet = false;

  // ZUPT hysteresis: require N consecutive non-stationary samples before releasing
  private wasZuptActiveLastCycle = false;
  private nonStationaryCount = 0;
  private readonly ZUPT_RELEASE_HYSTERESIS = 3; // require 3 consecutive non-stationary samples

  // IMU Motion Corroboration Gate
  private imuMotionCorroborationCount = 0;
  private readonly IMU_MOTION_CORROBORATION_THRESHOLD = 3; // require 3 samples of real IMU motion
  private sustainedUncorroboratedGnssCount = 0;
  private readonly SUSTAINED_GNSS_ESCAPE_THRESHOLD = 15; // 15 consecutive 1Hz GNSS fixes (~15s)
  private escapeInitialSpeed: number | null = null;
  private isGnssUncorroborated = false;
  private gnssAppliedThisTick = false;

  // IDR Rest Floor Rule & Stationary Variance Gates (varA < 0.025, varG < 0.01)
  private readonly REST_VAR_A_THRESHOLD = 0.025;
  private readonly REST_VAR_G_THRESHOLD = 0.01;
  private consecutiveRestFloorTicks = 0;
  private consecutiveGnssZeroTicks = 0;

  // Initial Rest Calibration (2 seconds / up to 200 samples)
  private isCalibrated = false;
  private calibrationSamples: { accel: { x: number; y: number; z: number }; gyro: { x: number; y: number; z: number } }[] = [];
  private calibrationStartTime = 0;
  private staticBiasA = { x: 0, y: 0, z: 0 };
  private staticBiasG = { x: 0, y: 0, z: 0 };

  // 1.5s Low-pass filtered horizontal acceleration for motion corroboration (rejects idle vibration)
  private horizAccelWindow: [number, number][] = [];
  private lastLpHorizAccel = 0;
  private imuIntegratedSpeedDuringWindow = 0;

  // GNSS speed band hysteresis with debounce (enter moving >= 1.5 km/h, exit < 1.0 km/h)
  private isGnssMovingBand: boolean = false;
  private gnssMovingDebounceCount: number = 0;
  private gnssStationaryDebounceCount: number = 0;
  private readonly GNSS_DEBOUNCE_THRESHOLD = 2; // 2 consecutive samples

  private lastImuVarA: number = 0;
  private lastImuVarG: number = 0;
  private lastGnssSpeedRawKmH: number = 0;
  private lastGnssAccuracyRaw: number | null = null;
  private lastGnssRejected: boolean = false;

  // Consolidated Earth radius in meters (WGS-84)
  private readonly R_EARTH = EARTH_RADIUS_METERS;

  // Callback to update subscribers
  private onFusedDataCallback: ((state: FusedState) => void) | null = null;
  private latestFusedState: FusedState | null = null;
  private lastGnssAccuracy: number | null = null;
  private prevGnssArrivalTimestamp: number = 0;
  private lastKnownGnssSpeed: number | null = null;
  private lastKnownGnssSpeedTimestamp: number = 0;
  private lastGnssRawPos: { x: number; y: number } | null = null;
  private lastGnssRawTime: number = 0;

  constructor() {
    this.ekf = new EkfCore();
    this.pureIns = new InsMechanization();
    this.outputStabilizer = new OutputStabilizer({ maxSpeedMps: 33.3, maxAccelMps2: 9.8 });
    this.motionClassifier = new MotionClassifier();
    this.anomalyDetector = new AnomalyDetector();
    // Pre-fill IMU window with gravity baseline
    for (let i = 0; i < 20; i++) {
      this.imuWindow.push([0, 0, 9.81, 0, 0, 0]);
    }
  }

  public setOnFusedDataCallback(callback: (state: FusedState) => void) {
    this.onFusedDataCallback = callback;
  }

  public getLatestState(): FusedState | null {
    return this.latestFusedState;
  }

  public getEkf(): EkfCore {
    return this.ekf;
  }

  /**
   * Injects an AI velocity error correction from aiCorrection.worker.ts into the
   * EKF's AiMotionModel. Called asynchronously from NavigationContext after each
   * AIErrorCorrectionService.correct() resolves.
   */
  public setExternalAiCorrection(errVelX: number, errVelY: number, confidence: number): void {
    this.ekf.getAiModel().setExternalCorrection(errVelX, errVelY, confidence);
  }

  public isImuMotionCorroborated(): boolean {
    return this.imuMotionCorroborationCount >= this.IMU_MOTION_CORROBORATION_THRESHOLD;
  }

  public async start() {
    if (this.isRunning) return;
    this.isRunning = true;
    this.lastGnssTimestamp = Date.now();

    // AI model initialization (stubbed for web runtime)
    try {
      await this.ekf.getAiModel().loadModel();
    } catch (e) {
      console.warn("Failed to load AI model, falling back to dummy measurements:", e);
    }
  }

  public stop() {
    this.isRunning = false;
  }

  public reset(originLat?: number, originLon?: number) {
    this.ekf = new EkfCore();
    this.pureIns = new InsMechanization();
    this.outputStabilizer = new OutputStabilizer({ maxSpeedMps: 33.3, maxAccelMps2: 9.8 });
    this.motionClassifier = new MotionClassifier();
    this.anomalyDetector = new AnomalyDetector();
    this.lastAnomalyReport = undefined;
    this.imuWindow = [];
    for (let i = 0; i < 20; i++) {
      this.imuWindow.push([0, 0, 9.81, 0, 0, 0]);
    }
    this.lastImuTimestamp = 0;
    this.lastGnssTimestamp = Date.now();
    this.lastGnssAccuracy = null;
    this.prevGnssArrivalTimestamp = 0;
    this.lastKnownGnssSpeed = null;
    this.lastKnownGnssSpeedTimestamp = 0;
    this.lastGnssRawPos = null;
    this.lastGnssRawTime = 0;
    this.isAttitudeInitialized = false;
    this.isInitialHeadingSet = false;
    this.hasReceivedGnssFix = false;
    this.wasZuptActiveLastCycle = false;
    this.nonStationaryCount = 0;
    this.imuMotionCorroborationCount = 0;
    this.sustainedUncorroboratedGnssCount = 0;
    this.escapeInitialSpeed = null;
    this.isGnssUncorroborated = false;
    this.gnssAppliedThisTick = false;
    this.consecutiveRestFloorTicks = 0;
    this.consecutiveGnssZeroTicks = 0;
    this.isCalibrated = false;
    this.calibrationSamples = [];
    this.calibrationStartTime = 0;
    this.staticBiasA = { x: 0, y: 0, z: 0 };
    this.staticBiasG = { x: 0, y: 0, z: 0 };
    this.horizAccelWindow = [];
    this.lastLpHorizAccel = 0;
    this.imuIntegratedSpeedDuringWindow = 0;
    this.isGnssMovingBand = false;
    this.gnssMovingDebounceCount = 0;
    this.gnssStationaryDebounceCount = 0;
    this.lastImuVarA = 0;
    this.lastImuVarG = 0;
    this.lastGnssSpeedRawKmH = 0;
    this.lastGnssAccuracyRaw = null;
    this.lastGnssRejected = false;

    if (originLat !== undefined && originLon !== undefined) {
      this.initialLat = originLat;
      this.initialLon = originLon;
    } else {
      this.initialLat = null;
      this.initialLon = null;
    }
    this.latestFusedState = null;
  }

  public isLastGnssRejected(): boolean {
    return this.lastGnssRejected;
  }

  public updateGnss(gnss: GnssSample) {
    if (gnss.latitude === null || gnss.latitude === undefined ||
        gnss.longitude === null || gnss.longitude === undefined) {
      return;
    }

    let gnssVel: number[] | null = null;
    if (gnss.speed != null && gnss.heading != null) {
      gnssVel = [0, 0, 0];
      const hdgRad = gnss.heading * (Math.PI / 180);
      gnssVel[0] = gnss.speed * Math.sin(hdgRad); // East
      gnssVel[1] = gnss.speed * Math.cos(hdgRad); // North
    }

    const isFirstFix = !this.hasReceivedGnssFix;
    if (isFirstFix) {
      this.hasReceivedGnssFix = true;
      if (this.initialLat === null || this.initialLon === null) {
        this.initialLat = gnss.latitude;
        this.initialLon = gnss.longitude;
      }
      this.pureIns.position = { x: 0, y: 0, z: 0 };
      this.pureIns.velocity = { x: 0, y: 0, z: 0 };
      this.outputStabilizer.reset();
      if (!this.isAttitudeInitialized) {
        this.ekf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });
        this.pureIns.initializeAttitude({ x: 0, y: 0, z: 9.81 });
        this.ekf.notifyAttitudeInitialized();
        this.isAttitudeInitialized = true;
      }
    }

    if (gnss.heading !== null && gnss.heading !== undefined) {
      const trackYaw = -(gnss.heading * (Math.PI / 180));
      if (!this.isInitialHeadingSet) {
        this.ekf.getIns().attitude.yaw = trackYaw;
        this.pureIns.attitude.yaw = trackYaw;
        this.isInitialHeadingSet = true;
      } else if (gnss.speed !== null && gnss.speed !== undefined && gnss.speed > 0.8) {
        // Periodic gyro yaw drift correction during reliable high-speed GNSS track
        // Gentle 10% complementary filter blending to correct gyro drift without jumping
        this.ekf.getIns().attitude.yaw = this.ekf.getIns().attitude.yaw * 0.9 + trackYaw * 0.1;
        this.pureIns.attitude.yaw = this.pureIns.attitude.yaw * 0.9 + trackYaw * 0.1;
      }
    }

    // Convert GNSS lat/lon to local ENU
    const latRad = this.initialLat! * (Math.PI / 180);
    const dx = (gnss.longitude - this.initialLon!) * (Math.PI / 180) * this.R_EARTH * Math.cos(latRad);
    const dy = (gnss.latitude - this.initialLat!) * (Math.PI / 180) * this.R_EARTH;

    const gnssPos = [dx, dy, 0];

    const effAccuracy = (gnss.accuracy !== null && gnss.accuracy !== undefined && !isNaN(gnss.accuracy) && gnss.accuracy > 0)
      ? gnss.accuracy
      : 15.0;

    const nowMs = gnss.timestamp || Date.now();
    const timeGapMs = this.prevGnssArrivalTimestamp > 0 ? (nowMs - this.prevGnssArrivalTimestamp) : 0;
    this.prevGnssArrivalTimestamp = nowMs;
    this.lastGnssAccuracy = effAccuracy;
    this.lastGnssTimestamp = nowMs;

    this.lastGnssSpeedRawKmH = (gnss.speed !== null && gnss.speed !== undefined && !isNaN(gnss.speed)) ? gnss.speed * 3.6 : 0;
    this.lastGnssAccuracyRaw = effAccuracy;

    // Track consecutive GNSS zero-speed updates for fallback ZUPT triggering
    if (gnss.speed !== null && gnss.speed !== undefined && !isNaN(gnss.speed)) {
      if (gnss.speed < 0.1) {
        this.consecutiveGnssZeroTicks++;
      } else if (gnss.speed >= 0.3) {
        this.consecutiveGnssZeroTicks = 0;
      }
    }

    // Track trusted GNSS speed for stationary cross-checking
    if (gnss.speed !== null && gnss.speed !== undefined && !isNaN(gnss.speed) && gnss.speed >= 0) {
      this.lastKnownGnssSpeed = gnss.speed;
      this.lastKnownGnssSpeedTimestamp = nowMs;
    } else if (this.lastGnssRawPos !== null && this.lastGnssRawTime > 0) {
      const dtPos = (nowMs - this.lastGnssRawTime) / 1000.0;
      if (dtPos >= 0.2 && dtPos <= 3.0) {
        const dEast = gnssPos[0] - this.lastGnssRawPos.x;
        const dNorth = gnssPos[1] - this.lastGnssRawPos.y;
        const disp = Math.sqrt(dEast * dEast + dNorth * dNorth);
        this.lastKnownGnssSpeed = disp / dtPos;
        this.lastKnownGnssSpeedTimestamp = nowMs;
      }
    }
    this.lastGnssRawPos = { x: gnssPos[0], y: gnssPos[1] };
    this.lastGnssRawTime = nowMs;

    const isImuCorroborated = this.isImuMotionCorroborated();
    const gnssSpeedMps = (gnss.speed !== null && gnss.speed !== undefined && !isNaN(gnss.speed))
      ? gnss.speed
      : (this.lastKnownGnssSpeed || 0);

    // Hardened Escape Rule (Item 2b):
    // 1. Accuracy must be high (effAccuracy <= 12.0m), rejecting 14-20m multipath drift.
    // 2. Uncorroborated GNSS speed cannot exceed IMU-integrated speed plus a small margin (2.5 m/s),
    //    OR GNSS is in steady cruising motion (speed variation <= 1.5 m/s) with valid accuracy.
    const speedDiffFromImu = gnssSpeedMps - this.imuIntegratedSpeedDuringWindow;
    const isSpeedConsistent = speedDiffFromImu <= 2.5; // margin ~ 9 km/h

    if (this.escapeInitialSpeed === null && gnssSpeedMps >= 0.416 && effAccuracy <= 12.0) {
      this.escapeInitialSpeed = gnssSpeedMps;
    }
    const isSteadyCruise = this.escapeInitialSpeed !== null && Math.abs(gnssSpeedMps - this.escapeInitialSpeed) <= 1.5;

    if (gnssSpeedMps >= 0.416 && effAccuracy <= 12.0 && (isSpeedConsistent || isSteadyCruise)) {
      if (!isImuCorroborated) {
        this.sustainedUncorroboratedGnssCount++;
      } else {
        this.sustainedUncorroboratedGnssCount = 0;
        this.escapeInitialSpeed = null;
      }
    } else {
      this.sustainedUncorroboratedGnssCount = 0;
      this.escapeInitialSpeed = null;
    }

    const isEscapeActive = this.sustainedUncorroboratedGnssCount >= this.SUSTAINED_GNSS_ESCAPE_THRESHOLD;
    this.isGnssUncorroborated = isEscapeActive && !isImuCorroborated;
    const allowMotionRelease = isImuCorroborated || isEscapeActive;

    const isGnssSpeedMoving = this.isGnssMovingBand || (this.lastKnownGnssSpeed !== null && this.lastKnownGnssSpeed >= 0.15);
    if (this.wasZuptActiveLastCycle && allowMotionRelease && isGnssSpeedMoving) {
      this.ekf.notifyZuptReleased();
      this.wasZuptActiveLastCycle = false;
      this.nonStationaryCount = this.ZUPT_RELEASE_HYSTERESIS + 1;
    }

    const gnssDist = this.lastKnownGnssSpeed !== null ? this.lastKnownGnssSpeed * (timeGapMs / 1000.0) : 0;
    const gnssAnomaly = this.anomalyDetector.checkGnssSample(
      gnssDist,
      timeGapMs / 1000.0,
      isImuCorroborated,
      Math.sqrt(dx * dx + dy * dy)
    );
    if (gnssAnomaly.severity !== 'NONE') {
      this.lastAnomalyReport = gnssAnomaly;
    }

    this.ekf.updateGnss(
      gnssPos,
      gnssVel,
      effAccuracy,
      this.imuWindow,
      isImuCorroborated,
      isEscapeActive,
      gnss.hdop,
      gnss.satellites
    );
    this.lastGnssRejected = this.ekf.isLastGnssRejected();
    this.gnssAppliedThisTick = !this.lastGnssRejected;

    this.emitFusedState();
  }

  public processImuSample(
    accel: { x: number; y: number; z: number },
    gyro: { x: number; y: number; z: number },
    timestamp?: number
  ) {
    const now = timestamp || Date.now();
    let dt = (now - this.lastImuTimestamp) / 1000.0;
    if (this.lastImuTimestamp === 0) {
      dt = 0.1; // Default 100ms on first run
    } else if (dt <= 0 || dt > 1.0) {
      this.lastImuTimestamp = now;
      return;
    }
    this.lastImuTimestamp = now;
    this.gnssAppliedThisTick = false;

    // Requirement 4: Initial Rest Calibration Phase (2 seconds / up to 200 samples)
    if (!this.isCalibrated) {
      if (this.calibrationStartTime === 0) {
        this.calibrationStartTime = now;
      }
      this.calibrationSamples.push({ accel: { ...accel }, gyro: { ...gyro } });

      const elapsedMs = now - this.calibrationStartTime;
      const count = this.calibrationSamples.length;
      const isMovingStartup = (this.lastKnownGnssSpeed !== null && this.lastKnownGnssSpeed > 1.0) ||
                              (Math.sqrt(accel.x * accel.x + accel.y * accel.y) > 2.0);

      if (count >= 200 || (elapsedMs >= 2000 && count >= 20) || (isMovingStartup && count >= 5)) {
        let sumAx = 0, sumAy = 0, sumAz = 0;
        let sumGx = 0, sumGy = 0, sumGz = 0;
        for (const s of this.calibrationSamples) {
          sumAx += s.accel.x; sumAy += s.accel.y; sumAz += s.accel.z;
          sumGx += s.gyro.x; sumGy += s.gyro.y; sumGz += s.gyro.z;
        }
        const meanAx = sumAx / count;
        const meanAy = sumAy / count;
        const meanAz = sumAz / count;
        const meanGx = sumGx / count;
        const meanGy = sumGy / count;
        const meanGz = sumGz / count;

        const meanMag = Math.sqrt(meanAx * meanAx + meanAy * meanAy + meanAz * meanAz);
        if (meanMag <= 3.0 && !isMovingStartup) {
          // Pure linear acceleration: average is exact zero bias
          this.staticBiasA = { x: meanAx, y: meanAy, z: meanAz };
        } else {
          this.staticBiasA = { x: 0, y: 0, z: 0 };
        }
        if (!isMovingStartup) {
          this.staticBiasG = { x: meanGx, y: meanGy, z: meanGz };
        }

        if (!this.isAttitudeInitialized) {
          const initAccel = { x: meanAx, y: meanAy, z: meanAz };
          this.ekf.getIns().initializeAttitude(initAccel);
          this.pureIns.initializeAttitude(initAccel);
          this.ekf.notifyAttitudeInitialized();
          this.isAttitudeInitialized = true;
        }
        this.isCalibrated = true;
      } else {
        // Calibration in progress: force zero velocity and output
        this.emitFusedState();
        return;
      }
    }

    // Subtract initial calibration baseline biases
    const imuAnomaly = this.anomalyDetector.checkImuSample(accel, now);
    if (imuAnomaly.severity !== 'NONE') {
      this.lastAnomalyReport = imuAnomaly;
    }

    const unbiasedAccel = {
      x: accel.x - this.staticBiasA.x,
      y: accel.y - this.staticBiasA.y,
      z: accel.z - this.staticBiasA.z,
    };
    const unbiasedGyro = {
      x: gyro.x - this.staticBiasG.x,
      y: gyro.y - this.staticBiasG.y,
      z: gyro.z - this.staticBiasG.z,
    };

    // Build sample vector [accelX, accelY, accelZ, gyroX, gyroY, gyroZ]
    const imuSample = [
      unbiasedAccel.x, unbiasedAccel.y, unbiasedAccel.z,
      unbiasedGyro.x, unbiasedGyro.y, unbiasedGyro.z
    ];

    // Rolling window (keep exactly 20 samples)
    this.imuWindow.push(imuSample);
    while (this.imuWindow.length > 20) {
      this.imuWindow.shift();
    }

    // 0. Low-pass filter horizontal acceleration / net specific-force (1.5 s window = 15 samples @ 10 Hz)
    const navA = this.pureIns.getLastNavAccel();
    this.horizAccelWindow.push([navA.x, navA.y]);
    if (this.horizAccelWindow.length > 15) {
      this.horizAccelWindow.shift();
    }

    let sumNx = 0, sumNy = 0;
    for (const h of this.horizAccelWindow) {
      sumNx += h[0];
      sumNy += h[1];
    }
    const lpAx = sumNx / this.horizAccelWindow.length;
    const lpAy = sumNy / this.horizAccelWindow.length;
    const lpHorizAccel = Math.sqrt(lpAx * lpAx + lpAy * lpAy);
    this.lastLpHorizAccel = lpHorizAccel;

    // ZUPT / Stationary Detection
    let isStationary = false;
    let isAtRestFloorLevel = false;
    let varA = 0;
    let varG = 0;
    if (this.imuWindow.length >= 20) {
      const n = this.imuWindow.length;
      let sumAx = 0, sumAy = 0, sumAz = 0;
      let sumGx = 0, sumGy = 0, sumGz = 0;
      for (const s of this.imuWindow) {
        sumAx += s[0]; sumAy += s[1]; sumAz += s[2];
        sumGx += s[3]; sumGy += s[4]; sumGz += s[5];
      }
      const meanAx = sumAx / n, meanAy = sumAy / n, meanAz = sumAz / n;
      const meanGx = sumGx / n, meanGy = sumGy / n, meanGz = sumGz / n;

      for (const s of this.imuWindow) {
        varA += Math.pow(s[0] - meanAx, 2) + Math.pow(s[1] - meanAy, 2) + Math.pow(s[2] - meanAz, 2);
        varG += Math.pow(s[3] - meanGx, 2) + Math.pow(s[4] - meanGy, 2) + Math.pow(s[5] - meanGz, 2);
      }
      varA /= n;
      varG /= n;
      this.lastImuVarA = varA;
      this.lastImuVarG = varG;

      // Track IMU motion corroboration evidence (Item 2c):
      // When at rest / smooth (varA < 0.10), threshold is 0.06 m/s^2 (reliably detects 0.10 m/s^2 soft start).
      // When engine idle vibration is present (varA in 0.15..0.65, varG up to 0.08),
      // vibration creates filter ripple up to ~0.18 m/s^2, so corroboration requires true acceleration >= 0.22 m/s^2.
      const isIdleVibrating = varA >= 0.15 || varG >= 0.035;
      const corroborationThreshold = isIdleVibrating ? 0.22 : 0.06;
      const isImuMotionEvident = isIdleVibrating ? (lpHorizAccel >= corroborationThreshold) : (lpHorizAccel >= corroborationThreshold || varA >= 0.035);
      const isImuCompletelyStill = lpHorizAccel < 0.05 && varA < 0.025 && varG < 0.02;

      if (isImuMotionEvident) {
        this.imuMotionCorroborationCount = Math.min(20, this.imuMotionCorroborationCount + 1);
        this.imuIntegratedSpeedDuringWindow = Math.min(60.0, this.imuIntegratedSpeedDuringWindow + lpHorizAccel * dt);
      } else if (isImuCompletelyStill || isIdleVibrating) {
        this.imuMotionCorroborationCount = Math.max(0, this.imuMotionCorroborationCount - 1);
        this.imuIntegratedSpeedDuringWindow = Math.max(0, this.imuIntegratedSpeedDuringWindow - 0.5 * dt);
      }

      // IDR Rest-Floor Counter (no horizontal acceleration AND low variance)
      isAtRestFloorLevel = lpHorizAccel < 0.05 && (
        (varA < this.REST_VAR_A_THRESHOLD && varG < this.REST_VAR_G_THRESHOLD) ||
        (varA < 0.05 && varG < 0.025)
      );
      if (isAtRestFloorLevel) {
        this.consecutiveRestFloorTicks++;
      } else {
        this.consecutiveRestFloorTicks = 0;
      }

      // Stationary Detection: true physical stillness or vehicle idling
      const isImuStationary = ((varA < this.REST_VAR_A_THRESHOLD && varG < this.REST_VAR_G_THRESHOLD) ||
                               (varA < 0.15 && varG < 0.035)) && lpHorizAccel < 0.05;

      const gnssQuality = this.ekf.getGnssState().getState();
      const isGnssFresh = (now - this.lastKnownGnssSpeedTimestamp) < 2500 && (gnssQuality === 'GOOD' || gnssQuality === 'DEGRADED');

      // Fallback check: If GNSS speed is 0.0 for > 2 seconds and variance is relatively stable (varA < 0.25, varG < 0.05)
      const isGnssZeroSpeedFallback = this.consecutiveGnssZeroTicks >= 2 && varA < 0.25 && varG < 0.05;

      // Hysteresis with debounce for GNSS speed bands (enter moving >= 1.5 km/h, exit < 1.0 km/h)
      if (isGnssFresh && this.lastKnownGnssSpeed !== null) {
        const gnssSpeedKmh = this.lastKnownGnssSpeed * 3.6;
        if (gnssSpeedKmh >= 1.5) {
          this.gnssStationaryDebounceCount = 0;
          this.gnssMovingDebounceCount++;
          if (this.gnssMovingDebounceCount >= this.GNSS_DEBOUNCE_THRESHOLD) {
            this.isGnssMovingBand = true;
          }
        } else if (gnssSpeedKmh < 1.0) {
          this.gnssMovingDebounceCount = 0;
          this.gnssStationaryDebounceCount++;
          if (this.gnssStationaryDebounceCount >= this.GNSS_DEBOUNCE_THRESHOLD) {
            this.isGnssMovingBand = false;
          }
        } else {
          // Hysteresis deadband [1.0, 1.5 km/h): hold state, reset debounce counters
          this.gnssMovingDebounceCount = 0;
          this.gnssStationaryDebounceCount = 0;
        }
      } else {
        this.gnssMovingDebounceCount = 0;
        this.gnssStationaryDebounceCount = 0;
      }

      const allowMotionRelease = this.isImuMotionCorroborated() || (this.sustainedUncorroboratedGnssCount >= this.SUSTAINED_GNSS_ESCAPE_THRESHOLD);

      // Physical launch hysteresis threshold: a_horiz >= 0.5 m/s^2 and varG < 0.05
      const isPhysicalLaunch = lpHorizAccel >= 0.5 && varG < 0.05;

      if (this.wasZuptActiveLastCycle) {
        // Strict ZUPT lock: hold until physical launch threshold or corroborated GNSS motion
        const isGnssIndicatingMotion = this.isGnssMovingBand || (isGnssFresh && this.lastKnownGnssSpeed !== null && this.lastKnownGnssSpeed >= 0.15);
        if ((isGnssIndicatingMotion && allowMotionRelease) || isPhysicalLaunch) {
          isStationary = false;
          if (isPhysicalLaunch) {
            this.imuMotionCorroborationCount = Math.max(this.imuMotionCorroborationCount, this.IMU_MOTION_CORROBORATION_THRESHOLD);
            this.nonStationaryCount = this.ZUPT_RELEASE_HYSTERESIS + 1;
            this.wasZuptActiveLastCycle = false;
            this.ekf.notifyZuptReleased();
          }
        } else {
          isStationary = true;
        }
      } else {
        // Active motion evaluation:
        if (isPhysicalLaunch) {
          this.imuMotionCorroborationCount = Math.max(this.imuMotionCorroborationCount, this.IMU_MOTION_CORROBORATION_THRESHOLD);
          isStationary = false;
        } else if (isGnssZeroSpeedFallback) {
          isStationary = true;
          this.imuMotionCorroborationCount = 0;
          this.imuIntegratedSpeedDuringWindow = 0;
        } else if (isGnssFresh && this.lastKnownGnssSpeed !== null) {
          const gnssSpeedKmh = this.lastKnownGnssSpeed * 3.6;
          if (this.isGnssMovingBand || gnssSpeedKmh >= 1.5) {
            isStationary = false;
          } else if ((gnssSpeedKmh >= 0.20 || this.isImuMotionCorroborated()) && (lpHorizAccel >= 0.05 || this.isImuMotionCorroborated() || varA >= 0.035)) {
            // Low-speed creep band with IMU motion evidence
            isStationary = false;
          } else if (gnssSpeedKmh < 1.0 && this.gnssStationaryDebounceCount >= this.GNSS_DEBOUNCE_THRESHOLD && !this.isImuMotionCorroborated()) {
            isStationary = true;
            this.imuMotionCorroborationCount = 0;
            this.imuIntegratedSpeedDuringWindow = 0;
          } else {
            isStationary = false;
          }
        } else {
          // GNSS unavailable / stale (IDR mode)
          const currentInsVel = this.ekf.getIns().velocity;
          const currentInsSpeed = Math.sqrt(currentInsVel.x * currentInsVel.x + currentInsVel.y * currentInsVel.y);

          if (currentInsSpeed <= 0.5) {
            // Low speed / at rest: engage ZUPT immediately when IMU is stationary
            isStationary = isImuStationary || isAtRestFloorLevel;
          } else {
            // High speed motion (e.g. tunnel outage): maintain velocity during outage,
            // but if device is physically resting (low variance), stop after 1.5s debounce.
            const isCompletelyResting = isAtRestFloorLevel || (lpHorizAccel < 0.08 && varA < 0.04 && varG < 0.015);
            if (isCompletelyResting) {
              this.consecutiveRestFloorTicks++;
            } else {
              this.consecutiveRestFloorTicks = 0;
            }

            if (this.consecutiveRestFloorTicks >= 15) {
              isStationary = true;
              this.ekf.getIns().velocity = { x: 0, y: 0, z: 0 };
              this.pureIns.velocity = { x: 0, y: 0, z: 0 };
            } else {
              isStationary = false;
            }
          }
        }
      }

      // Initial Stationary Alignment Phase
      if (!this.isAttitudeInitialized) {
        const initialAccel = { x: meanAx, y: meanAy, z: meanAz };
        this.ekf.getIns().initializeAttitude(initialAccel);
        this.pureIns.initializeAttitude(initialAccel);
        this.ekf.notifyAttitudeInitialized();
        this.isAttitudeInitialized = true;
      }
    } else {
      if (!this.isAttitudeInitialized) {
        this.emitFusedState();
        return;
      }
    }

    const posBefore = { ...this.ekf.getPosition() };
    const purePosBefore = { ...this.pureIns.position };

    // Pure INS Predict (tracks true un-clamped specific force for launch corroboration)
    this.pureIns.predict(
      dt, 
      { x: unbiasedAccel.x, y: unbiasedAccel.y, z: unbiasedAccel.z }, 
      { x: unbiasedGyro.x, y: unbiasedGyro.y, z: unbiasedGyro.z }
    );

    // Requirement 3: Hard State Overrides during ZUPT
    if (isStationary) {
      // Force input linear acceleration to [0, 0, 0] for EKF propagation to prevent velocity integration
      const isSpecificForce = Math.sqrt(unbiasedAccel.x * unbiasedAccel.x + unbiasedAccel.y * unbiasedAccel.y + unbiasedAccel.z * unbiasedAccel.z) > 3.0;
      const zeroAccel = [0, 0, isSpecificForce ? 9.81 : 0];

      this.ekf.predict(dt, zeroAccel, [0, 0, 0]);

      // Overwrite velocity state vector to exactly [0, 0, 0]
      this.ekf.getIns().velocity = { x: 0, y: 0, z: 0 };
      this.pureIns.velocity = { x: 0, y: 0, z: 0 };

      this.ekf.updateZupt();
      this.wasZuptActiveLastCycle = true;
      this.nonStationaryCount = 0;

      // Hard-freeze position to completely eliminate stationary drift
      this.ekf.getPosition().x = posBefore.x;
      this.ekf.getPosition().y = posBefore.y;
      this.ekf.getPosition().z = posBefore.z;
      this.pureIns.position.x = purePosBefore.x;
      this.pureIns.position.y = purePosBefore.y;
      this.pureIns.position.z = purePosBefore.z;
    } else {
      // Normal EKF Predict
      this.ekf.predict(dt, [imuSample[0], imuSample[1], imuSample[2]], [imuSample[3], imuSample[4], imuSample[5]]);

      if (this.imuWindow.length >= 20) {
        this.nonStationaryCount++;
        if (this.nonStationaryCount <= this.ZUPT_RELEASE_HYSTERESIS) {
          this.ekf.updateZupt();
          this.ekf.getPosition().x = posBefore.x;
          this.ekf.getPosition().y = posBefore.y;
          this.ekf.getPosition().z = posBefore.z;
          this.pureIns.position.x = purePosBefore.x;
          this.pureIns.position.y = purePosBefore.y;
          this.pureIns.position.z = purePosBefore.z;
        } else {
          if (this.wasZuptActiveLastCycle) {
            this.ekf.notifyZuptReleased();
            this.wasZuptActiveLastCycle = false;
          }

          // Gentle velocity damping during quasi-stationary periods
          const gnssQuality = this.ekf.getGnssState().getState();
          const isGnssFresh = (now - this.lastKnownGnssSpeedTimestamp) < 2500 && (gnssQuality === 'GOOD' || gnssQuality === 'DEGRADED');
          const isMovingWithGnss = isGnssFresh && this.lastKnownGnssSpeed !== null && (this.lastKnownGnssSpeed * 3.6 >= 1.5);
          const currentInsVel = this.ekf.getIns().velocity;
          const currentInsSpeed = Math.sqrt(currentInsVel.x * currentInsVel.x + currentInsVel.y * currentInsVel.y);
          if (!isMovingWithGnss && !this.isImuMotionCorroborated() && this.lastLpHorizAccel < 0.06 && currentInsSpeed < 0.8 && varA < 0.25 && varG < 0.06) {
            const dampFactor = Math.max(0, 1 - 0.5 * dt);
            this.ekf.getIns().velocity.x *= dampFactor;
            this.ekf.getIns().velocity.y *= dampFactor;
            this.ekf.getIns().velocity.z *= dampFactor;
          } else if (!isGnssFresh && isAtRestFloorLevel) {
            // Smooth decay during sustained IDR rest floor
            const decayRate = currentInsSpeed > 2.0 ? 1.5 : 0.6;
            const dampFactor = Math.max(0, 1 - decayRate * dt);
            this.ekf.getIns().velocity.x *= dampFactor;
            this.ekf.getIns().velocity.y *= dampFactor;
            this.ekf.getIns().velocity.z *= dampFactor;
          }
        }
      }
    }

    // Trigger AI / IDR fallback mode only if GNSS is truly lost for > 6.0 seconds
    if (now - this.lastGnssTimestamp > 6000) {
      this.ekf.updateGnss([0, 0, 0], [0, 0, 0], null, this.imuWindow);
      this.lastGnssTimestamp = now - 5000;
    }

    this.emitFusedState();
  }

  private emitFusedState() {
    const pos = this.ekf.getPosition();
    const vel = this.ekf.getIns().velocity;
    const att = this.ekf.getIns().attitude;

    let fusedLat: number | null = null;
    let fusedLon: number | null = null;
    let pureInsLat: number | null = null;
    let pureInsLon: number | null = null;

    if (this.initialLat !== null && this.initialLon !== null) {
      const latRad = this.initialLat * (Math.PI / 180);
      fusedLat = this.initialLat + (pos.y / this.R_EARTH) * (180 / Math.PI);
      fusedLon = this.initialLon + (pos.x / (this.R_EARTH * Math.cos(latRad))) * (180 / Math.PI);

      pureInsLat = this.initialLat + (this.pureIns.position.y / this.R_EARTH) * (180 / Math.PI);
      pureInsLon = this.initialLon + (this.pureIns.position.x / (this.R_EARTH * Math.cos(latRad))) * (180 / Math.PI);
    } else {
      fusedLat = null;
      fusedLon = null;
      pureInsLat = null;
      pureInsLon = null;
    }

    const state = this.ekf.getGnssState().getState();
    const now = this.lastImuTimestamp > 0 ? this.lastImuTimestamp : (this.lastGnssTimestamp > 0 ? this.lastGnssTimestamp : Date.now());
    const isGnssFresh = (now - this.lastGnssTimestamp) < 6000;
    const isGnssUsable = isGnssFresh && !this.lastGnssRejected && (state === 'GOOD' || state === 'DEGRADED');
    const sourceMode: 'GNSS' | 'IDR' = isGnssUsable ? 'GNSS' : 'IDR';

    // Compass heading (clockwise 0°..360° from North)
    let headingDeg = (-att.yaw * (180 / Math.PI)) % 360;
    if (headingDeg < 0) headingDeg += 360;
    headingDeg = Math.round(headingDeg * 10) / 10;

    let stabilizationInfo = undefined;
    if (this.initialLat !== null && fusedLat !== null && fusedLon !== null) {
      const stabilizerInput: StabilizerInput = {
        lat: fusedLat,
        lon: fusedLon,
        timestamp: now,
        gnssState: state
      };
      const stabilized = this.outputStabilizer.process(stabilizerInput);
      fusedLat = stabilized.lat;
      fusedLon = stabilized.lon;
      stabilizationInfo = {
        wasClamped: stabilized.wasClamped,
        wasSmoothed: stabilized.wasSmoothed,
        clampEvent: stabilized.clampEvent
      };
    }

    const biases = this.ekf.getBiases();

    const speedMag = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
    const isZuptActive = this.wasZuptActiveLastCycle || speedMag < 0.05 || !this.isCalibrated;
    const cleanVel = isZuptActive ? { x: 0, y: 0, z: 0 } : { x: vel.x, y: vel.y, z: vel.z };
    const cleanGnssSpeedKmH = isZuptActive ? 0.0 : this.lastGnssSpeedRawKmH;
    const effectiveSpeedKmh = Math.sqrt(cleanVel.x * cleanVel.x + cleanVel.y * cleanVel.y) * 3.6;

    const motionResult = this.motionClassifier.classify({
      varA: this.lastImuVarA,
      varG: this.lastImuVarG,
      lpHorizAccel: this.lastLpHorizAccel,
      speedKmh: effectiveSpeedKmh,
    });

    const fusedState: FusedState = {
      latitude: fusedLat,
      longitude: fusedLon,
      accuracy: this.lastGnssAccuracy,
      velocity: cleanVel,
      heading: headingDeg,
      timestamp: now,
      sourceMode,
      gnssState: state,
      pureInsLatitude: pureInsLat,
      pureInsLongitude: pureInsLon,
      accelBias: biases.accel,
      gyroBias: biases.gyro,
      isAligned: this.isAttitudeInitialized && this.isCalibrated,
      motionActivity: motionResult.activity,
      anomalyReport: this.lastAnomalyReport,
      imuVarA: this.lastImuVarA,
      imuVarG: this.lastImuVarG,
      gnssSpeedRawKmH: cleanGnssSpeedKmH,
      gnssAccuracyRaw: this.lastGnssAccuracyRaw,
      zuptState: this.wasZuptActiveLastCycle ? 'LOCKED' : 'RELEASED',
      gnssRejected: this.lastGnssRejected,
      gnssUncorroborated: this.isGnssUncorroborated,
      gnssAppliedThisTick: this.gnssAppliedThisTick,
      lpHorizAccel: this.lastLpHorizAccel,
      stabilization: stabilizationInfo
    };

    this.latestFusedState = fusedState;
    if (this.onFusedDataCallback) {
      this.onFusedDataCallback(fusedState);
    }
  }
}

export const fusionRuntime = new FusionRuntime();
