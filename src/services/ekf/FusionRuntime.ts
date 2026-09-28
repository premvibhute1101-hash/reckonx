import { EkfCore } from './EkfCore';
import { InsMechanization } from './InsMechanization';
import { OutputStabilizer } from './OutputStabilizer';
import type { StabilizerInput } from './OutputStabilizer';
import type { GnssState } from './GnssQualityStateMachine';

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
  timestamp?: number;
}

export class FusionRuntime {
  private ekf: EkfCore;
  private pureIns: InsMechanization;
  private outputStabilizer: OutputStabilizer;
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

  // IDR Rest Floor Rule (Configurable: default varA < 0.01, varG < 0.001)
  private readonly REST_VAR_A_THRESHOLD = 0.01;
  private readonly REST_VAR_G_THRESHOLD = 0.001;
  private consecutiveRestFloorTicks = 0;

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

  // Earth radius in meters
  private readonly R_EARTH = 6378137;

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

    this.ekf.updateGnss(gnssPos, gnssVel, effAccuracy, this.imuWindow, isImuCorroborated, isEscapeActive);
    this.lastGnssRejected = this.ekf.isLastGnssRejected();
    this.gnssAppliedThisTick = !this.lastGnssRejected;

    const qualityState = this.ekf.getGnssState().getState();
    const action = qualityState === 'WEAK_LOST' ? 'DR_FALLBACK' : qualityState === 'DEGRADED' ? 'APPLIED_DOWNWEIGHTED' : 'APPLIED_FULL';
    console.log(`[GNSS Ingest] gap=${timeGapMs}ms, ts=${this.lastGnssTimestamp}, lat=${gnss.latitude.toFixed(6)}, lng=${gnss.longitude.toFixed(6)}, acc=${effAccuracy.toFixed(1)}m, quality=${qualityState}, action=${action}, rejected=${this.lastGnssRejected}`);

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

    // Build sample vector [accelX, accelY, accelZ, gyroX, gyroY, gyroZ]
    const imuSample = [
      accel.x, accel.y, accel.z,
      gyro.x, gyro.y, gyro.z
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
      const isImuMotionEvident = lpHorizAccel >= corroborationThreshold;
      const isImuCompletelyStill = lpHorizAccel < 0.04 && varA < 0.05 && varG < 0.02;

      if (isImuMotionEvident) {
        this.imuMotionCorroborationCount = Math.min(20, this.imuMotionCorroborationCount + 1);
        this.imuIntegratedSpeedDuringWindow = Math.min(60.0, this.imuIntegratedSpeedDuringWindow + lpHorizAccel * dt);
      } else if (isImuCompletelyStill || isIdleVibrating) {
        this.imuMotionCorroborationCount = Math.max(0, this.imuMotionCorroborationCount - 1);
        this.imuIntegratedSpeedDuringWindow = Math.max(0, this.imuIntegratedSpeedDuringWindow - 0.5 * dt);
      }

      // IDR Rest-Floor Counter (Item 2a: varA < 0.01, varG < 0.001)
      const isAtRestFloorLevel = varA < this.REST_VAR_A_THRESHOLD && varG < this.REST_VAR_G_THRESHOLD;
      if (isAtRestFloorLevel) {
        this.consecutiveRestFloorTicks++;
      } else {
        this.consecutiveRestFloorTicks = 0;
      }

      // Stationary Detection: true physical stillness or vehicle idling
      const isImuStationary = varA < 0.15 && varG < 0.035;

      const gnssQuality = this.ekf.getGnssState().getState();
      const isGnssFresh = (now - this.lastKnownGnssSpeedTimestamp) < 2500 && (gnssQuality === 'GOOD' || gnssQuality === 'DEGRADED');

      // Hysteresis with debounce for GNSS speed bands (Item 2d: enter moving at >= 1.5 km/h, exit at < 1.0 km/h)
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

      if (this.wasZuptActiveLastCycle) {
        // IMU-Corroboration Gate:
        // When locked in ZUPT:
        // Do NOT release ZUPT unless GNSS indicates motion (speed >= 0.15 m/s or moving band)
        // AND the motion is corroborated by IMU (or sustained escape)
        const isGnssIndicatingMotion = this.isGnssMovingBand || (isGnssFresh && this.lastKnownGnssSpeed !== null && this.lastKnownGnssSpeed >= 0.15);
        if (isGnssIndicatingMotion && allowMotionRelease) {
          isStationary = false;
        } else {
          isStationary = true;
        }
      } else {
        // When vehicle is in active motion:
        if (isGnssFresh && this.lastKnownGnssSpeed !== null) {
          const gnssSpeedKmh = this.lastKnownGnssSpeed * 3.6;
          if (this.isGnssMovingBand || gnssSpeedKmh >= 1.5) {
            isStationary = false;
          } else if (gnssSpeedKmh >= 0.35 && (lpHorizAccel >= 0.06 || this.isImuMotionCorroborated())) {
            // Low-speed creep band (0.35..1.0 km/h) with IMU motion evidence
            isStationary = false;
          } else if (gnssSpeedKmh < 1.0 && this.gnssStationaryDebounceCount >= this.GNSS_DEBOUNCE_THRESHOLD) {
            // Speed dropped to ~0 (< 1.0 km/h) and debounced: vehicle stopped at red light / parked
            isStationary = true;
            this.imuMotionCorroborationCount = 0;
            this.imuIntegratedSpeedDuringWindow = 0;
          } else {
            // In hysteresis deadband [1.0, 1.5 km/h) or awaiting stationary debounce: hold moving
            isStationary = false;
          }
        } else {
          // GNSS unavailable / stale (IDR mode)
          const currentInsVel = this.ekf.getIns().velocity;
          const currentInsSpeed = Math.sqrt(currentInsVel.x * currentInsVel.x + currentInsVel.y * currentInsVel.y);

          // IDR Rest-Floor Rule (Item 2a):
          // If IMU variance stays at measured rest floor (varA < 0.01, varG < 0.001) for >= 5s (50 ticks)
          // apply ZUPT even if INS speed > 5 m/s!
          if (this.consecutiveRestFloorTicks >= 50) {
            isStationary = true;
            this.ekf.getIns().velocity = { x: 0, y: 0, z: 0 };
            this.pureIns.velocity = { x: 0, y: 0, z: 0 };
          } else if (this.consecutiveRestFloorTicks >= 10 && currentInsSpeed > 0.5) {
            // Active velocity decay during rest floor onset
            const dampFactor = Math.max(0, 1 - 0.5 * dt);
            this.ekf.getIns().velocity.x *= dampFactor;
            this.ekf.getIns().velocity.y *= dampFactor;
            this.ekf.getIns().velocity.z *= dampFactor;
            isStationary = false;
          } else if (currentInsSpeed > 0.5) {
            isStationary = false;
          } else {
            isStationary = isImuStationary;
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

    // EKF Predict
    this.ekf.predict(dt, [imuSample[0], imuSample[1], imuSample[2]], [imuSample[3], imuSample[4], imuSample[5]]);

    // Pure INS Predict
    this.pureIns.predict(
      dt, 
      { x: imuSample[0], y: imuSample[1], z: imuSample[2] }, 
      { x: imuSample[3], y: imuSample[4], z: imuSample[5] }
    );

    if (this.imuWindow.length >= 20) {
      if (isStationary) {
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

          // Gentle velocity damping ONLY during quasi-stationary periods (e.g. handheld tilt at rest)
          const gnssQuality = this.ekf.getGnssState().getState();
          const isGnssFresh = (now - this.lastKnownGnssSpeedTimestamp) < 2500 && (gnssQuality === 'GOOD' || gnssQuality === 'DEGRADED');
          const isMovingWithGnss = isGnssFresh && this.lastKnownGnssSpeed !== null && (this.lastKnownGnssSpeed * 3.6 >= 1.5);
          const currentInsVel = this.ekf.getIns().velocity;
          const currentInsSpeed = Math.sqrt(currentInsVel.x * currentInsVel.x + currentInsVel.y * currentInsVel.y);
          if (!isMovingWithGnss && currentInsSpeed < 0.8 && varA < 0.25 && varG < 0.06) {
            const dampFactor = Math.max(0, 1 - 0.5 * dt);
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
    const sourceMode: 'GNSS' | 'IDR' = (isGnssFresh && (state === 'GOOD' || state === 'DEGRADED')) ? 'GNSS' : 'IDR';

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
    const cleanVel = (this.wasZuptActiveLastCycle || speedMag < 0.03) ? { x: 0, y: 0, z: 0 } : { x: vel.x, y: vel.y, z: vel.z };

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
      isAligned: this.isAttitudeInitialized,
      imuVarA: this.lastImuVarA,
      imuVarG: this.lastImuVarG,
      gnssSpeedRawKmH: this.lastGnssSpeedRawKmH,
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
