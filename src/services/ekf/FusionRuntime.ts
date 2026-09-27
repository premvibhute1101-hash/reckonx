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
  private isAttitudeInitialized = false;
  private isInitialHeadingSet = false;

  // ZUPT hysteresis: require N consecutive non-stationary samples before releasing
  private wasZuptActiveLastCycle = false;
  private nonStationaryCount = 0;
  private readonly ZUPT_RELEASE_HYSTERESIS = 3; // require 3 consecutive non-stationary samples

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
    this.wasZuptActiveLastCycle = false;
    this.nonStationaryCount = 0;

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

    const isFirstFix = this.initialLat === null || this.initialLon === null;
    if (isFirstFix) {
      this.initialLat = gnss.latitude;
      this.initialLon = gnss.longitude;
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
    
    // Estimate velocity from GNSS speed/heading if available
    let gnssVel: number[] | null = null;
    if (gnss.speed != null && gnss.heading != null) {
      gnssVel = [0, 0, 0];
      const hdgRad = gnss.heading * (Math.PI / 180);
      gnssVel[0] = gnss.speed * Math.sin(hdgRad); // East
      gnssVel[1] = gnss.speed * Math.cos(hdgRad); // North
    }

    const effAccuracy = (gnss.accuracy !== null && gnss.accuracy !== undefined && !isNaN(gnss.accuracy) && gnss.accuracy > 0)
      ? gnss.accuracy
      : 15.0;

    const nowMs = gnss.timestamp || Date.now();
    const timeGapMs = this.prevGnssArrivalTimestamp > 0 ? (nowMs - this.prevGnssArrivalTimestamp) : 0;
    this.prevGnssArrivalTimestamp = nowMs;
    this.lastGnssAccuracy = effAccuracy;
    this.lastGnssTimestamp = nowMs;

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

    if (this.lastKnownGnssSpeed !== null && this.lastKnownGnssSpeed >= 0.4) {
      if (this.wasZuptActiveLastCycle) {
        this.ekf.notifyZuptReleased();
        this.wasZuptActiveLastCycle = false;
      }
      this.nonStationaryCount = this.ZUPT_RELEASE_HYSTERESIS + 1;
    }

    this.ekf.updateGnss(gnssPos, gnssVel, effAccuracy, this.imuWindow);

    const qualityState = this.ekf.getGnssState().getState();
    const action = qualityState === 'WEAK_LOST' ? 'DR_FALLBACK' : qualityState === 'DEGRADED' ? 'APPLIED_DOWNWEIGHTED' : 'APPLIED_FULL';
    console.log(`[GNSS Ingest] gap=${timeGapMs}ms, ts=${this.lastGnssTimestamp}, lat=${gnss.latitude.toFixed(6)}, lng=${gnss.longitude.toFixed(6)}, acc=${effAccuracy.toFixed(1)}m, quality=${qualityState}, action=${action}`);

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

      // Stationary Detection: true physical standstill or vehicle idling
      const isImuStationary = varA < 0.15 && varG < 0.035;

      const gnssQuality = this.ekf.getGnssState().getState();
      const isGnssFresh = (now - this.lastKnownGnssSpeedTimestamp) < 2500 && (gnssQuality === 'GOOD' || gnssQuality === 'DEGRADED');

      if (isGnssFresh && this.lastKnownGnssSpeed !== null) {
        const gnssSpeedKmh = this.lastKnownGnssSpeed * 3.6;
        if (gnssSpeedKmh >= 1.5) {
          // 1. Moving at speed (cruising): suppress ZUPT completely, even if IMU is flat on smooth road
          isStationary = false;
        } else if (gnssSpeedKmh < 1.0) {
          // 2. GNSS confirms near-zero speed: allow ZUPT for true standstill OR engine vibration while idling
          // Vehicle engine vibration at idle typically produces varA in 0.15..1.2 and varG in 0.035..0.15
          const isVibratingStationary = varA < 1.2 && varG < 0.15;
          isStationary = isImuStationary || isVibratingStationary;
        } else {
          // 3. Ambiguous low GNSS speed (1.0 - 1.5 km/h): rely on strict IMU stillness
          isStationary = isImuStationary;
        }
      } else {
        // GNSS unavailable / stale / WEAK_LOST (e.g. tunnel outage):
        // Only trigger ZUPT if IMU is still AND current velocity has already integrated near zero (< 0.5 m/s)
        // If current velocity is non-zero, allow INS mechanization to continue dead-reckoning until braking deceleration brings velocity to 0.
        const currentInsVel = this.ekf.getIns().velocity;
        const currentInsSpeed = Math.sqrt(currentInsVel.x * currentInsVel.x + currentInsVel.y * currentInsVel.y);
        if (currentInsSpeed > 0.5) {
          isStationary = false;
        } else {
          isStationary = isImuStationary;
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
          // Suppress damping if GNSS confirms real forward motion (isGnssFresh && gnssSpeed >= 1.5 km/h)
          // or if vehicle/user is already moving at speed (currentInsSpeed >= 0.8 m/s)
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
      timestamp: Date.now(),
      sourceMode,
      gnssState: state,
      pureInsLatitude: pureInsLat,
      pureInsLongitude: pureInsLon,
      accelBias: biases.accel,
      gyroBias: biases.gyro,
      isAligned: this.isAttitudeInitialized,
      stabilization: stabilizationInfo
    };

    this.latestFusedState = fusedState;
    if (this.onFusedDataCallback) {
      this.onFusedDataCallback(fusedState);
    }
  }
}

export const fusionRuntime = new FusionRuntime();
