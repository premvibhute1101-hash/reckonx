import { useSensorStore } from '../store/useSensorStore';
import { EkfCore } from './EkfCore';
import { InsMechanization } from './InsMechanization';
import { OutputStabilizer, StabilizerInput } from './OutputStabilizer';
import { MapMatcher } from '../mapmatch/MapMatcher';

export interface FusedState {
  latitude: number | null;
  longitude: number | null;
  velocity: { x: number; y: number; z: number };
  heading: number;
  timestamp: number;
  sourceMode: 'GNSS' | 'IDR';
  gnssState: string;
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
  mapSnapped?: {
    lat: number;
    lon: number;
    segmentId: string | null;
  };
}

export interface ImuSource {
  timestamp: number; // ms
  accel: { x: number; y: number; z: number };
  gyro: { x: number; y: number; z: number };
  mag?: { x: number; y: number; z: number };
}

class FusionRuntime {
  private ekf: EkfCore;
  private pureIns: InsMechanization;
  private outputStabilizer: OutputStabilizer;
  public mapMatcher: MapMatcher = new MapMatcher();
  public enableMapMatching: boolean = false;
  private lastMapMatchTimestamp: number = 0;

  private isRunning = false;
  private unsubscribe: (() => void) | null = null;

  private imuWindow: number[][] = [];
  private lastImuTimestamp: number = 0;
  private lastGnssTimestamp: number = 0;
  
  private initialLat: number | null = null;
  private initialLon: number | null = null;
  private isAttitudeInitialized = false;

  // Phase 5: Latency Tracking
  private lastValidGnssTs: number = 0;
  private hasReportedLossLatency = false;
  private gnssReturnTs: number = 0;
  private hasReportedReturnLatency = false;
  public latencyMetrics = {
    lossLatencyMs: 0,
    returnLatencyMs: 0
  };

  // ZUPT hysteresis: require N consecutive non-stationary samples before releasing
  private wasZuptActiveLastCycle = false;
  private nonStationaryCount = 0;
  private readonly ZUPT_RELEASE_HYSTERESIS = 3; // require 3 consecutive non-stationary samples

  // Earth radius in meters
  private readonly R_EARTH = 6378137;

  // Callback to update UI
  private onFusedDataCallbacks: Set<(state: FusedState) => void> = new Set();

  constructor() {
    this.ekf = new EkfCore();
    this.pureIns = new InsMechanization();
    this.outputStabilizer = new OutputStabilizer({ maxSpeedMps: 33.3, maxAccelMps2: 9.8 });
    // Pre-fill IMU window with zeros
    for (let i = 0; i < 20; i++) {
      this.imuWindow.push([0, 0, 9.81, 0, 0, 0]);
    }
  }

  public setOnFusedDataCallback(callback: (state: FusedState) => void) {
    this.onFusedDataCallbacks.add(callback);
  }

  public removeOnFusedDataCallback(callback: (state: FusedState) => void) {
    this.onFusedDataCallbacks.delete(callback);
  }

  public async start() {
    if (this.isRunning) return;
    this.isRunning = true;
    this.lastGnssTimestamp = Date.now();

    // Load AI Model safely
    try {
      await this.ekf.getAiModel().loadModel(require('../../assets/ins_error_model_fused.tflite'));
    } catch (e) {
      console.warn("Failed to load AI model, falling back to dummy measurements:", e);
    }

    // Subscribe to Zustore
    this.unsubscribe = useSensorStore.subscribe((state, prevState) => {
      // 1. Check for GNSS update (1Hz)
      if (state.gnss !== prevState.gnss && state.gnss.timestamp) {
        this.handleGnssUpdate(state);
      }

      // 2. Check for IMU update (triggered at 10Hz by Accel)
      if (state.accel !== prevState.accel) {
        this.handleImuUpdate(state);
      }
    });
  }

  public stop() {
    this.isRunning = false;
    if (this.unsubscribe) {
      this.unsubscribe();
      this.unsubscribe = null;
    }
  }

  private handleGnssUpdate(state: any) {
    const { gnss } = state;
    if (gnss.latitude === null || gnss.longitude === null) return;

    if (this.initialLat === null || this.initialLon === null) {
      this.initialLat = gnss.latitude;
      this.initialLon = gnss.longitude;
      // Also reset EKF and pure INS position to origin (already 0)
      this.pureIns.position = { x: 0, y: 0, z: 0 };
      this.pureIns.velocity = { x: 0, y: 0, z: 0 };
    }

    // Convert GNSS lat/lon to local ENU (simplified equirectangular projection)
    const latRad = this.initialLat! * (Math.PI / 180);
    const dx = (gnss.longitude - this.initialLon!) * (Math.PI / 180) * this.R_EARTH * Math.cos(latRad);
    const dy = (gnss.latitude - this.initialLat!) * (Math.PI / 180) * this.R_EARTH;

    // Assume Z is 0 for POC (altitude usually very noisy)
    const gnssPos = [dx, dy, 0];
    
    // Estimate velocity from GNSS speed/heading if available
    let gnssVel: number[] | null = null;
    if (gnss.speed != null && gnss.heading != null) {
      gnssVel = [0, 0, 0];
      const hdgRad = gnss.heading * (Math.PI / 180);
      gnssVel[0] = gnss.speed * Math.sin(hdgRad); // East
      gnssVel[1] = gnss.speed * Math.cos(hdgRad); // North
    }

    const nowTs = state.gnss.timestamp || Date.now();
    this.lastGnssTimestamp = nowTs;
    this.lastValidGnssTs = nowTs;
    
    // Reset loss latency flag on valid GNSS
    this.hasReportedLossLatency = false;
    if (this.gnssReturnTs === 0) {
      this.gnssReturnTs = nowTs; // First valid return
    }
    
    // Update EKF (which internally uses GnssQualityStateMachine)
    this.ekf.updateGnss(gnssPos, gnssVel, gnss.accuracy, this.imuWindow);
  }

  // Phase 5: Accept standard ImuSource interface
  public feedExternalImu(source: ImuSource) {
    this.handleImuUpdate(source);
  }

  private handleImuUpdate(state: any | ImuSource) {
    // Phase 5: Derive dt from IMU timestamps rather than system Date.now() if available
    const now = state.timestamp || Date.now();
    let dt = (now - this.lastImuTimestamp) / 1000.0;
    if (this.lastImuTimestamp === 0) {
      dt = 0.1; // Default on first run
    } else if (dt <= 0 || dt > 1.0) {
      console.warn(`[Guard] Invalid dt detected: ${dt}s. Skipping IMU update to prevent velocity runaway.`);
      this.lastImuTimestamp = now;
      return;
    }
    this.lastImuTimestamp = now;

    // Build sample vector
    const imuSample = [
      state.accel.x, state.accel.y, state.accel.z,
      state.gyro.x, state.gyro.y, state.gyro.z
    ];

    // Rolling window (keep exactly 20 samples)
    this.imuWindow.push(imuSample);
    while (this.imuWindow.length > 20) {
      this.imuWindow.shift();
    }

    // ZUPT / Stationary Detection
    let isStationary = false;
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

      let varA = 0, varG = 0;
      for (const s of this.imuWindow) {
        varA += Math.pow(s[0] - meanAx, 2) + Math.pow(s[1] - meanAy, 2) + Math.pow(s[2] - meanAz, 2);
        varG += Math.pow(s[3] - meanGx, 2) + Math.pow(s[4] - meanGy, 2) + Math.pow(s[5] - meanGz, 2);
      }
      varA /= n;
      varG /= n;

      isStationary = varA < 0.5 && varG < 0.05;

      // Initial Stationary Alignment Phase
      if (!this.isAttitudeInitialized) {
        if (isStationary) {
          // Initialize using the stable averaged gravity vector
          const initialAccel = { x: meanAx, y: meanAy, z: meanAz };
          this.ekf.getIns().initializeAttitude(initialAccel);
          this.pureIns.initializeAttitude(initialAccel);
          this.ekf.notifyAttitudeInitialized(); // Unlocks NHC — C_b_n is now valid
          this.isAttitudeInitialized = true;
          console.log("[FusionRuntime] Initial alignment complete. EKF started.");
        } else {
          // Device is moving. Do not initialize yet. Just emit unaligned state.
          this.emitFusedState();
          return;
        }
      }
    } else {
      // Buffer not full yet. Cannot initialize or run ZUPT.
      if (!this.isAttitudeInitialized) {
        this.emitFusedState();
        return;
      }
    }
    const posBefore = { ...this.ekf.getPosition() };
    const purePosBefore = { ...this.pureIns.position };

    // EKF Predict
    this.ekf.predict(dt, [imuSample[0], imuSample[1], imuSample[2]], [imuSample[3], imuSample[4], imuSample[5]]);

    // Pure INS Predict (for comparison, without any bias correction or updates)
    this.pureIns.predict(
      dt, 
      { x: imuSample[0], y: imuSample[1], z: imuSample[2] }, 
      { x: imuSample[3], y: imuSample[4], z: imuSample[5] }
    );

    if (this.imuWindow.length >= 20) {
      if (isStationary) {
        // Immediately apply ZUPT when stationary
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
        // Hysteresis: require N consecutive non-stationary samples before releasing ZUPT
        this.nonStationaryCount++;
        if (this.nonStationaryCount <= this.ZUPT_RELEASE_HYSTERESIS) {
          // Still within hysteresis window — keep applying ZUPT to prevent toggling
          this.ekf.updateZupt();

          // Hard-freeze position during hysteresis as well
          this.ekf.getPosition().x = posBefore.x;
          this.ekf.getPosition().y = posBefore.y;
          this.ekf.getPosition().z = posBefore.z;
          this.pureIns.position.x = purePosBefore.x;
          this.pureIns.position.y = purePosBefore.y;
          this.pureIns.position.z = purePosBefore.z;
        } else if (this.wasZuptActiveLastCycle) {
          // ZUPT release: notify EKF to inflate Q for graceful transition
          this.ekf.notifyZuptReleased();
          this.wasZuptActiveLastCycle = false;
        }
      }
    }

    // Stationary drift regression guard: warn if velocity drifts > 0.1 m/s at rest
    const vel = this.ekf.getIns().velocity;
    const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
    if (speed > 0.1) {
      // Only log occasionally to avoid spam (roughly once per second)
      if (Math.round(now / 1000) !== Math.round((now - dt * 1000) / 1000)) {
        console.warn(`[Drift Regression] |v|=${speed.toFixed(3)} m/s — potential stationary drift`);
      }
    }

    // Trigger AI / IDR mode if GNSS is lost for > 2 seconds
    if (now - this.lastGnssTimestamp > 2000) {
      this.ekf.updateGnss([0,0,0], [0,0,0], null, this.imuWindow);
      // Throttle IDR pseudo-updates to 1Hz
      this.lastGnssTimestamp = now - 1000;
      
      // Phase 5: Measure Loss Latency
      if (!this.hasReportedLossLatency && this.lastValidGnssTs > 0) {
        this.latencyMetrics.lossLatencyMs = now - this.lastValidGnssTs;
        // console.log(`[Latency] GNSS Loss to first AI-aided output: ${this.latencyMetrics.lossLatencyMs} ms`);
        this.hasReportedLossLatency = true;
        this.gnssReturnTs = 0; // Reset return timer
        this.hasReportedReturnLatency = false;
      }
    } else if (this.gnssReturnTs > 0 && !this.hasReportedReturnLatency) {
       // Phase 5: Measure Return Latency
       this.latencyMetrics.returnLatencyMs = now - this.gnssReturnTs;
       // console.log(`[Latency] GNSS Return to first fused output: ${this.latencyMetrics.returnLatencyMs} ms`);
       this.hasReportedReturnLatency = true;
    }

    // Phase 3: Map Matching Feedback
    // Apply soft constraint at 1Hz max when we are moving and map matching is enabled
    if (this.enableMapMatching && this.initialLat !== null && this.initialLon !== null && (now - this.lastMapMatchTimestamp >= 1000)) {
      const vel = this.ekf.getIns().velocity;
      const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
      if (speed > 1.0) { // Only snap if moving
        const pos = this.ekf.getPosition();
        const latRad = this.initialLat * (Math.PI / 180);
        const curLat = this.initialLat + (pos.y / this.R_EARTH) * (180 / Math.PI);
        const curLon = this.initialLon + (pos.x / (this.R_EARTH * Math.cos(latRad))) * (180 / Math.PI);
        
        let headingDeg = this.ekf.getIns().attitude.yaw * (180 / Math.PI);
        if (headingDeg < 0) headingDeg += 360;
        const headingRad = headingDeg * Math.PI / 180;
        
        const snapRes = this.mapMatcher.snap(curLat, curLon, headingRad, speed);
        
        if (snapRes.segmentId && snapRes.confidence > 0) {
          // Convert snapped lat/lon back to local ENU
          const dx = (snapRes.lon - this.initialLon) * (Math.PI / 180) * this.R_EARTH * Math.cos(latRad);
          const dy = (snapRes.lat - this.initialLat) * (Math.PI / 180) * this.R_EARTH;
          
          // Variance inversely proportional to confidence. 
          // confidence=1 -> R=5.0m^2; confidence=0.1 -> R=50.0m^2
          const R_pos = 5.0 / snapRes.confidence;
          this.ekf.updateMapMatch([dx, dy, pos.z], R_pos);
          
          this.lastMapMatchTimestamp = now;
        }
      }
    }

    this.emitFusedState();
  }

  private emitFusedState() {
    if (this.onFusedDataCallbacks.size === 0) return;

    const pos = this.ekf.getPosition();
    const vel = this.ekf.getIns().velocity;
    const att = this.ekf.getIns().attitude;

    // Reconstruct Lat/Lon from local ENU
    let fusedLat = null;
    let fusedLon = null;
    let pureInsLat = null;
    let pureInsLon = null;

    if (this.initialLat !== null && this.initialLon !== null) {
      const latRad = this.initialLat * (Math.PI / 180);
      fusedLat = this.initialLat + (pos.y / this.R_EARTH) * (180 / Math.PI);
      fusedLon = this.initialLon + (pos.x / (this.R_EARTH * Math.cos(latRad))) * (180 / Math.PI);

      pureInsLat = this.initialLat + (this.pureIns.position.y / this.R_EARTH) * (180 / Math.PI);
      pureInsLon = this.initialLon + (this.pureIns.position.x / (this.R_EARTH * Math.cos(latRad))) * (180 / Math.PI);
    } else {
      // Offline start with no GNSS anchor ever
      fusedLat = pos.y * 0.00001; // Fake scaling for UI visualization
      fusedLon = pos.x * 0.00001;

      pureInsLat = this.pureIns.position.y * 0.00001;
      pureInsLon = this.pureIns.position.x * 0.00001;
    }

    // Purely for visualization/metrics: what would the map matcher snap to?
    let mapSnapped = undefined;
    if (this.enableMapMatching && fusedLat !== null && fusedLon !== null) {
        let hRad = att.yaw; // already radians
        const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
        const snapRes = this.mapMatcher.snap(fusedLat, fusedLon, hRad, speed);
        if (snapRes.segmentId) {
            mapSnapped = {
                lat: snapRes.lat,
                lon: snapRes.lon,
                segmentId: snapRes.segmentId
            };
        }
    }

    const state = this.ekf.getGnssState().getState();
    const sourceMode = (state === 'GOOD' || state === 'DEGRADED') ? 'GNSS' : 'IDR';

    // Heading from Yaw (radians to degrees, adjusting relative to North)
    let headingDeg = att.yaw * (180 / Math.PI);
    if (headingDeg < 0) headingDeg += 360;

    let stabilizationInfo = undefined;
    if (this.initialLat !== null && fusedLat !== null && fusedLon !== null) {
      const stabilizerInput: StabilizerInput = {
        lat: fusedLat,
        lon: fusedLon,
        timestamp: this.lastImuTimestamp,
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

    const biases = this.ekf.getBiases ? this.ekf.getBiases() : { accel: {x:0,y:0,z:0}, gyro: {x:0,y:0,z:0} };

    const stateObj = {
      latitude: fusedLat,
      longitude: fusedLon,
      velocity: vel,
      heading: headingDeg,
      timestamp: this.lastImuTimestamp,
      sourceMode,
      gnssState: state,
      pureInsLatitude: pureInsLat,
      pureInsLongitude: pureInsLon,
      accelBias: biases.accel,
      gyroBias: biases.gyro,
      isAligned: this.isAttitudeInitialized,
      stabilization: stabilizationInfo,
      mapSnapped: mapSnapped
    };

    this.onFusedDataCallbacks.forEach(cb => cb(stateObj));
  }
}

export const fusionRuntime = new FusionRuntime();
