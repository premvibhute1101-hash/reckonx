import type { RecordedGPSPoint } from '../services/api/trackingService';
import type { AIModelStatus } from '../services/AIErrorCorrectionService';
import type { FusedState } from '../services/ekf/FusionRuntime';

/** Reflects whether AI dead-reckoning correction is correcting, falling back, or absent. */
export type AICorrectionStatus = 'active' | 'fallback' | 'unavailable';

/** Re-export for consumers that need it from the types module only. */
export type { AIModelStatus, FusedState };

export type OperationalMatrixScenario =
  | 'scenario1' // [GPS ON + Net ON] — Standard Online Navigation
  | 'scenario2' // [GPS OFF + Net ON] — GNSS Outage / Tunnel Mode
  | 'scenario3' // [GPS ON + Net OFF] — Pure Offline Satellite Mode
  | 'scenario4'; // [GPS OFF + Net OFF] — Pure Offline Dead Reckoning

export type SystemDataMode = 'live' | 'simulation';
export type LocationSource = 'gps' | 'dead_reckoning' | 'manual' | 'simulation';

export interface CurrentLocationData {
  latitude: number | null;
  longitude: number | null;
  accuracy: number | null; // meters
  altitude: number | null; // meters
  speed: number | null; // km/h
  bearing: number | null; // degrees 0-360
  timestamp: number | null;
  address: string;
  source: LocationSource;
  isStale: boolean;
  ageSec: number;
}

export interface RealSensorData {
  // 3-Axis Accelerometer (m/s²)
  ax: number;
  ay: number;
  az: number;
  accelMag: number;
  
  // 3-Axis Gyroscope (rad/s or deg/s)
  gx: number;
  gy: number;
  gz: number;
  gyroMag: number;

  // Orientation / Compass
  alpha: number | null; // Yaw (0-360)
  beta: number | null;  // Pitch (-180 to 180)
  gamma: number | null; // Roll (-90 to 90)
  headingDeg: number | null;

  // Magnetometer (μT) if hardware provides
  magX: number | null;
  magY: number | null;
  magZ: number | null;

  // Rates & Timestamps
  timestamp: number | null;
  intervalMs: number;
  sampleRateHz: number;
}

export interface SensorStatus {
  accel: boolean;
  gyro: boolean;
  compass: boolean;
  gnss: boolean;
  hasMotionHardware: boolean;
  hasOrientationHardware: boolean;
  sensorsEnabled: boolean;
  gpsPermission: 'prompt' | 'granted' | 'denied';
}

export type VehicleType = 'car' | 'bike' | 'walking';

export interface RouteStep {
  maneuverType: string;
  modifier?: string;
  name: string;
  distanceMeters: number;
  durationSec: number;
  instruction: string;
}

export interface RouteOption {
  id: string;
  index: number;
  coordinates: [number, number][];
  distanceKm: number;
  durationMin: number;
  label: 'Recommended' | 'Fastest' | 'Shortest' | 'Alternative';
  summary: string;
  routeType?: string;
  steps: RouteStep[];
}

export interface RoutingDebugInfo {
  vehicle: VehicleType;
  profile: string;
  provider: string;
  requestId: number;
  distanceKm: number;
  durationMin: number;
  source: string;
  timestamp: number;
}

export interface NormalizedRouteResult {
  vehicleProfile: VehicleType;
  distanceMeters: number;
  durationSeconds: number;
  distanceKm: number;
  durationMin: number;
  geometry: [number, number][];
  steps: RouteStep[];
  alternatives: RouteOption[];
  selectedRoute: RouteOption;
  routes: RouteOption[];
  summary: string;
  source: string;
  providerUrl: string;
  requestId: number;
  profileLabel: string;
  routeTypeLabel: string;
}

export interface RouteState {
  origin: string;
  destination: string;
  startCoords: [number, number] | null;
  destCoords: [number, number] | null;
  vehicleType: VehicleType;
  routes: RouteOption[];
  selectedRouteIndex: number;
  routeCoordinates: [number, number][];
  calculated: boolean;
  distance: string;
  duration: string;
  distanceKm: number;
  durationMin: number;
  tunnelLength: string;
  via?: string;
  routeType: string;
  profileLabel?: string;
  steps?: RouteStep[];
  isCalculating: boolean;
  isAcquiringLocation: boolean;
  error?: string | null;
  isFallbackRoute?: boolean;
  routingDebug?: RoutingDebugInfo;
}

export interface TelemetryData {
  speed: number;
  drift: number;
  eta: string;
  remainingKm: number;
  ax: number;
  ay: number;
  az: number;
  gx: number;
  gy: number;
  gz: number;
  pitch: number;
  roll: number;
  yaw: number;
  sampleRateHz: number;
  isStreamingMotion: boolean;
  isStreamingOrientation: boolean;
  lastEventTimestamp: number | null;
  /** AI-corrected INS velocity X (m/s ENU). Null when AI is in fallback mode. */
  aiCorrectedVelX: number | null;
  /** AI-corrected INS velocity Y (m/s ENU). Null when AI is in fallback mode. */
  aiCorrectedVelY: number | null;
  /** AI confidence proxy [0..1]. Null when AI is in fallback mode. */
  aiConfidence: number | null;
}

export interface SettingsState {
  highSpeedPolling: boolean;
  mapMatching: boolean;
  keepScreenAwake: boolean;
  autoCenterVehicle: boolean;
  speedUnit: 'km/h' | 'mph';
  distanceUnit: 'km' | 'mi';
  offlineLogs: string;
}

export interface UserProfile {
  name: string;
  role: string;
  id: string;
  email: string;
  isBackendConnected: boolean;
  avatar: string;
  stats?: {
    driven: string;
    tunnels: number;
    uptime: string;
  };
}

export interface SensorEventLogEntry {
  id: string;
  timestamp: number;
  type: 'devicemotion' | 'deviceorientation' | 'geolocation';
  summary: string;
  details: Record<string, any>;
}

export interface ActiveTrackingSession {
  isActive: boolean;
  sessionId: string;
  startTime: number | null;
  endTime: number | null;
  points: RecordedGPSPoint[];
  totalDistanceKm: number;
  maxSpeedKmH: number;
  averageSpeedKmH: number;
  gnssPointsCount: number;
  drPointsCount: number;
  gnssOutageDurationSec: number;
}

export interface ToastState {
  show: boolean;
  message: string;
}

export interface NavigationContextType {
  // Central Sources of Truth
  currentLocation: CurrentLocationData;
  realSensors: RealSensorData;
  systemMode: SystemDataMode;
  isSensorsEnabled: boolean;

  sensorStatus: SensorStatus;
  routeState: RouteState;
  telemetry: TelemetryData;
  settings: SettingsState;
  user: UserProfile;
  toast: ToastState;
  isLoggedIn: boolean;
  isOnline: boolean;
  matrixScenario: OperationalMatrixScenario;
  cachedTilesCount: number;
  sensorEventsStream: SensorEventLogEntry[];
  trackingSession: ActiveTrackingSession;
  /** Whether AI velocity correction is active, running in fallback, or unavailable. */
  aiCorrectionStatus: AICorrectionStatus;
  /** Real-time 15-state EKF sensor-fused state (lat, lon, velocity, heading, sourceMode, biases, stabilization). */
  fusedState: FusedState | null;
  /** Real-time status of backend API connection ('connected' | 'offline' | 'error'). */
  backendConnectionStatus: 'connected' | 'offline' | 'error';

  // Actions
  toggleSensors: (enabled?: boolean) => Promise<boolean>;
  setSystemMode: (mode: SystemDataMode) => void;
  refreshGpsLocation: () => Promise<CurrentLocationData | null>;
  setMatrixScenario: (scenario: OperationalMatrixScenario) => void;
  /** Update AI correction status from the DR ticker after each inference attempt. */
  setAiCorrectionStatus: (status: AICorrectionStatus) => void;

  calibrateCompass: () => Promise<void>;
  grantGnssPermission: () => Promise<void>;
  grantAllSensors: () => Promise<void>;
  acquireLiveLocation: () => Promise<void>;
  setStartCoordsAndAddress: (coords: [number, number], address: string, isManual?: boolean) => Promise<void>;
  setDestCoordsAndAddress: (coords: [number, number], address: string) => Promise<void>;
  setVehicleType: (type: VehicleType) => Promise<void>;
  setRouteDestination: (destination: string) => void;
  updateOriginDestination: (origin: string, destination: string) => void;
  calculateDynamicRoute: (
    start?: [number, number],
    dest?: [number, number],
    vehicleType?: VehicleType
  ) => Promise<void>;
  selectRoute: (index: number) => void;
  swapLocations: () => void;
  clearRoute: () => void;
  toggleSetting: (key: keyof SettingsState) => void;
  updateSettingValue: <K extends keyof SettingsState>(key: K, value: SettingsState[K]) => void;
  clearOfflineLogs: () => void;
  clearTileCache: () => Promise<void>;
  showToast: (msg: string) => void;
  loginUser: (email?: string) => void;
  logoutUser: () => void;
  startTrackingSession: () => void;
  stopTrackingSession: () => void;
  recordSessionPoint: (point: RecordedGPSPoint) => void;
  exportCurrentSessionLogs: () => Promise<{ success: boolean; message: string; filename?: string }>;
  resetSensorZeroPoint: () => void;
}
