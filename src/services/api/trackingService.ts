import { API_CONFIG } from '../../config/apiConfig';
import { apiClient } from './apiClient';
import { AuthService } from './authService';

export interface RecordedGPSPoint {
  timestamp: number;
  lat: number;
  lng: number;
  accuracyMeters?: number;
  speedKmH?: number;
  headingDeg?: number;
  altitudeMeters?: number | null;
  isDeadReckoning?: boolean;
  /** Raw INS velocity X before AI correction (m/s ENU). Populated during DR epochs. */
  rawInsVelX?: number;
  /** Raw INS velocity Y before AI correction (m/s ENU). Populated during DR epochs. */
  rawInsVelY?: number;
  /** AI-corrected velocity X (m/s ENU). Null when AI is in fallback mode. */
  aiCorrectedVelX?: number | null;
  /** AI-corrected velocity Y (m/s ENU). Null when AI is in fallback mode. */
  aiCorrectedVelY?: number | null;
  /** AI confidence proxy [0..1]. Null when AI is in fallback mode. */
  aiConfidence?: number | null;
  /** GPS ground-truth lat at this point (same as lat for GNSS points). */
  gpsGroundTruthLat?: number;
  /** GPS ground-truth lng at this point (same as lng for GNSS points). */
  gpsGroundTruthLng?: number;
  /** Offline HMM map-matched latitude (batch post-processed). */
  matchedLat?: number | null;
  /** Offline HMM map-matched longitude (batch post-processed). */
  matchedLng?: number | null;
  // IMU & ZUPT motion corroboration diagnostic columns
  imuVarA?: number;
  imuVarG?: number;
  gnssSpeedRawKmH?: number;
  gnssAccuracyRaw?: number | null;
  zuptState?: 'LOCKED' | 'RELEASED';
  gnssRejected?: boolean;
  gnssUncorroborated?: boolean;
  // Raw geolocation callback diagnostics
  rawCbLatitude?: number | null;
  rawCbLongitude?: number | null;
  rawCbAccuracy?: number | null;
  rawCbSpeedMps?: number | null;
  rawCbHeading?: number | null;
  rawCbTimestamp?: number | null;
  gnssAppliedThisTick?: boolean;
}


export interface TrackingSessionPayload {
  sessionId: string;
  startTime: number;
  endTime?: number;
  originAddress: string;
  destAddress: string;
  startCoords: [number, number];
  destCoords: [number, number];
  points: RecordedGPSPoint[];
  totalDistanceKm: number;
  durationSeconds: number;
  gnssPointsCount: number;
  drPointsCount: number;
}

export const TrackingService = {
  /**
   * Start a tracking session on the backend
   */
  async startSession(origin: string, destination: string, start: [number, number], dest: [number, number]): Promise<{ sessionId: string }> {
    const session = AuthService.getLocalSession();
    return apiClient<{ sessionId: string }>(API_CONFIG.ENDPOINTS.TRACKING_START, {
      method: 'POST',
      headers: session?.token ? { Authorization: `Bearer ${session.token}` } : {},
      body: JSON.stringify({ origin, destination, start, dest, timestamp: Date.now() }),
    });
  },

  /**
   * Stream a batch of recorded GPS & DR points to the backend
   */
  async sendPointsBatch(sessionId: string, points: RecordedGPSPoint[]): Promise<{ received: number }> {
    const session = AuthService.getLocalSession();
    return apiClient<{ received: number }>(API_CONFIG.ENDPOINTS.TRACKING_POINTS, {
      method: 'POST',
      headers: session?.token ? { Authorization: `Bearer ${session.token}` } : {},
      body: JSON.stringify({ sessionId, points }),
    });
  },

  /**
   * Conclude tracking session on backend
   */
  async endSession(payload: TrackingSessionPayload): Promise<{ success: boolean; summaryId?: string }> {
    const session = AuthService.getLocalSession();
    return apiClient<{ success: boolean; summaryId?: string }>(API_CONFIG.ENDPOINTS.TRACKING_END, {
      method: 'POST',
      headers: session?.token ? { Authorization: `Bearer ${session.token}` } : {},
      body: JSON.stringify(payload),
    });
  },
};
