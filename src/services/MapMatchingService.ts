/**
 * MapMatchingService.ts
 * =====================
 * Lightweight service proxy to `mapMatcher.worker.ts`.
 * Handles offline HMM map matching in a background Web Worker.
 * Follows the exact messageId correlation pattern of AIErrorCorrectionService.ts.
 */

import type { RecordedGPSPoint } from './api/trackingService';

export interface MatchSuccessResult {
  matched: true;
  points: { lat: number; lng: number }[];
}

export interface MatchFailureResult {
  matched: false;
  reason: string;
}

export type MapMatchResult = MatchSuccessResult | MatchFailureResult;

let worker: Worker | null = null;
let messageIdCounter = 0;
const pendingMatches = new Map<number, (res: MapMatchResult) => void>();

function initWorker() {
  if (worker || typeof window === 'undefined') return;

  try {
    worker = new Worker(new URL('../workers/mapMatcher.worker.ts', import.meta.url), {
      type: 'module',
    });

    worker.addEventListener('message', (e: MessageEvent) => {
      const { type, messageId, result } = e.data;
      if (type === 'MATCH_RESULT') {
        const resolve = pendingMatches.get(messageId);
        if (resolve) {
          resolve(result);
          pendingMatches.delete(messageId);
        }
      }
    });

    worker.addEventListener('error', (err) => {
      console.warn('[MapMatchingService] Worker error:', err);
    });
  } catch (err) {
    console.warn('[MapMatchingService] Worker initialization failed:', err);
  }
}

export const MapMatchingService = {
  /**
   * Request offline HMM map matching for a sequence of points in the background worker.
   */
  async matchSegment(points: RecordedGPSPoint[]): Promise<MapMatchResult> {
    if (!points || points.length === 0) {
      return { matched: false, reason: 'empty_points' };
    }

    initWorker();

    if (!worker) {
      return { matched: false, reason: 'worker_unavailable' };
    }

    const messageId = ++messageIdCounter;

    return new Promise<MapMatchResult>((resolve) => {
      pendingMatches.set(messageId, resolve);

      worker!.postMessage({
        type: 'MATCH_SEGMENT',
        payload: { points },
        messageId,
      });

      // 10s Timeout safety fallback
      setTimeout(() => {
        if (pendingMatches.has(messageId)) {
          pendingMatches.delete(messageId);
          resolve({ matched: false, reason: 'timeout' });
        }
      }, 10000);
    });
  },

  /**
   * Terminate worker instance if needed (e.g. during cleanup).
   */
  terminate() {
    if (worker) {
      worker.terminate();
      worker = null;
      pendingMatches.clear();
    }
  },
};
