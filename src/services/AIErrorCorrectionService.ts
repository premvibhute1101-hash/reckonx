/**
 * AIErrorCorrectionService.ts
 * ============================
 * Now a lightweight proxy to `aiCorrection.worker.ts` that handles TFJS inference
 * in a background thread to prevent UI freezing on desktop.
 */

const WINDOW_SIZE = 20;

export interface IMUSample {
  accX: number;
  accY: number;
  accZ: number;
  gyroYaw: number;
  gyroPitch: number;
  gyroRoll: number;
}

export interface INSState {
  velX: number;
  velY: number;
}

export interface AICorrectionResult {
  errVelX: number;
  errVelY: number;
  confidence: number;
  correctedVelX: number;
  correctedVelY: number;
  timestamp: number;
}

export type AIModelStatus = 'loading' | 'ready' | 'unavailable';

// ---------------------------------------------------------------------------
// Worker State
// ---------------------------------------------------------------------------
let worker: Worker | null = null;
let modelStatus: AIModelStatus = 'loading';
let loadError: string | null = null;
let inferenceCount = 0;
let bufferFill = 0;
let messageIdCounter = 0;

// Promise resolvers for pending correct() requests
const pendingCorrections = new Map<number, (res: AICorrectionResult | null) => void>();
let pendingLoadResolver: ((status: AIModelStatus) => void) | null = null;

function initWorker() {
  if (worker || typeof window === 'undefined') return;

  worker = new Worker(new URL('../workers/aiCorrection.worker.ts', import.meta.url), {
    type: 'module',
  });

  worker.addEventListener('message', (e) => {
    const { type, status, error, result, inferenceCount: count, bufferFill: fill, messageId } = e.data;
    
    if (count !== undefined) inferenceCount = count;
    if (fill !== undefined) bufferFill = fill;

    if (type === 'MODEL_LOADED') {
      modelStatus = status;
      loadError = error || null;
      if (pendingLoadResolver) {
        pendingLoadResolver(modelStatus);
        pendingLoadResolver = null;
      }
    } else if (type === 'STATUS_UPDATE') {
      modelStatus = status;
      loadError = error || null;
    } else if (type === 'CORRECTION_RESULT') {
      const resolve = pendingCorrections.get(messageId);
      if (resolve) {
        resolve(result || null);
        pendingCorrections.delete(messageId);
      }
    }
  });
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

async function loadModel(): Promise<AIModelStatus> {
  if (modelStatus === 'ready' || modelStatus === 'unavailable') {
    return modelStatus;
  }

  initWorker();
  
  if (!worker) {
    modelStatus = 'unavailable';
    loadError = 'Web Workers are not supported in this environment.';
    return modelStatus;
  }

  return new Promise<AIModelStatus>((resolve) => {
    pendingLoadResolver = resolve;
    worker!.postMessage({ type: 'LOAD_MODEL' });
  });
}

function pushSample(sample: IMUSample): void {
  initWorker();
  worker?.postMessage({ type: 'PUSH_SAMPLE', payload: sample });
}

function isBufferReady(): boolean {
  return bufferFill === WINDOW_SIZE;
}

function resetBuffer(): void {
  worker?.postMessage({ type: 'RESET_BUFFER' });
}

async function correct(insState: INSState): Promise<AICorrectionResult | null> {
  if (modelStatus !== 'ready' || !isBufferReady() || !worker) {
    return null;
  }

  return new Promise<AICorrectionResult | null>((resolve) => {
    const messageId = ++messageIdCounter;
    pendingCorrections.set(messageId, resolve);
    worker!.postMessage({ type: 'CORRECT', payload: insState, messageId });
  });
}

function getStatus(): AIModelStatus {
  return modelStatus;
}

function getLoadError(): string | null {
  return loadError;
}

function getInferenceCount(): number {
  return inferenceCount;
}

function getBufferFill(): number {
  return bufferFill;
}

export const AIErrorCorrectionService = {
  loadModel,
  pushSample,
  isBufferReady,
  resetBuffer,
  correct,
  getStatus,
  getLoadError,
  getInferenceCount,
  getBufferFill,
  WINDOW_SIZE,
} as const;
