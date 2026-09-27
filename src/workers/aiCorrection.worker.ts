import * as tf from '@tensorflow/tfjs';
import type { IMUSample, INSState, AICorrectionResult } from '../services/AIErrorCorrectionService';

// ---------------------------------------------------------------------------
// Normalization constants from normalization_stats.npz
// ---------------------------------------------------------------------------
const IMU_MEAN = [
  0.003788369707763195,
  -0.0357765257358551,
  9.856799125671387,
  -0.0001779352460289374,
  -0.0051023387350142,
  0.00037530806730501354,
];
const IMU_STD = [
  1.629942536354065,
  1.494972825050354,
  0.7497653961181641,
  0.11333028972148895,
  0.22307415306568146,
  0.13506340980529785,
];

const STATE_MEAN = [-0.20375685393810272, 0.5688913464546204];
const STATE_STD = [7.349498748779297, 7.638456344604492];

const Y_MEAN = [-0.3668696880340576, 1.0653403997421265];
const Y_STD = [10.648472785949707, 10.95954704284668];

const WINDOW_SIZE = 20;
const N_IMU_FEATURES = 6;
const N_STATE_FEATURES = 2;

// ---------------------------------------------------------------------------
// Safe numeric helpers
// ---------------------------------------------------------------------------
function safeNum(v: number): number {
  return isFinite(v) && !isNaN(v) ? v : 0;
}

function safeArr(arr: number[]): number[] {
  return arr.map(safeNum);
}

function zNormalize(values: number[], mean: number[], std: number[]): number[] {
  return values.map((v, i) => safeNum((v - mean[i]) / std[i]));
}

function deNormalize(values: number[], mean: number[], std: number[]): number[] {
  return values.map((v, i) => safeNum(v * std[i] + mean[i]));
}

function computeConfidence(rawOutput: number[]): number {
  const l2 = Math.sqrt(rawOutput.reduce((s, v) => s + v * v, 0));
  return Math.min(1.0, Math.max(0.0, l2 / 3.0));
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
let model: tf.LayersModel | null = null;
let modelStatus: 'loading' | 'ready' | 'unavailable' = 'loading';
let loadError: string | null = null;
let inferenceCount = 0;
let isProcessing = false;

const imuBuffer: number[][] = [];

// ---------------------------------------------------------------------------
// Logic
// ---------------------------------------------------------------------------
async function loadModel() {
  if (model !== null) {
    postMessage({ type: 'MODEL_LOADED', status: 'ready', inferenceCount, bufferFill: imuBuffer.length });
    return;
  }

  try {
    const tfliteModule = await import('@tensorflow/tfjs-tflite').catch(() => null);
    if (!tfliteModule) {
      throw new Error('@tensorflow/tfjs-tflite not installed.');
    }
    const tfliteModel = await (tfliteModule as any).loadTFLiteModel(
      '/deadreckon-idr/ins_error_model.tflite'
    );
    model = tfliteModel as unknown as tf.LayersModel;
    
    // Warm-up
    const dummyImu = tf.zeros([1, WINDOW_SIZE, N_IMU_FEATURES]);
    const dummyState = tf.zeros([1, N_STATE_FEATURES]);
    try {
      await tf.tidy(() => {
        (model as any).predict([dummyImu, dummyState]);
      });
    } catch {} finally {
      dummyImu.dispose();
      dummyState.dispose();
    }

    modelStatus = 'ready';
    postMessage({ type: 'MODEL_LOADED', status: 'ready', inferenceCount, bufferFill: imuBuffer.length });
  } catch (err: any) {
    modelStatus = 'unavailable';
    loadError = err?.message ?? String(err);
    postMessage({ type: 'MODEL_LOADED', status: 'unavailable', error: loadError, inferenceCount, bufferFill: imuBuffer.length });
  }
}

async function correct(insState: INSState, messageId: number) {
  if (modelStatus !== 'ready' || !model || imuBuffer.length !== WINDOW_SIZE || isProcessing) {
    postMessage({ type: 'CORRECTION_RESULT', messageId, result: null, inferenceCount, bufferFill: imuBuffer.length });
    return;
  }

  isProcessing = true;
  let imuTensor: tf.Tensor | null = null;
  let stateTensor: tf.Tensor | null = null;
  let outputTensor: tf.Tensor | null = null;

  try {
    const rawImuFlat: number[] = [];
    for (const sample of imuBuffer) {
      rawImuFlat.push(...zNormalize(sample, IMU_MEAN, IMU_STD));
    }

    const rawState = [safeNum(insState.velX), safeNum(insState.velY)];
    const normalizedState = zNormalize(rawState, STATE_MEAN, STATE_STD);

    imuTensor = tf.tensor3d([rawImuFlat.reduce<number[][]>((rows, _, i, arr) => {
      if (i % N_IMU_FEATURES === 0) rows.push(arr.slice(i, i + N_IMU_FEATURES));
      return rows;
    }, [])], [1, WINDOW_SIZE, N_IMU_FEATURES]);
    stateTensor = tf.tensor2d([normalizedState], [1, N_STATE_FEATURES]);

    let rawOutput: number[];
    outputTensor = tf.tidy(() => {
      return (model as any).predict([imuTensor, stateTensor]);
    });

    rawOutput = safeArr(Array.from(await (outputTensor as tf.Tensor).data()));

    const [errVelX, errVelY] = deNormalize(rawOutput.slice(0, 2), Y_MEAN, Y_STD);
    const confidence = computeConfidence(rawOutput.slice(0, 2));

    const correctedVelX = safeNum(insState.velX + errVelX);
    const correctedVelY = safeNum(insState.velY + errVelY);

    inferenceCount++;

    const result: AICorrectionResult = {
      errVelX,
      errVelY,
      confidence,
      correctedVelX,
      correctedVelY,
      timestamp: Date.now(),
    };

    postMessage({ type: 'CORRECTION_RESULT', messageId, result, inferenceCount, bufferFill: imuBuffer.length });
  } catch (err) {
    postMessage({ type: 'CORRECTION_RESULT', messageId, result: null, inferenceCount, bufferFill: imuBuffer.length });
  } finally {
    isProcessing = false;
    imuTensor?.dispose();
    stateTensor?.dispose();
    outputTensor?.dispose();
  }
}

// ---------------------------------------------------------------------------
// Message Handler
// ---------------------------------------------------------------------------
self.addEventListener('message', async (e) => {
  const { type, payload, messageId } = e.data;

  switch (type) {
    case 'LOAD_MODEL':
      await loadModel();
      break;

    case 'PUSH_SAMPLE':
      const sample = payload as IMUSample;
      const cleaned = safeArr([
        sample.accX,
        sample.accY,
        sample.accZ,
        sample.gyroYaw,
        sample.gyroPitch,
        sample.gyroRoll,
      ]);
      imuBuffer.push(cleaned);
      if (imuBuffer.length > WINDOW_SIZE) {
        imuBuffer.shift();
      }
      postMessage({ type: 'STATUS_UPDATE', status: modelStatus, error: loadError, inferenceCount, bufferFill: imuBuffer.length });
      break;

    case 'RESET_BUFFER':
      imuBuffer.length = 0;
      postMessage({ type: 'STATUS_UPDATE', status: modelStatus, error: loadError, inferenceCount, bufferFill: imuBuffer.length });
      break;

    case 'CORRECT':
      await correct(payload as INSState, messageId);
      break;

    case 'GET_STATUS':
      postMessage({ type: 'STATUS_UPDATE', status: modelStatus, error: loadError, inferenceCount, bufferFill: imuBuffer.length });
      break;
  }
});
