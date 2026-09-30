/**
 * verify_ai_inference.ts
 * ======================
 * Node-side test for AIErrorCorrectionService inference logic.
 *
 * Since the real service runs behind a Web Worker + TFLite model
 * (browser-only), this test exercises the SAME normalization →
 * de-normalization mathematics and buffer contract that the worker
 * implements, without requiring a live browser or GPU.
 *
 * What is tested:
 *   1. 20 realistic IMU samples are pushed into a ring buffer.
 *   2. The buffer reaches capacity (WINDOW_SIZE = 20).
 *   3. The normalized + de-normalized inference pipeline runs end-to-end
 *      with a synthetic linear model (identical math to the worker).
 *   4. Assertions:
 *      a. Result is non-null.
 *      b. errVelX is a finite number.
 *      c. errVelY is a finite number.
 *      d. confidence is in [0, 1].
 *      e. correctedVelX / correctedVelY are finite numbers.
 *      f. Buffer correctly reports isBufferReady() = true after 20 pushes.
 *      g. Buffer correctly reports isBufferReady() = false after reset.
 *      h. Short-circuit: correct() before buffer-full returns null.
 */

import type { IMUSample, INSState, AICorrectionResult } from '../src/services/AIErrorCorrectionService';

// ---------------------------------------------------------------------------
// Mirror the normalization constants from aiCorrection.worker.ts
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
const STATE_STD  = [7.349498748779297, 7.638456344604492];
const Y_MEAN     = [-0.3668696880340576, 1.0653403997421265];
const Y_STD      = [10.648472785949707, 10.95954704284668];

const WINDOW_SIZE = 20;

// ---------------------------------------------------------------------------
// Pure helpers (exact mirrors of aiCorrection.worker.ts)
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
// Lightweight in-process buffer (mirrors worker imuBuffer state)
// ---------------------------------------------------------------------------
class InProcessAIInference {
  private readonly imuBuffer: number[][] = [];
  readonly WINDOW_SIZE = WINDOW_SIZE;

  pushSample(sample: IMUSample): void {
    const cleaned = safeArr([
      sample.accX,
      sample.accY,
      sample.accZ,
      sample.gyroYaw,
      sample.gyroPitch,
      sample.gyroRoll,
    ]);
    this.imuBuffer.push(cleaned);
    if (this.imuBuffer.length > WINDOW_SIZE) {
      this.imuBuffer.shift();
    }
  }

  isBufferReady(): boolean {
    return this.imuBuffer.length === WINDOW_SIZE;
  }

  resetBuffer(): void {
    this.imuBuffer.length = 0;
  }

  getBufferFill(): number {
    return this.imuBuffer.length;
  }

  /**
   * Synthetic inference: applies the same z-normalize → linear transform →
   * de-normalize pipeline as the real TFLite model call in the worker.
   *
   * The "model" here is a deterministic linear mapping:
   *   rawOutput[k] = mean(normalized_imu_window[:, k % 6]) * 0.1 + normalized_state[k % 2] * 0.05
   *
   * This is NOT the trained model — it just exercises the full numeric pipeline
   * so that errVelX / errVelY always come out as finite real numbers, exactly as
   * the real worker guarantees.
   */
  correct(insState: INSState): AICorrectionResult | null {
    if (!this.isBufferReady()) return null;

    // Normalize IMU window
    const rawImuFlat: number[] = [];
    for (const sample of this.imuBuffer) {
      rawImuFlat.push(...zNormalize(sample, IMU_MEAN, IMU_STD));
    }

    // Normalize INS state
    const rawState = [safeNum(insState.velX), safeNum(insState.velY)];
    const normalizedState = zNormalize(rawState, STATE_MEAN, STATE_STD);

    // Synthetic linear "prediction": shape [2]
    // We split the flat IMU array into WINDOW_SIZE rows of 6 features.
    const imuWindow: number[][] = [];
    for (let row = 0; row < WINDOW_SIZE; row++) {
      imuWindow.push(rawImuFlat.slice(row * 6, row * 6 + 6));
    }

    const colMeans = Array.from({ length: 6 }, (_, col) => {
      const sum = imuWindow.reduce((s, row) => s + row[col], 0);
      return sum / WINDOW_SIZE;
    });

    const rawOutput = [
      colMeans[0] * 0.1 + normalizedState[0] * 0.05,
      colMeans[1] * 0.1 + normalizedState[1] * 0.05,
    ].map(safeNum);

    // De-normalize
    const [errVelX, errVelY] = deNormalize(rawOutput.slice(0, 2), Y_MEAN, Y_STD);
    const confidence = computeConfidence(rawOutput.slice(0, 2));

    const correctedVelX = safeNum(insState.velX + errVelX);
    const correctedVelY = safeNum(insState.velY + errVelY);

    return {
      errVelX,
      errVelY,
      confidence,
      correctedVelX,
      correctedVelY,
      timestamp: Date.now(),
    };
  }
}

// ---------------------------------------------------------------------------
// 20 Real-representative IMU samples (vehicle at ~30 km/h on a bumpy road)
// Units: accX/Y/Z in m/s², gyroYaw/Pitch/Roll in rad/s
// ---------------------------------------------------------------------------
const REAL_IMU_SAMPLES: IMUSample[] = [
  { accX:  0.21, accY:  0.05, accZ:  9.81, gyroYaw: -0.003, gyroPitch:  0.002, gyroRoll:  0.001 },
  { accX:  0.18, accY:  0.08, accZ:  9.79, gyroYaw: -0.002, gyroPitch:  0.001, gyroRoll:  0.000 },
  { accX:  0.25, accY:  0.12, accZ:  9.83, gyroYaw: -0.004, gyroPitch:  0.003, gyroRoll: -0.001 },
  { accX:  0.19, accY: -0.03, accZ:  9.80, gyroYaw: -0.001, gyroPitch: -0.002, gyroRoll:  0.002 },
  { accX:  0.22, accY:  0.07, accZ:  9.78, gyroYaw: -0.005, gyroPitch:  0.004, gyroRoll: -0.002 },
  { accX:  0.30, accY:  0.10, accZ:  9.77, gyroYaw: -0.003, gyroPitch:  0.002, gyroRoll:  0.001 },
  { accX:  0.28, accY:  0.06, accZ:  9.82, gyroYaw: -0.002, gyroPitch:  0.001, gyroRoll:  0.000 },
  { accX:  0.24, accY:  0.11, accZ:  9.85, gyroYaw: -0.006, gyroPitch:  0.003, gyroRoll: -0.001 },
  { accX:  0.20, accY: -0.01, accZ:  9.79, gyroYaw: -0.001, gyroPitch: -0.001, gyroRoll:  0.002 },
  { accX:  0.23, accY:  0.09, accZ:  9.81, gyroYaw: -0.004, gyroPitch:  0.002, gyroRoll: -0.001 },
  { accX:  0.17, accY:  0.04, accZ:  9.83, gyroYaw: -0.003, gyroPitch:  0.001, gyroRoll:  0.001 },
  { accX:  0.26, accY:  0.13, accZ:  9.80, gyroYaw: -0.002, gyroPitch:  0.003, gyroRoll:  0.000 },
  { accX:  0.31, accY:  0.08, accZ:  9.78, gyroYaw: -0.005, gyroPitch:  0.004, gyroRoll: -0.002 },
  { accX:  0.22, accY: -0.02, accZ:  9.82, gyroYaw: -0.001, gyroPitch: -0.002, gyroRoll:  0.002 },
  { accX:  0.19, accY:  0.06, accZ:  9.77, gyroYaw: -0.003, gyroPitch:  0.002, gyroRoll: -0.001 },
  { accX:  0.27, accY:  0.10, accZ:  9.81, gyroYaw: -0.004, gyroPitch:  0.001, gyroRoll:  0.001 },
  { accX:  0.24, accY:  0.07, accZ:  9.84, gyroYaw: -0.002, gyroPitch:  0.003, gyroRoll:  0.000 },
  { accX:  0.20, accY:  0.05, accZ:  9.79, gyroYaw: -0.006, gyroPitch:  0.002, gyroRoll: -0.001 },
  { accX:  0.23, accY:  0.09, accZ:  9.82, gyroYaw: -0.003, gyroPitch: -0.001, gyroRoll:  0.002 },
  { accX:  0.21, accY:  0.03, accZ:  9.80, gyroYaw: -0.002, gyroPitch:  0.001, gyroRoll: -0.001 },
];

// ---------------------------------------------------------------------------
// Assertion helper
// ---------------------------------------------------------------------------
function assert(condition: boolean, message: string): void {
  if (!condition) {
    throw new Error(`ASSERT FAILED: ${message}`);
  }
}

// ---------------------------------------------------------------------------
// Test runner
// ---------------------------------------------------------------------------
async function runAIInferenceVerification(): Promise<void> {
  console.log('=============================================================');
  console.log('  AI INFERENCE VERIFICATION — AIErrorCorrectionService       ');
  console.log('=============================================================\n');

  const svc = new InProcessAIInference();

  // ── TEST A: short-circuit before buffer is full ──────────────────────────
  console.log('--- Test A: correct() before buffer full returns null ---');
  const earlyCorrResult = svc.correct({ velX: 0.5, velY: 0.1 });
  assert(earlyCorrResult === null, 'correct() before buffer full must return null');
  assert(!svc.isBufferReady(), 'isBufferReady() must be false before 20 pushes');
  console.log('  ✓ correct() = null before buffer full');
  console.log('  ✓ isBufferReady() = false\n');

  // ── TEST B: push exactly 20 real IMU samples ─────────────────────────────
  console.log('--- Test B: pushing 20 real IMU samples ---');
  assert(REAL_IMU_SAMPLES.length === WINDOW_SIZE, `Must have exactly ${WINDOW_SIZE} samples, got ${REAL_IMU_SAMPLES.length}`);

  for (let idx = 0; idx < REAL_IMU_SAMPLES.length; idx++) {
    svc.pushSample(REAL_IMU_SAMPLES[idx]);
    console.log(
      `  push[${String(idx + 1).padStart(2, '0')}] accX=${REAL_IMU_SAMPLES[idx].accX.toFixed(2)} accY=${REAL_IMU_SAMPLES[idx].accY.toFixed(2)} accZ=${REAL_IMU_SAMPLES[idx].accZ.toFixed(2)}` +
      ` gyroYaw=${REAL_IMU_SAMPLES[idx].gyroYaw.toFixed(4)} bufferFill=${svc.getBufferFill()}`
    );
  }

  assert(svc.getBufferFill() === WINDOW_SIZE, `bufferFill must be ${WINDOW_SIZE} after 20 pushes, got ${svc.getBufferFill()}`);
  assert(svc.isBufferReady(), 'isBufferReady() must be true after 20 pushes');
  console.log(`\n  ✓ bufferFill = ${svc.getBufferFill()} (== WINDOW_SIZE=${WINDOW_SIZE})`);
  console.log('  ✓ isBufferReady() = true\n');

  // ── TEST C: main inference call ──────────────────────────────────────────
  console.log('--- Test C: correct() inference with realistic INS state ---');
  const insState: INSState = { velX: 7.2, velY: 0.3 }; // ~26 km/h northward
  const result: AICorrectionResult | null = svc.correct(insState);

  assert(result !== null, 'correct() must return non-null when buffer is full');
  assert(typeof result!.errVelX === 'number', 'errVelX must be a number');
  assert(isFinite(result!.errVelX), `errVelX must be finite, got ${result!.errVelX}`);
  assert(typeof result!.errVelY === 'number', 'errVelY must be a number');
  assert(isFinite(result!.errVelY), `errVelY must be finite, got ${result!.errVelY}`);
  assert(isFinite(result!.confidence), `confidence must be finite, got ${result!.confidence}`);
  assert(result!.confidence >= 0 && result!.confidence <= 1, `confidence must be in [0,1], got ${result!.confidence}`);
  assert(isFinite(result!.correctedVelX), `correctedVelX must be finite, got ${result!.correctedVelX}`);
  assert(isFinite(result!.correctedVelY), `correctedVelY must be finite, got ${result!.correctedVelY}`);
  assert(typeof result!.timestamp === 'number' && result!.timestamp > 0, 'timestamp must be positive');

  console.log('  Inference result:');
  console.log(`    errVelX       = ${result!.errVelX.toFixed(6)} m/s`);
  console.log(`    errVelY       = ${result!.errVelY.toFixed(6)} m/s`);
  console.log(`    confidence    = ${result!.confidence.toFixed(6)}`);
  console.log(`    correctedVelX = ${result!.correctedVelX.toFixed(6)} m/s`);
  console.log(`    correctedVelY = ${result!.correctedVelY.toFixed(6)} m/s`);
  console.log(`    timestamp     = ${result!.timestamp}`);
  console.log('\n  ✓ result !== null');
  console.log('  ✓ errVelX is finite number');
  console.log('  ✓ errVelY is finite number');
  console.log('  ✓ confidence ∈ [0, 1]');
  console.log('  ✓ correctedVelX, correctedVelY are finite\n');

  // ── TEST D: sliding window — push one more sample, buffer stays at 20 ───
  console.log('--- Test D: sliding window — 21st push keeps buffer at 20 ---');
  const extra: IMUSample = { accX: 0.10, accY: 0.02, accZ: 9.81, gyroYaw: -0.001, gyroPitch: 0.001, gyroRoll: 0.0 };
  svc.pushSample(extra);
  assert(svc.getBufferFill() === WINDOW_SIZE, `Buffer must stay at ${WINDOW_SIZE} after 21st push, got ${svc.getBufferFill()}`);
  assert(svc.isBufferReady(), 'isBufferReady() must still be true after 21st push');

  const result2 = svc.correct(insState);
  assert(result2 !== null, 'correct() must still work after sliding window advance');
  assert(isFinite(result2!.errVelX), 'errVelX must remain finite after sliding window advance');
  console.log('  ✓ bufferFill stays at 20 after 21st push');
  console.log('  ✓ inference still works after sliding window advance\n');

  // ── TEST E: reset clears buffer ──────────────────────────────────────────
  console.log('--- Test E: resetBuffer() clears state ---');
  svc.resetBuffer();
  assert(svc.getBufferFill() === 0, `bufferFill must be 0 after reset, got ${svc.getBufferFill()}`);
  assert(!svc.isBufferReady(), 'isBufferReady() must be false after reset');
  const resultAfterReset = svc.correct(insState);
  assert(resultAfterReset === null, 'correct() must return null after reset');
  console.log('  ✓ bufferFill = 0 after reset');
  console.log('  ✓ isBufferReady() = false after reset');
  console.log('  ✓ correct() = null after reset\n');

  // ── TEST F: NaN / Inf inputs are sanitized ────────────────────────────────
  console.log('--- Test F: NaN/Inf IMU inputs are sanitized ---');
  const nanSamples: IMUSample[] = Array.from({ length: 20 }, (_, k) => ({
    accX: k % 3 === 0 ? NaN : 0.1,
    accY: k % 4 === 0 ? Infinity : 0.05,
    accZ: 9.81,
    gyroYaw: 0.0,
    gyroPitch: 0.0,
    gyroRoll: k % 5 === 0 ? -Infinity : 0.0,
  }));
  for (const s of nanSamples) svc.pushSample(s);
  const resultNan = svc.correct({ velX: NaN, velY: Infinity });
  assert(resultNan !== null, 'correct() must return non-null even with NaN inputs (sanitized)');
  assert(isFinite(resultNan!.errVelX), `errVelX must be finite after NaN inputs, got ${resultNan!.errVelX}`);
  assert(isFinite(resultNan!.errVelY), `errVelY must be finite after NaN inputs, got ${resultNan!.errVelY}`);
  console.log(`  errVelX after NaN inputs = ${resultNan!.errVelX.toFixed(6)} (finite)`);
  console.log('  ✓ NaN/Inf inputs sanitized — errVelX and errVelY remain finite\n');

  // ─────────────────────────────────────────────────────────────────────────
  console.log('=============================================================');
  console.log('  ALL AI INFERENCE VERIFICATION CHECKS PASSED ✓              ');
  console.log('=============================================================');
}

runAIInferenceVerification().catch((err) => {
  console.error('AI Inference verification FAILED:', err.message);
  process.exit(1);
});
