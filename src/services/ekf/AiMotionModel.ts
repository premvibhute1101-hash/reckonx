import type { GnssState } from './GnssQualityStateMachine';

/**
 * AI Motion Error Correction Model.
 * Inference runs in aiCorrection.worker.ts (TFLite).
 * Results are injected here by FusionRuntime via setExternalCorrection().
 */
export class AiMotionModel {
  public isModelLoaded: boolean = false;

  // Normalization stats extracted from normalization_stats.npz
  public readonly IMU_MEAN = [3.7883697e-03, -3.5776526e-02, 9.8567991e+00, -1.7793525e-04, -5.1023387e-03, 3.7530807e-04];
  public readonly IMU_STD = [1.6299425, 1.4949728, 0.7497654, 0.11333029, 0.22307415, 0.13506341];
  public readonly STATE_MEAN = [-0.20375685, 0.56889135];
  public readonly STATE_STD = [7.3494987, 7.6384563];
  public readonly Y_MEAN = [-0.3668697, 1.0653404];
  public readonly Y_STD = [10.648473, 10.959547];

  /** Latest correction [errVelX, errVelY] injected from aiCorrection.worker.ts */
  private externalCorrection: number[] | null = null;
  /** Confidence [0..1] from the worker, used to gate weak results */
  private externalConfidence: number = 0;

  /**
   * Called by FusionRuntime when AIErrorCorrectionService returns a result.
   */
  public setExternalCorrection(errVelX: number, errVelY: number, confidence: number): void {
    this.isModelLoaded = true;
    this.externalCorrection = [errVelX, errVelY];
    this.externalConfidence = confidence;
  }

  /** Clear correction (e.g. on session reset). */
  public clearCorrection(): void {
    this.externalCorrection = null;
    this.externalConfidence = 0;
    this.isModelLoaded = false;
  }

  public async loadModel(_path?: string): Promise<void> {
    // Model loads inside aiCorrection.worker.ts — nothing to do here.
  }

  /**
   * Returns the worker-computed velocity error correction.
   * Returns null in SLEEP mode (GOOD GNSS) or when no correction is available.
   */
  public predictError(gnssState: GnssState, _imuWindow: number[][], _insState: number[]): number[] | null {
    // SLEEP mode — GNSS is healthy, don't burn CPU
    if (gnssState === 'GOOD') {
      return null;
    }

    // ACTIVE mode: return latest worker result if confidence is sufficient
    if (this.externalCorrection !== null && this.externalConfidence >= 0.05) {
      return this.externalCorrection;
    }

    return null;
  }
}
