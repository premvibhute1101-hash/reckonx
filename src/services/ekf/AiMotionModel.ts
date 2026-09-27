import type { GnssState } from './GnssQualityStateMachine';

/**
 * AI Motion Error Correction Model.
 * TODO: Port to TensorFlow.js (TF.js) web model loading and inference for browser runtime.
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

  public async loadModel(_path?: string): Promise<void> {
    // TODO: Implement TF.js web model loading here
    this.isModelLoaded = false;
  }

  /**
   * Predicts velocity error correction if in ACTIVE mode.
   * @param gnssState Current GNSS quality state
   * @param _imuWindow Recent IMU samples [20, 6]
   * @param _insState Current INS state [2] (e.g. vel x, vel y)
   * @returns [error_x, error_y] or null if SLEEP mode / stubbed
   */
  public predictError(gnssState: GnssState, _imuWindow: number[][], _insState: number[]): number[] | null {
    // 1. Check if SLEEP mode
    if (gnssState === 'GOOD') {
      return null;
    }

    // 2. ACTIVE mode (WEAK_LOST or DEGRADED)
    // TODO: In the future, run TF.js inference when web model is integrated.
    // Return null / stubbed correction for now.
    return null;
  }
}
