import { GnssState } from './GnssQualityStateMachine';

// Using require to avoid breaking jest tests (since react-native-fast-tflite uses native modules)
let loadTensorflowModel: any = null;
try {
  const tflite = require('react-native-fast-tflite');
  loadTensorflowModel = tflite.loadTensorflowModel;
} catch (e) {
  // Ignored for test environments
}

export class AiMotionModel {
  private model: any = null;
  private isModelLoaded = false;
  
  // Normalization stats extracted from normalization_stats.npz
  private readonly IMU_MEAN = [3.7883697e-03, -3.5776526e-02, 9.8567991e+00, -1.7793525e-04, -5.1023387e-03, 3.7530807e-04];
  private readonly IMU_STD = [1.6299425, 1.4949728, 0.7497654, 0.11333029, 0.22307415, 0.13506341];
  private readonly STATE_MEAN = [-0.20375685, 0.56889135];
  private readonly STATE_STD = [7.3494987, 7.6384563];
  private readonly Y_MEAN = [-0.3668697, 1.0653404]; 
  private readonly Y_STD = [10.648473, 10.959547];

  public async loadModel(path: string) {
    if (loadTensorflowModel) {
      // Must pass [] as second argument to avoid C++ crash in react-native-fast-tflite NitroModule
      this.model = await loadTensorflowModel(path, []);
      this.isModelLoaded = true;
    }
  }

  /**
   * Predicts velocity error correction if in ACTIVE mode.
   * @param gnssState Current GNSS quality state
   * @param imuWindow Recent IMU samples [20, 6]
   * @param insState Current INS state [2] (e.g. vel x, vel y)
   * @returns [error_x, error_y] or null if SLEEP mode
   */
  public predictError(gnssState: GnssState, imuWindow: number[][], insState: number[]): number[] | null {
    // 1. Check if SLEEP mode
    if (gnssState === 'GOOD') {
      return null; // 0% CPU/GPU used
    }

    // 2. ACTIVE mode (WEAK_LOST or DEGRADED)
    if (this.isModelLoaded && this.model) {
      // Normalize IMU window [20, 6]
      const normalizedImu: number[] = [];
      for (const sample of imuWindow) {
        for (let i = 0; i < 6; i++) {
          normalizedImu.push((sample[i] - this.IMU_MEAN[i]) / this.IMU_STD[i]);
        }
      }

      // Normalize INS state [2] (vel x, vel y)
      const normalizedState = [
        (insState[0] - this.STATE_MEAN[0]) / this.STATE_STD[0],
        (insState[1] - this.STATE_MEAN[1]) / this.STATE_STD[1]
      ];

      const flatImu = Float32Array.from(normalizedImu);
      const flatState = Float32Array.from(normalizedState);

      try {
        const output = this.model.run([flatImu.buffer, flatState.buffer]);
        const predErrorNorm = new Float32Array(output[0]);

        // Denormalize output (m/s)
        return [
          predErrorNorm[0] * this.Y_STD[0] + this.Y_MEAN[0], // Error X (East)
          predErrorNorm[1] * this.Y_STD[1] + this.Y_MEAN[1]  // Error Y (North)
        ];
      } catch (e) {
        console.warn('Inference failed', e);
        return [0, 0];
      }
    } else {
      // MOCKED RESPONSE FOR JEST TESTS / FALLBACK
      return [0, 0];
    }
  }
}
