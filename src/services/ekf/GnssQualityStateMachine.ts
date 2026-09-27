export type GnssState = 'GOOD' | 'DEGRADED' | 'WEAK_LOST';

export class GnssQualityStateMachine {
  public currentState: GnssState = 'WEAK_LOST';
  
  /**
   * Updates the state machine with the latest GNSS accuracy and returns the R-matrix scale factor.
   * @param accuracy GNSS accuracy in meters (null if no fix)
   * @returns A multiplier for the GNSS measurement covariance (R). Higher = less trust.
   */
  public updateState(accuracy: number | null): number {
    if (accuracy === null) {
      this.currentState = 'WEAK_LOST';
      return 1000.0;
    }

    if (accuracy < 10) {
      this.currentState = 'GOOD';
      return 1.0;
    } else if (accuracy >= 10 && accuracy <= 150) {
      this.currentState = 'DEGRADED';
      if (accuracy <= 50) {
        // Linear scaling for mild degradation (10-50m): 1.0 to 5.0
        return accuracy / 10.0;
      } else {
        // Steep quadratic scaling for high degradation (50-150m): 5.0 to 105.0+
        // Prevents 100m+ urban multipath spikes from dragging position/velocity
        const excess = (accuracy - 50) / 10.0;
        return 5.0 + Math.pow(excess, 2) * 2.0;
      }
    } else {
      this.currentState = 'WEAK_LOST';
      return 500.0;
    }
  }

  public getState(): GnssState {
    return this.currentState;
  }
}
