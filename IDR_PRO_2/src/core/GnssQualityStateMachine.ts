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
      return 1000.0; // Huge inflation, effectively rejecting the measurement if it were used
    }

    if (accuracy < 10) {
      this.currentState = 'GOOD';
      return 1.0; // Base trust
    } else if (accuracy >= 10 && accuracy <= 25) {
      this.currentState = 'DEGRADED';
      // Scale R linearly or quadratically. Let's do a simple scaling:
      return Math.pow(accuracy / 5.0, 2); 
    } else {
      this.currentState = 'WEAK_LOST';
      return 1000.0; // Highly inflated
    }
  }

  public getState(): GnssState {
    return this.currentState;
  }
}
