export type GnssState = 'GOOD' | 'DEGRADED' | 'WEAK_LOST';

/**
 * GNSS Quality State Machine with hysteresis debounce.
 *
 * Hysteresis prevents rapid GOOD↔DEGRADED toggling caused by single-sample
 * accuracy spikes (e.g. one bad 11 m reading in an otherwise clean fix).
 *
 * Rules:
 *   - Downgrade (GOOD→DEGRADED, DEGRADED→WEAK_LOST): require DOWNGRADE_HOLD
 *     consecutive samples in the lower state before committing.
 *   - Upgrade (WEAK_LOST→DEGRADED, DEGRADED→GOOD): immediate — we always
 *     want to trust a good fix quickly. Debounce on downgrade only.
 */
export class GnssQualityStateMachine {
  public currentState: GnssState = 'WEAK_LOST';

  // Number of consecutive degraded/lost readings required before downgrading
  private readonly downgradeHold: number;

  // Candidate state and how many consecutive samples have been in it
  private candidateState: GnssState = 'WEAK_LOST';
  private candidateCount: number = 0;

  constructor(downgradeHold: number = 1) {
    this.downgradeHold = downgradeHold;
  }

  /**
   * Updates the state machine with the latest GNSS accuracy and returns the R-matrix scale factor.
   * @param accuracy GNSS accuracy in meters (null if no fix)
   * @param hdop Optional Horizontal Dilution of Precision
   * @param satCount Optional visible/used satellite count
   * @returns A multiplier for the GNSS measurement covariance (R). Higher = less trust.
   */
  public updateState(
    accuracy: number | null,
    hdop?: number | null,
    satCount?: number | null
  ): number {
    const raw = this.classifyRaw(accuracy, hdop, satCount);

    if (raw === 'WEAK_LOST') {
      // WEAK_LOST (null fix / very poor): always immediate — entering a tunnel must
      // trigger IDR without delay.
      this.currentState = 'WEAK_LOST';
      this.candidateState = 'WEAK_LOST';
      this.candidateCount = 0;
    } else if (this.isUpgrade(raw)) {
      // Upgrade (better signal): always immediate.
      this.currentState = raw;
      this.candidateState = raw;
      this.candidateCount = 0;
    } else if (raw === this.currentState) {
      // Same state — keep candidate reset.
      this.candidateState = raw;
      this.candidateCount = 0;
    } else {
      // Downgrade: require downgradeHold consecutive samples if downgradeHold > 1.
      if (raw === this.candidateState) {
        this.candidateCount++;
      } else {
        this.candidateState = raw;
        this.candidateCount = 1;
      }
      if (this.candidateCount >= this.downgradeHold) {
        this.currentState = raw;
        this.candidateCount = 0;
      }
    }

    return this.scaleFor(accuracy, hdop, satCount);
  }

  /** Raw classification with accuracy, HDOP, and satellite count */
  public classifyRaw(
    accuracy: number | null,
    hdop?: number | null,
    satCount?: number | null
  ): GnssState {
    if (accuracy === null || accuracy > 150) return 'WEAK_LOST';
    if (satCount !== undefined && satCount !== null && satCount < 4) return 'WEAK_LOST';
    if (hdop !== undefined && hdop !== null && hdop > 5.0) return 'WEAK_LOST';

    if (
      accuracy >= 10 ||
      (hdop !== undefined && hdop !== null && hdop > 2.0) ||
      (satCount !== undefined && satCount !== null && satCount < 6)
    ) {
      return 'DEGRADED';
    }

    return 'GOOD';
  }

  /** True if transitioning to a "better" state (upgrade direction) */
  private isUpgrade(raw: GnssState): boolean {
    const rank: Record<GnssState, number> = { GOOD: 2, DEGRADED: 1, WEAK_LOST: 0 };
    return rank[raw] > rank[this.currentState];
  }

  /** R scale factor based on accuracy and HDOP (independent of hysteresis state) */
  public scaleFor(
    accuracy: number | null,
    hdop?: number | null,
    satCount?: number | null
  ): number {
    if (accuracy === null) return 1000.0;
    let baseScale = 1.0;
    if (accuracy < 10) {
      baseScale = 1.0;
    } else if (accuracy <= 50) {
      baseScale = accuracy / 10.0; // linear 1.0→5.0
    } else if (accuracy <= 150) {
      const excess = (accuracy - 50) / 10.0;
      baseScale = 5.0 + Math.pow(excess, 2) * 2.0; // quadratic 5.0→105+
    } else {
      baseScale = 500.0;
    }

    // Penalize high HDOP or low satellite count if present
    if (hdop !== undefined && hdop !== null && hdop > 1.5) {
      baseScale *= Math.min(hdop / 1.5, 5.0);
    }
    if (satCount !== undefined && satCount !== null && satCount < 6) {
      baseScale *= satCount <= 3 ? 3.0 : 1.5;
    }

    return baseScale;
  }

  public getState(): GnssState {
    return this.currentState;
  }
}
