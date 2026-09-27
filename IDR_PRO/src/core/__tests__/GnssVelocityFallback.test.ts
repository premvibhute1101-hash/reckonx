import { EkfCore } from '../EkfCore';

describe('GNSS Velocity Fallback (Bug Fix Regression)', () => {

  const runSimulation = (simulateOldBug: boolean) => {
    const ekf = new EkfCore();
    
    const dt = 0.1; // 10Hz IMU
    let time = 0;
    
    // Simulate motion at ~1.4 m/s in East (X) direction
    const trueSpeed = 1.4;
    
    let currentX = 0;
    let velocityLog: number[] = [];

    // Phase 1: 10s of GNSS fixes (position only), moving at 1.4m/s
    // IMU runs at 10Hz, GNSS at 1Hz
    for (let step = 0; step < 100; step++) {
      time += dt;
      currentX += trueSpeed * dt;

      // Accelerometer sees 0 acceleration (constant velocity) except for initial bump
      const ax = step < 5 ? 0.28 : 0; // Accelerate to 1.4m/s over 0.5s
      
      // Predict step
      ekf.predict(dt, [ax, 0, 9.81], [0, 0, 0]);

      // 1Hz GNSS Update
      if (step % 10 === 0) {
        let gnssVel: number[] | null = null;
        
        if (simulateOldBug) {
          // OLD BEHAVIOR: FusionRuntime passes [0,0,0] when speed/heading are null
          gnssVel = [0, 0, 0];
        } else {
          // NEW BEHAVIOR: FusionRuntime passes null
          gnssVel = null;
        }

        // GNSS provides current position, but no velocity
        ekf.updateGnss([currentX, 0, 0], gnssVel, 5.0, []);
      }
    }

    const velocityAtGNSSLost = ekf.getIns().velocity.x;
    const positionAtGNSSLost = ekf.getPosition().x;

    // Phase 2: 10s of GNSS Blackout (IMU only)
    for (let step = 0; step < 100; step++) {
      time += dt;
      currentX += trueSpeed * dt;
      
      // Constant velocity, no acceleration
      ekf.predict(dt, [0, 0, 9.81], [0, 0, 0]);
      
      // GNSS is lost, so no updateGnss is called (or called with null accuracy, which gets rejected)
      // To simulate FusionRuntime strictly, we could call updateGnss([0,0,0], [0,0,0], null) occasionally
      if (step % 10 === 0) {
         ekf.updateGnss([0, 0, 0], [0, 0, 0], null, []); // Accuracy null -> WEAK_LOST
      }
      
      velocityLog.push(ekf.getIns().velocity.x);
    }

    const positionAtEnd = ekf.getPosition().x;
    
    return {
      velocityAtGNSSLost,
      positionAtGNSSLost,
      positionAtEnd,
      velocityLog
    };
  };

  it('NEW BEHAVIOR: Velocity tracks motion and position propagates smoothly during blackout', () => {
    const result = runSimulation(false);
    
    console.log(`NEW BEHAVIOR Numeric Output:
      - Velocity tracked during position-only GNSS: ${result.velocityAtGNSSLost.toFixed(3)} m/s (Expected ~1.4 m/s)
      - Final blackout position (X): ${result.positionAtEnd.toFixed(3)} m
      - Position at start of blackout (X): ${result.positionAtGNSSLost.toFixed(3)} m
      - Distance moved during blackout: ${(result.positionAtEnd - result.positionAtGNSSLost).toFixed(3)} m (Expected > 5 m)
    `);

    // 1. Velocity should NOT be pinned to 0. It should be close to 1.4 m/s.
    // (Due to imperfect IMU integration in simplified test, it might not be exactly 1.4, but definitely > 0.5)
    expect(result.velocityAtGNSSLost).toBeGreaterThan(0.5);
    
    // 2. Position should continue propagating forward.
    // Over 10s at roughly 1.4m/s, it should advance by ~14m.
    const distanceMovedInBlackout = result.positionAtEnd - result.positionAtGNSSLost;
    expect(distanceMovedInBlackout).toBeGreaterThan(5);
  });

  it('OLD BEHAVIOR: Velocity is pinned to 0 during position-only GNSS, causing failure to track true velocity', () => {
    const result = runSimulation(true);

    console.log(`OLD BEHAVIOR Numeric Output:
      - Velocity tracked during position-only GNSS: ${result.velocityAtGNSSLost.toFixed(3)} m/s (Expected ~0 m/s)
    `);
    
    // 1. Velocity WAS artificially pinned near 0 because of the [0,0,0] GNSS update.
    expect(result.velocityAtGNSSLost).toBeLessThan(0.5);
    
    // 2. Because velocity was incorrectly pinned, the EKF learns a massive false accelerometer bias
    // to explain the GNSS position drift. This means the position during blackout will be driven 
    // by this false bias (quadratic drift) rather than true constant velocity.
    // We just assert that it completely fails to track the true 1.4m/s velocity.
    expect(result.velocityAtGNSSLost).not.toBeCloseTo(1.4, 1);
  });

});
