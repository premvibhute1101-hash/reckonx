import { EkfCore } from '../EkfCore';

describe('EkfCore Integration Test (Synthetic Blackout)', () => {
  it('maintains continuous position estimation through a 30s GNSS blackout', () => {
    const ekf = new EkfCore();

    // Simulation params
    const dt = 0.1; // 10Hz IMU
    const totalTime = 60; // seconds
    const steps = totalTime / dt;

    // Simulate vehicle moving forward at constant velocity (5 m/s in Y direction)
    const trueVel = [0, 5, 0];
    const accel = [0, 0, 9.81]; // Constant 1g up (perfectly level)
    const gyro = [0, 0, 0];
    
    let simulatedYPos = 0;
    const dummyImuWindow = Array(20).fill([0, 0, 9.81, 0, 0, 0]);

    // Initialize EKF to prevent outlier rejection on the first frame
    ekf.getIns().velocity.y = 5;

    for (let step = 0; step < steps; step++) {
      const t = step * dt;
      simulatedYPos += trueVel[1] * dt;
      

      // 1. Predict (runs every 10Hz tick)
      ekf.predict(dt, accel, gyro);

      // 2. GNSS Update (runs at 1Hz)
      if (step % 10 === 0) {
        let accuracy: number | null = 5.0; // 5m accuracy (GOOD)
        
        // Blackout period: 20s to 50s
        if (t >= 20 && t < 50) {
          accuracy = null; // No fix! (WEAK_LOST)
        }

        // Add some noise to GNSS if it's available
        let gnssPos = [0, simulatedYPos, 0];
        let gnssVel = [...trueVel];

        if (accuracy !== null) {
          gnssPos[1] += (Math.random() - 0.5) * 2; // +/- 1m noise
        }

        ekf.updateGnss(gnssPos, gnssVel, accuracy, dummyImuWindow);
      }
    }

    // At t=60s, true position is 300m (60 * 5)
    // Even with a 30s blackout, the EKF should not have wildly diverged 
    // (since velocity was maintained by AI mock / NHC).
    const finalPos = ekf.getPosition();
    
    // Check if the final position is within a reasonable bound (e.g. 10 meters of truth)
    console.log(`Final true Y: 300m. Final EKF Y: ${finalPos.y}m`);
    expect(finalPos.y).toBeGreaterThan(290);
    expect(finalPos.y).toBeLessThan(310);
    
    // X drift should be minimal due to NHC/AI keeping it steady
    expect(Math.abs(finalPos.x)).toBeLessThan(10);
  });
});
