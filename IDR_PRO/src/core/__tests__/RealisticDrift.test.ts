import { EkfCore } from '../EkfCore';

describe('Realistic Stationary Drift (noisy sensors)', () => {
  it('simulates real phone sensor noise at rest for 30s', () => {
    const ekf = new EkfCore();
    const dt = 0.1;
    const totalTime = 30;
    const steps = totalTime / dt;

    // Initialize attitude from gravity
    ekf.getIns().initializeAttitude({ x: 0.05, y: -0.03, z: 9.78 });

    const dummyImuWindow: number[][] = [];
    for (let i = 0; i < 20; i++) {
      dummyImuWindow.push([0.05, -0.03, 9.78, 0.001, -0.002, 0.0005]);
    }

    // Seed for reproducibility
    let seed = 42;
    function seededRandom() {
      seed = (seed * 16807) % 2147483647;
      return (seed - 1) / 2147483646;
    }
    function gaussianNoise(stddev: number): number {
      const u1 = seededRandom();
      const u2 = seededRandom();
      return stddev * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    }

    // Typical MEMS phone sensor characteristics:
    // Accel bias: ~0.02-0.05 m/s^2 residual after gravity subtraction
    // Accel noise: ~0.05 m/s^2 per axis
    // Gyro noise: ~0.01 rad/s per axis
    const accelBias = { x: 0.03, y: -0.02, z: -0.05 }; // residual bias after gravity
    const accelNoise = 0.05;
    const gyroNoise = 0.005;

    const velocityLog: { t: number; vx: number; vy: number; vz: number; speed: number }[] = [];

    let zupts = 0;
    const imuWindow: number[][] = [];
    for (let i = 0; i < 20; i++) {
      imuWindow.push([accelBias.x, accelBias.y, 9.81 + accelBias.z, 0, 0, 0]);
    }

    for (let step = 0; step < steps; step++) {
      const t = step * dt;

      // Simulate noisy stationary accelerometer (phone on table)
      const ax = accelBias.x + gaussianNoise(accelNoise);
      const ay = accelBias.y + gaussianNoise(accelNoise);
      const az = 9.81 + accelBias.z + gaussianNoise(accelNoise);

      const gx = gaussianNoise(gyroNoise);
      const gy = gaussianNoise(gyroNoise);
      const gz = gaussianNoise(gyroNoise);

      const imuSample = [ax, ay, az, gx, gy, gz];

      // Maintain rolling window (keep exactly 20)
      imuWindow.push(imuSample);
      while (imuWindow.length > 20) imuWindow.shift();

      ekf.predict(dt, [ax, ay, az], [gx, gy, gz]);

      // Simulate GNSS at 1Hz
      if (step % 10 === 0) {
        ekf.updateGnss([0, 0, 0], [0, 0, 0], 5.0, imuWindow);
      }

      // ZUPT detection (same logic as fixed FusionRuntime)
      if (imuWindow.length >= 20) {
        const n = imuWindow.length;
        let sumAx = 0, sumAy = 0, sumAz = 0;
        let sumGx = 0, sumGy = 0, sumGz = 0;
        for (const s of imuWindow) {
          sumAx += s[0]; sumAy += s[1]; sumAz += s[2];
          sumGx += s[3]; sumGy += s[4]; sumGz += s[5];
        }
        const meanAx = sumAx / n, meanAy = sumAy / n, meanAz = sumAz / n;
        const meanGx = sumGx / n, meanGy = sumGy / n, meanGz = sumGz / n;

        let varA = 0, varG = 0;
        for (const s of imuWindow) {
          varA += Math.pow(s[0] - meanAx, 2) + Math.pow(s[1] - meanAy, 2) + Math.pow(s[2] - meanAz, 2);
          varG += Math.pow(s[3] - meanGx, 2) + Math.pow(s[4] - meanGy, 2) + Math.pow(s[5] - meanGz, 2);
        }
        varA /= n;
        varG /= n;

        if (varA < 0.5 && varG < 0.05) {
          ekf.updateZupt();
          zupts++;
        }
      }

      const vel = ekf.getIns().velocity;
      const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
      velocityLog.push({
        t: parseFloat(t.toFixed(1)),
        vx: parseFloat(vel.x.toFixed(4)),
        vy: parseFloat(vel.y.toFixed(4)),
        vz: parseFloat(vel.z.toFixed(4)),
        speed: parseFloat(speed.toFixed(4)),
      });
    }

    // Log at 1-second intervals
    for (let i = 0; i < velocityLog.length; i += 10) {
      const v = velocityLog[i];
      console.log(`t=${v.t}s: vx=${v.vx}, vy=${v.vy}, vz=${v.vz}, |v|=${v.speed}`);
    }

    const finalVel = velocityLog[velocityLog.length - 1];
    console.log(`\nFinal: |v|=${finalVel.speed} m/s after ${totalTime}s at rest`);
    console.log(`ZUPTs applied: ${zupts}`);

    const biases = ekf.getBiases();
    console.log(`Estimated accel biases: x=${biases.accel.x.toFixed(4)}, y=${biases.accel.y.toFixed(4)}, z=${biases.accel.z.toFixed(4)}`);

    // REGRESSION: must stay < 0.1 m/s at rest even with realistic noise
    expect(finalVel.speed).toBeLessThan(0.1);
  });
});
