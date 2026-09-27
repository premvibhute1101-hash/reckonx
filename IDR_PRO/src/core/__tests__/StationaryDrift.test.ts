import { EkfCore } from '../EkfCore';

describe('Stationary Velocity Drift Diagnosis', () => {
  it('logs velocity over 10s with phone at rest on a table', () => {
    const ekf = new EkfCore();
    const dt = 0.1; // 10Hz
    const totalTime = 10; // seconds
    const steps = totalTime / dt;

    // Stationary phone: accel reads ~[0, 0, 9.81] in body frame (gravity along Z)
    // Gyro reads ~[0, 0, 0]
    const accel = [0, 0, 9.81];
    const gyro = [0, 0, 0];

    // Initialize attitude from gravity vector
    ekf.getIns().initializeAttitude({ x: accel[0], y: accel[1], z: accel[2] });

    const dummyImuWindow = Array(20).fill([0, 0, 9.81, 0, 0, 0]);
    const velocityLog: { t: number; vx: number; vy: number; vz: number; speed: number }[] = [];

    for (let step = 0; step < steps; step++) {
      const t = step * dt;

      ekf.predict(dt, accel, gyro);

      // Simulate GNSS at 1Hz
      if (step % 10 === 0) {
        ekf.updateGnss([0, 0, 0], [0, 0, 0], 5.0, dummyImuWindow);
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

    // Log samples at 1-second intervals
    for (let i = 0; i < velocityLog.length; i += 10) {
      console.log(`t=${velocityLog[i].t}s: vx=${velocityLog[i].vx}, vy=${velocityLog[i].vy}, vz=${velocityLog[i].vz}, |v|=${velocityLog[i].speed}`);
    }

    const finalVel = velocityLog[velocityLog.length - 1];
    console.log(`\nFinal velocity at ${totalTime}s: |v|=${finalVel.speed} m/s`);

    // REGRESSION: at rest, speed must stay < 0.1 m/s over 10 seconds
    expect(finalVel.speed).toBeLessThan(0.1);
  });

  it('logs velocity over 10s at rest WITHOUT GNSS (pure INS + ZUPT)', () => {
    const ekf = new EkfCore();
    const dt = 0.1;
    const totalTime = 10;
    const steps = totalTime / dt;

    const accel = [0, 0, 9.81];
    const gyro = [0, 0, 0];

    ekf.getIns().initializeAttitude({ x: accel[0], y: accel[1], z: accel[2] });

    // Build a stationary IMU window for ZUPT detection
    const imuWindow: number[][] = [];
    for (let i = 0; i < 20; i++) {
      imuWindow.push([0, 0, 9.81, 0, 0, 0]);
    }

    const velocityLog: { t: number; speed: number }[] = [];

    for (let step = 0; step < steps; step++) {
      const t = step * dt;

      ekf.predict(dt, accel, gyro);

      // No GNSS - simulate ZUPT detection (variance = 0 for constant readings)
      // variance of [0,0,9.81,0,0,0] repeated = 0, so ZUPT should trigger
      ekf.updateZupt();

      const vel = ekf.getIns().velocity;
      const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
      velocityLog.push({
        t: parseFloat(t.toFixed(1)),
        speed: parseFloat(speed.toFixed(6)),
      });
    }

    for (let i = 0; i < velocityLog.length; i += 10) {
      console.log(`t=${velocityLog[i].t}s: |v|=${velocityLog[i].speed}`);
    }

    const finalSpeed = velocityLog[velocityLog.length - 1].speed;
    console.log(`\nFinal velocity at ${totalTime}s (no GNSS, ZUPT every tick): |v|=${finalSpeed} m/s`);

    expect(finalSpeed).toBeLessThan(0.1);
  });
});
