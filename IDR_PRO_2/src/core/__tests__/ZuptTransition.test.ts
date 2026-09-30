import { EkfCore } from '../EkfCore';

describe('ZUPT-to-Walking Transition Velocity Spike', () => {
  // Seeded PRNG for reproducible noise
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

  it('velocity stays plausible during ZUPT→walking transition', () => {
    const ekf = new EkfCore();
    const dt = 0.1;

    ekf.getIns().initializeAttitude({ x: 0.02, y: -0.01, z: 9.79 });

    // Phase 1: 2 seconds stationary with ZUPT
    console.log('=== PHASE 1: Stationary (2s) ===');
    for (let step = 0; step < 20; step++) {
      const ax = 0.02 + gaussianNoise(0.03);
      const ay = -0.01 + gaussianNoise(0.03);
      const az = 9.81 + gaussianNoise(0.03);
      ekf.predict(dt, [ax, ay, az], [gaussianNoise(0.003), gaussianNoise(0.003), gaussianNoise(0.003)]);
      ekf.updateZupt();
      if (step % 10 === 0) ekf.updateGnss([0, 0, 0], [0, 0, 0], 5.0, []);
    }
    const velAfterZupt = ekf.getIns().velocity;
    console.log(`After ZUPT: vx=${velAfterZupt.x.toFixed(4)}, vy=${velAfterZupt.y.toFixed(4)}, vz=${velAfterZupt.z.toFixed(4)}`);

    // Notify EKF that ZUPT is releasing (this inflates Q)
    ekf.notifyZuptReleased();

    // Phase 2: Physically realistic walking profile
    // Key insight: heel-strike spikes are OSCILLATORY — the Z accel goes UP then DOWN around gravity.
    // Net vertical velocity over a full step cycle should be ~0. Horizontal net accel ~1.5 m/s^2 forward.
    // Walking at ~1.3 m/s → takes ~1s to reach cruising speed.
    console.log('\n=== PHASE 2: Walking Steps (2s) ===');
    const walkingProfile = [
      // Step 1: heel strike oscillation (high Z → low Z around 9.81 mean)
      { ax: 1.5,  ay: 0.3,  az: 15.0 },  // Impact up
      { ax: 0.8,  ay: 0.1,  az: 5.0  },  // Rebound down (below gravity)
      { ax: 0.3,  ay: 0.0,  az: 10.5 },  // Recovery
      { ax: 0.1,  ay: 0.0,  az: 9.0  },  // Swing (below g)
      { ax: -0.1, ay: 0.0,  az: 9.3  },  // Swing
      { ax: 0.0,  ay: 0.0,  az: 9.7  },  // Pre-contact
      { ax: 0.0,  ay: 0.0,  az: 9.81 },  // Mid-stride level
      // Step 2: heel strike oscillation
      { ax: 1.2,  ay: 0.2,  az: 13.0 },  // Impact up
      { ax: 0.6,  ay: 0.1,  az: 6.0  },  // Rebound down
      { ax: 0.2,  ay: 0.0,  az: 10.0 },  // Recovery
      { ax: 0.0,  ay: 0.0,  az: 9.2  },  // Swing
      { ax: -0.1, ay: 0.0,  az: 9.5  },  // Swing
      { ax: 0.0,  ay: 0.0,  az: 9.7  },  // Pre-contact
      { ax: 0.0,  ay: 0.0,  az: 9.81 },  // Level
      // Cruising at ~1.3 m/s (very small forward accel, gravity balanced)
      { ax: 0.1,  ay: 0.0,  az: 9.81 },
      { ax: 0.0,  ay: 0.0,  az: 9.81 },
      { ax: 0.0,  ay: 0.0,  az: 9.81 },
      { ax: 0.0,  ay: 0.0,  az: 9.81 },
      { ax: 0.0,  ay: 0.0,  az: 9.81 },
      { ax: 0.0,  ay: 0.0,  az: 9.81 },
    ];

    let peakSpeed = 0;
    let prevSpeed = 0;
    let maxJump = 0;

    for (let step = 0; step < walkingProfile.length; step++) {
      const { ax, ay, az } = walkingProfile[step];
      ekf.predict(dt, [ax, ay, az], [gaussianNoise(0.01), gaussianNoise(0.01), gaussianNoise(0.01)]);

      const vel = ekf.getIns().velocity;
      const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
      const jump = Math.abs(speed - prevSpeed);
      if (speed > peakSpeed) peakSpeed = speed;
      if (jump > maxJump) maxJump = jump;

      console.log(
        `step ${step}: a=[${ax.toFixed(1)},${ay.toFixed(1)},${az.toFixed(1)}] ` +
        `v=[${vel.x.toFixed(3)},${vel.y.toFixed(3)},${vel.z.toFixed(3)}] ` +
        `|v|=${speed.toFixed(3)} Δ=${jump.toFixed(3)}`
      );
      prevSpeed = speed;
    }

    console.log(`\nPeak: ${peakSpeed.toFixed(3)} m/s | Max jump: ${maxJump.toFixed(3)} m/s`);

    // Pedestrian walking: peak velocity should be < 2.5 m/s
    expect(peakSpeed).toBeLessThan(2.5);
    // No single step should jump > 3 m/s
    expect(maxJump).toBeLessThan(3.0);
  });
});
