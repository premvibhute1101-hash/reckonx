import { fusionRuntime } from '../src/services/ekf/FusionRuntime';

async function runVehicleVerification() {
  console.log('=== Starting Vehicle Mount 4-Phase Trajectory Verification ===\n');

  const originLat = 17.659920;
  const originLon = 75.906408;
  fusionRuntime.reset(originLat, originLon);

  let currentTimeMs = 1790532000000;
  const dtSec = 0.05; // 20 Hz IMU loop
  const dtMs = Math.round(dtSec * 1000);

  // Initial alignment: 1 second resting
  for (let i = 0; i < 20; i++) {
    fusionRuntime.processImuSample({ x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 }, currentTimeMs);
    currentTimeMs += dtMs;
  }
  fusionRuntime.updateGnss({
    latitude: originLat,
    longitude: originLon,
    accuracy: 4.0,
    speed: 0,
    heading: 0,
    timestamp: currentTimeMs,
  });

  console.log('--- Phase (a): Engine Idling at Stop (5 seconds) ---');
  let maxSpeedDuringIdle = 0;
  let maxPosDriftDuringIdle = 0;

  for (let t = 0; t < 5.0; t += dtSec) {
    currentTimeMs += dtMs;
    // Engine vibration pattern: high frequency vibration producing elevated variance
    const angle = t * 30.0; // 30 rad/s (~5 Hz engine vibration)
    const vibAx = 0.5 * Math.sin(angle);
    const vibAy = 0.5 * Math.cos(angle * 1.3);
    const vibAz = 9.81 + 0.4 * Math.sin(angle * 2.1);
    const vibGx = 0.06 * Math.sin(angle * 0.9);
    const vibGy = 0.06 * Math.cos(angle * 1.1);
    const vibGz = 0.04 * Math.sin(angle * 1.7);

    fusionRuntime.processImuSample(
      { x: vibAx, y: vibAy, z: vibAz },
      { x: vibGx, y: vibGy, z: vibGz },
      currentTimeMs
    );

    // 1 Hz GNSS update reporting 0 speed
    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: originLat,
        longitude: originLon,
        accuracy: 4.5,
        speed: 0.0,
        heading: 0,
        timestamp: currentTimeMs,
      });
    }

    const state = fusionRuntime.getLatestState();
    if (state) {
      const speedKmh = Math.sqrt(state.velocity.x ** 2 + state.velocity.y ** 2) * 3.6;
      if (speedKmh > maxSpeedDuringIdle) maxSpeedDuringIdle = speedKmh;
      const dLatM = (state.latitude! - originLat) * 111320;
      const dLonM = (state.longitude! - originLon) * 111320 * Math.cos(originLat * Math.PI / 180);
      const drift = Math.sqrt(dLatM ** 2 + dLonM ** 2);
      if (drift > maxPosDriftDuringIdle) maxPosDriftDuringIdle = drift;
    }
  }

  console.log(`Phase (a) Idle Result: Max Speed = ${maxSpeedDuringIdle.toFixed(2)} km/h, Max Drift = ${maxPosDriftDuringIdle.toFixed(3)} m`);

  console.log('\n--- Phase (b): Accelerating from Stop to 60 km/h (8 seconds) ---');
  let currentSpeedMps = 0;
  const targetSpeedMps = 60.0 / 3.6; // 16.667 m/s
  const accelMps2 = targetSpeedMps / 8.0; // ~2.08 m/s^2
  let currentLat = originLat;
  let currentLon = originLon;

  for (let t = 0; t < 8.0; t += dtSec) {
    currentTimeMs += dtMs;
    currentSpeedMps += accelMps2 * dtSec;
    const currentDist = currentSpeedMps * dtSec;
    currentLat += (currentDist / 111320); // Moving North

    // Forward acceleration in Y axis (North in level frame)
    fusionRuntime.processImuSample(
      { x: 0.05 * (Math.random() - 0.5), y: accelMps2 + 0.05 * (Math.random() - 0.5), z: 9.81 },
      { x: 0.01 * (Math.random() - 0.5), y: 0.01 * (Math.random() - 0.5), z: 0.005 * (Math.random() - 0.5) },
      currentTimeMs
    );

    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: currentLat,
        longitude: currentLon,
        accuracy: 4.2,
        speed: currentSpeedMps,
        heading: 0,
        timestamp: currentTimeMs,
      });
    }
  }

  const statePostAccel = fusionRuntime.getLatestState();
  const speedPostAccel = statePostAccel ? Math.sqrt(statePostAccel.velocity.x ** 2 + statePostAccel.velocity.y ** 2) * 3.6 : 0;
  console.log(`Phase (b) Acceleration Result: Reached Speed = ${speedPostAccel.toFixed(2)} km/h (Expected: ~60 km/h)`);

  console.log('\n--- Phase (c): Steady Cruising on Smooth Road (10 seconds) ---');
  // Cruising at 60 km/h on smooth highway: flat IMU signals (very low variance)
  const cruiseSpeeds: number[] = [];
  let minCruiseSpeed = 999;
  let maxCruiseSpeed = 0;

  for (let t = 0; t < 10.0; t += dtSec) {
    currentTimeMs += dtMs;
    const currentDist = targetSpeedMps * dtSec;
    currentLat += (currentDist / 111320);

    // Smooth road: very low IMU variance (flat accel & gyro)
    fusionRuntime.processImuSample(
      { x: 0.01 * (Math.random() - 0.5), y: 0.01 * (Math.random() - 0.5), z: 9.81 + 0.01 * (Math.random() - 0.5) },
      { x: 0.002 * (Math.random() - 0.5), y: 0.002 * (Math.random() - 0.5), z: 0.001 * (Math.random() - 0.5) },
      currentTimeMs
    );

    // 1 Hz GNSS update at 60 km/h
    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: currentLat,
        longitude: currentLon,
        accuracy: 4.0,
        speed: targetSpeedMps,
        heading: 0,
        timestamp: currentTimeMs,
      });
    }

    const state = fusionRuntime.getLatestState();
    if (state) {
      const speedKmh = Math.sqrt(state.velocity.x ** 2 + state.velocity.y ** 2) * 3.6;
      cruiseSpeeds.push(speedKmh);
      if (speedKmh < minCruiseSpeed) minCruiseSpeed = speedKmh;
      if (speedKmh > maxCruiseSpeed) maxCruiseSpeed = speedKmh;
    }
  }

  const avgCruiseSpeed = cruiseSpeeds.reduce((a, b) => a + b, 0) / cruiseSpeeds.length;
  console.log(`Phase (c) Cruise Result: Avg Speed = ${avgCruiseSpeed.toFixed(2)} km/h, Min = ${minCruiseSpeed.toFixed(2)} km/h, Max = ${maxCruiseSpeed.toFixed(2)} km/h`);
  const cruiseVariation = maxCruiseSpeed - minCruiseSpeed;
  console.log(`Cruise Speed Fluctuation (Sawtooth): ${cruiseVariation.toFixed(2)} km/h`);

  console.log('\n--- Phase (d): Braking to a Stop (6 seconds) ---');
  currentSpeedMps = targetSpeedMps;
  const decelMps2 = targetSpeedMps / 6.0; // ~2.78 m/s^2

  for (let t = 0; t < 6.0; t += dtSec) {
    currentTimeMs += dtMs;
    currentSpeedMps = Math.max(0, currentSpeedMps - decelMps2 * dtSec);
    const currentDist = currentSpeedMps * dtSec;
    currentLat += (currentDist / 111320);

    fusionRuntime.processImuSample(
      { x: 0.05 * (Math.random() - 0.5), y: -decelMps2 + 0.05 * (Math.random() - 0.5), z: 9.81 },
      { x: 0.01 * (Math.random() - 0.5), y: 0.01 * (Math.random() - 0.5), z: 0.005 * (Math.random() - 0.5) },
      currentTimeMs
    );

    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: currentLat,
        longitude: currentLon,
        accuracy: 4.5,
        speed: currentSpeedMps,
        heading: 0,
        timestamp: currentTimeMs,
      });
    }
  }

  // 2 seconds stopped after braking
  for (let t = 0; t < 2.0; t += dtSec) {
    currentTimeMs += dtMs;
    fusionRuntime.processImuSample({ x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 }, currentTimeMs);
    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: currentLat,
        longitude: currentLon,
        accuracy: 4.0,
        speed: 0.0,
        heading: 0,
        timestamp: currentTimeMs,
      });
    }
  }

  const stateFinal = fusionRuntime.getLatestState();
  const speedFinal = stateFinal ? Math.sqrt(stateFinal.velocity.x ** 2 + stateFinal.velocity.y ** 2) * 3.6 : 0;
  console.log(`Phase (d) Final Stopped Speed: ${speedFinal.toFixed(2)} km/h (Expected: 0.00 km/h)`);

  console.log('\n=== Summary of Validation Checks ===');
  const passedIdle = maxSpeedDuringIdle === 0.00 && maxPosDriftDuringIdle < 0.1;
  const passedCruise = avgCruiseSpeed >= 55.0 && avgCruiseSpeed <= 65.0 && cruiseVariation < 5.0;
  const passedFinalStop = speedFinal === 0.00;

  console.log(`1. Engine Idling (Vibration Resistance): ${passedIdle ? 'PASSED (0.0 km/h)' : 'FAILED'}`);
  console.log(`2. Smooth Highway Cruising (Anti-Sawtooth): ${passedCruise ? 'PASSED (Steady ~60 km/h)' : 'FAILED'}`);
  console.log(`3. Braking to Complete Rest: ${passedFinalStop ? 'PASSED (0.0 km/h)' : 'FAILED'}`);

  if (passedIdle && passedCruise && passedFinalStop) {
    console.log('\nALL VEHICLE PROFILE TESTS PASSED!');
  } else {
    throw new Error('Vehicle profile verification failed.');
  }
}

runVehicleVerification().catch((e) => {
  console.error(e);
  process.exit(1);
});
