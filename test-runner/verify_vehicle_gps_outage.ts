import { fusionRuntime } from '../src/services/ekf/FusionRuntime';

async function runVehicleGpsOutageVerification() {
  console.log('=== Starting Test 1: Vehicle GPS Outage While Moving Verification ===\n');

  const originLat = 17.659920;
  const originLon = 75.906408;
  fusionRuntime.reset(originLat, originLon);

  let currentTimeMs = 1790533000000;
  const dtSec = 0.05; // 20 Hz IMU loop
  const dtMs = Math.round(dtSec * 1000);

  // 1. Initial stationary alignment: 1 second
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

  const cruiseSpeedKmh = 60.0;
  const cruiseSpeedMps = cruiseSpeedKmh / 3.6; // 16.667 m/s
  let currentLat = originLat;
  let currentLon = originLon;

  console.log('--- Step 1: Cruising at 60 km/h with Good GNSS (5 seconds) ---');
  for (let t = 0; t < 5.0; t += dtSec) {
    currentTimeMs += dtMs;
    const accelAy = t < 1.5 ? (cruiseSpeedMps / 1.5) : 0;
    const currentSpeed = Math.min(cruiseSpeedMps, (t / 1.5) * cruiseSpeedMps);
    const distStep = currentSpeed * dtSec;
    currentLat += (distStep / 111320); // Moving North

    // Real highway road vibration: small oscillations in accel and gyro plus initial vehicle acceleration
    const angle = t * 25.0;
    const vibAx = 0.3 * Math.sin(angle);
    const vibAy = accelAy + 0.35 * Math.cos(angle * 1.2);
    const vibAz = 9.81 + 0.3 * Math.sin(angle * 1.8);
    const vibGx = 0.04 * Math.sin(angle * 0.8);
    const vibGy = 0.04 * Math.cos(angle * 1.4);
    const vibGz = 0.02 * Math.sin(angle * 1.6);

    fusionRuntime.processImuSample(
      { x: vibAx, y: vibAy, z: vibAz },
      { x: vibGx, y: vibGy, z: vibGz },
      currentTimeMs
    );

    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: currentLat,
        longitude: currentLon,
        accuracy: 4.0,
        speed: currentSpeed,
        heading: 0,
        timestamp: currentTimeMs,
      });
    }
  }

  const preOutageState = fusionRuntime.getLatestState();
  const preOutageSpeed = preOutageState ? Math.sqrt(preOutageState.velocity.x ** 2 + preOutageState.velocity.y ** 2) * 3.6 : 0;
  console.log(`Pre-Outage Speed: ${preOutageSpeed.toFixed(2)} km/h, Mode: ${preOutageState?.sourceMode}`);

  console.log('\n--- Step 2: Injected 7-Second Complete GPS Outage (Tunnels/Urban Canyon) ---');
  const outageDurationSec = 7.0;
  const outageSpeeds: number[] = [];
  const trueOutageStartLat = currentLat;
  const trueOutageStartLon = currentLon;
  let trueLatAtEnd = currentLat;

  for (let t = 0; t < outageDurationSec; t += dtSec) {
    currentTimeMs += dtMs;
    const distStep = cruiseSpeedMps * dtSec;
    currentLat += (distStep / 111320); // Ground truth continues advancing North
    trueLatAtEnd = currentLat;

    // Vehicle continues highway driving with real road vibration
    const angle = (5.0 + t) * 25.0;
    const vibAx = 0.3 * Math.sin(angle);
    const vibAy = 0.35 * Math.cos(angle * 1.2);
    const vibAz = 9.81 + 0.3 * Math.sin(angle * 1.8);
    const vibGx = 0.04 * Math.sin(angle * 0.8);
    const vibGy = 0.04 * Math.cos(angle * 1.4);
    const vibGz = 0.02 * Math.sin(angle * 1.6);

    // NO GNSS updates sent during this 7-second blackout
    fusionRuntime.processImuSample(
      { x: vibAx, y: vibAy, z: vibAz },
      { x: vibGx, y: vibGy, z: vibGz },
      currentTimeMs
    );

    const state = fusionRuntime.getLatestState();
    if (state) {
      const speedKmh = Math.sqrt(state.velocity.x ** 2 + state.velocity.y ** 2) * 3.6;
      outageSpeeds.push(speedKmh);
    }
  }

  const endOutageState = fusionRuntime.getLatestState();
  const minOutageSpeed = Math.min(...outageSpeeds);
  const maxOutageSpeed = Math.max(...outageSpeeds);
  const avgOutageSpeed = outageSpeeds.reduce((a, b) => a + b, 0) / outageSpeeds.length;

  const trueDistanceTraveledMeters = cruiseSpeedMps * outageDurationSec; // 116.67 meters
  const drDistanceTraveledMeters = ((endOutageState?.latitude! - trueOutageStartLat) * 111320);
  const positionDriftMeters = Math.abs(endOutageState?.latitude! - trueLatAtEnd) * 111320;

  console.log(`Outage Speed Profile: Min = ${minOutageSpeed.toFixed(2)} km/h, Max = ${maxOutageSpeed.toFixed(2)} km/h, Avg = ${avgOutageSpeed.toFixed(2)} km/h`);
  console.log(`True Distance Traveled during Outage: ${trueDistanceTraveledMeters.toFixed(2)} m`);
  console.log(`Dead-Reckoned Distance Advanced: ${drDistanceTraveledMeters.toFixed(2)} m`);
  console.log(`Accumulated Position Drift Error: ${positionDriftMeters.toFixed(2)} meters over ${outageDurationSec}s (${(positionDriftMeters / outageDurationSec).toFixed(2)} m/s drift rate)`);
  console.log(`Source Mode at end of Outage: ${endOutageState?.sourceMode}`);

  console.log('\n--- Step 3: GNSS Reacquisition (Clear Sky Restored) ---');
  // Re-acquire GNSS fix at ground truth position
  currentTimeMs += dtMs;
  fusionRuntime.updateGnss({
    latitude: currentLat,
    longitude: currentLon,
    accuracy: 4.0,
    speed: cruiseSpeedMps,
    heading: 0,
    timestamp: currentTimeMs,
  });

  const postReacqState = fusionRuntime.getLatestState();
  const postReacqSpeed = postReacqState ? Math.sqrt(postReacqState.velocity.x ** 2 + postReacqState.velocity.y ** 2) * 3.6 : 0;
  const postReacqPosError = Math.abs(postReacqState?.latitude! - currentLat) * 111320;

  console.log(`Post-Reacquisition Speed: ${postReacqSpeed.toFixed(2)} km/h (Expected: ~60 km/h)`);
  console.log(`Post-Reacquisition Position Error: ${postReacqPosError.toFixed(2)} m (Expected: < 1.0m)`);
  console.log(`Post-Reacquisition Source Mode: ${postReacqState?.sourceMode}`);

  console.log('\n=== Test 1 Summary & Checks ===');
  const passedNoCollapse = minOutageSpeed >= 45.0; // Speed did not collapse to zero
  const passedAdvance = drDistanceTraveledMeters >= 90.0 && drDistanceTraveledMeters <= 140.0; // Position advanced plausibly
  const passedReacq = postReacqPosError < 1.0 && postReacqSpeed >= 55.0; // Clean reacquisition

  console.log(`1. Speed Maintained Through Outage: ${passedNoCollapse ? 'PASSED' : 'FAILED'} (Min: ${minOutageSpeed.toFixed(2)} km/h)`);
  console.log(`2. Position Advanced Plausibly: ${passedAdvance ? 'PASSED' : 'FAILED'} (${drDistanceTraveledMeters.toFixed(2)}m advanced)`);
  console.log(`3. Clean GNSS Reacquisition: ${passedReacq ? 'PASSED' : 'FAILED'}`);

  if (passedNoCollapse && passedAdvance && passedReacq) {
    console.log('\nTEST 1 (VEHICLE GPS OUTAGE WHILE MOVING) PASSED!');
  } else {
    throw new Error('Test 1 failed.');
  }
}

runVehicleGpsOutageVerification().catch((e) => {
  console.error(e);
  process.exit(1);
});
