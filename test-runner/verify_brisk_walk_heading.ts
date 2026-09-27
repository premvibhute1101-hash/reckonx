import { fusionRuntime } from '../src/services/ekf/FusionRuntime';

async function runBriskWalkHeadingVerification() {
  console.log('=== Starting Test 2: Heading vs Velocity Vector Alignment at Brisk Walking Pace (>5 km/h) ===\n');

  const originLat = 17.659920;
  const originLon = 75.906408;
  fusionRuntime.reset(originLat, originLon);

  let currentTimeMs = 1790534000000;
  const dtSec = 0.05; // 20 Hz IMU loop
  const dtMs = Math.round(dtSec * 1000);

  const walkingSpeedKmh = 5.4; // 1.5 m/s (brisk walking pace)
  const walkingSpeedMps = walkingSpeedKmh / 3.6; // 1.5 m/s

  // 1. Initial stationary alignment (1 second) facing East (90 degrees compass)
  for (let i = 0; i < 20; i++) {
    fusionRuntime.processImuSample({ x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 }, currentTimeMs);
    currentTimeMs += dtMs;
  }
  // Initialize with East heading (90 deg)
  fusionRuntime.updateGnss({
    latitude: originLat,
    longitude: originLon,
    accuracy: 4.0,
    speed: 0,
    heading: 90.0,
    timestamp: currentTimeMs,
  });

  let currentLat = originLat;
  let currentLon = originLon;

  console.log('--- Leg 1: Walking East (90.0° Compass) at 5.4 km/h for 10 seconds ---');
  const leg1Logs: { heading: number; velHdg: number; speedKmh: number; diff: number }[] = [];

  for (let t = 0; t < 10.0; t += dtSec) {
    currentTimeMs += dtMs;
    const distStep = walkingSpeedMps * dtSec;
    // Moving East: latitude is constant, longitude increases
    currentLon += (distStep / (111320 * Math.cos(originLat * Math.PI / 180)));

    // Pedestrian gait acceleration signature (~2 Hz vertical bounce, forward acceleration)
    const gaitPhase = t * 2.0 * Math.PI * 2.0; // 2 steps per second
    const ax = 0.4 * Math.sin(gaitPhase);
    const ay = 0.5 * Math.abs(Math.sin(gaitPhase)); // forward acceleration impulses
    const az = 9.81 + 1.2 * Math.sin(gaitPhase);
    const gx = 0.05 * Math.sin(gaitPhase);
    const gy = 0.05 * Math.cos(gaitPhase);
    const gz = 0.02 * Math.sin(gaitPhase);

    fusionRuntime.processImuSample(
      { x: ax, y: ay, z: az },
      { x: gx, y: gy, z: gz },
      currentTimeMs
    );

    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: currentLat,
        longitude: currentLon,
        accuracy: 4.2,
        speed: walkingSpeedMps,
        heading: 90.0,
        timestamp: currentTimeMs,
      });
    }

    const state = fusionRuntime.getLatestState();
    if (state && t > 2.0) { // after initial ramp-up
      const vx = state.velocity.x; // East
      const vy = state.velocity.y; // North
      const speed = Math.sqrt(vx * vx + vy * vy) * 3.6;

      // Derived heading from velocity vector:
      // In Navigation ENU: East is X, North is Y. Clockwise from North:
      let velHeadingDeg = (Math.atan2(vx, vy) * (180 / Math.PI)) % 360;
      if (velHeadingDeg < 0) velHeadingDeg += 360;
      velHeadingDeg = Math.round(velHeadingDeg * 10) / 10;

      const hdgDiff = Math.abs(state.heading - 90.0);
      leg1Logs.push({
        heading: state.heading,
        velHdg: velHeadingDeg,
        speedKmh: speed,
        diff: hdgDiff
      });
    }
  }

  const avgLeg1Hdg = leg1Logs.reduce((a, b) => a + b.heading, 0) / leg1Logs.length;
  const avgLeg1VelHdg = leg1Logs.reduce((a, b) => a + b.velHdg, 0) / leg1Logs.length;
  const avgLeg1Speed = leg1Logs.reduce((a, b) => a + b.speedKmh, 0) / leg1Logs.length;
  console.log(`Leg 1 (East) Result: Logged Heading = ${avgLeg1Hdg.toFixed(1)}° (Expected: 90.0°), VelVector Heading = ${avgLeg1VelHdg.toFixed(1)}°, Speed = ${avgLeg1Speed.toFixed(2)} km/h`);

  console.log('\n--- Leg 2: Executing 90° Clockwise Turn (East 90° -> South 180°) over 2.0s ---');
  const turnDuration = 2.0;
  // Angular rate for 90 deg (pi/2 rad) turn in 2.0s: yaw rate = -pi/4 rad/s = -0.7854 rad/s (clockwise)
  const turnGz = -Math.PI / (2.0 * turnDuration);

  for (let t = 0; t < turnDuration; t += dtSec) {
    currentTimeMs += dtMs;
    // Continuing to walk while turning
    const currentTurnAngle = 90.0 + (t / turnDuration) * 90.0; // 90 -> 180 deg
    const rad = currentTurnAngle * (Math.PI / 180);
    const dEast = walkingSpeedMps * dtSec * Math.sin(rad);
    const dNorth = walkingSpeedMps * dtSec * Math.cos(rad);

    currentLon += (dEast / (111320 * Math.cos(originLat * Math.PI / 180)));
    currentLat += (dNorth / 111320);

    const gaitPhase = (10.0 + t) * 2.0 * Math.PI * 2.0;
    fusionRuntime.processImuSample(
      { x: 0.3 * Math.sin(gaitPhase), y: 0.4 * Math.abs(Math.sin(gaitPhase)), z: 9.81 + 1.0 * Math.sin(gaitPhase) },
      { x: 0.04 * Math.sin(gaitPhase), y: 0.04 * Math.cos(gaitPhase), z: turnGz },
      currentTimeMs
    );
  }

  const postTurnState = fusionRuntime.getLatestState();
  console.log(`Post-Turn Heading immediately after turn: ${postTurnState?.heading.toFixed(1)}° (Expected: ~180.0°)`);

  console.log('\n--- Leg 3: Walking South (180.0° Compass) at 5.4 km/h for 10 seconds ---');
  const leg3Logs: { heading: number; velHdg: number; speedKmh: number; diff: number }[] = [];

  for (let t = 0; t < 10.0; t += dtSec) {
    currentTimeMs += dtMs;
    const distStep = walkingSpeedMps * dtSec;
    // Moving South: latitude decreases
    currentLat -= (distStep / 111320);

    const gaitPhase = (12.0 + t) * 2.0 * Math.PI * 2.0;
    fusionRuntime.processImuSample(
      { x: 0.4 * Math.sin(gaitPhase), y: 0.5 * Math.abs(Math.sin(gaitPhase)), z: 9.81 + 1.2 * Math.sin(gaitPhase) },
      { x: 0.05 * Math.sin(gaitPhase), y: 0.05 * Math.cos(gaitPhase), z: 0.02 * Math.sin(gaitPhase) },
      currentTimeMs
    );

    if (Math.round(t * 20) % 20 === 0) {
      fusionRuntime.updateGnss({
        latitude: currentLat,
        longitude: currentLon,
        accuracy: 4.2,
        speed: walkingSpeedMps,
        heading: 180.0,
        timestamp: currentTimeMs,
      });
    }

    const state = fusionRuntime.getLatestState();
    if (state && t > 2.0) {
      const vx = state.velocity.x;
      const vy = state.velocity.y;
      const speed = Math.sqrt(vx * vx + vy * vy) * 3.6;

      let velHeadingDeg = (Math.atan2(vx, vy) * (180 / Math.PI)) % 360;
      if (velHeadingDeg < 0) velHeadingDeg += 360;
      velHeadingDeg = Math.round(velHeadingDeg * 10) / 10;

      const hdgDiff = Math.abs(state.heading - 180.0);
      leg3Logs.push({
        heading: state.heading,
        velHdg: velHeadingDeg,
        speedKmh: speed,
        diff: hdgDiff
      });
    }
  }

  const avgLeg3Hdg = leg3Logs.reduce((a, b) => a + b.heading, 0) / leg3Logs.length;
  const avgLeg3VelHdg = leg3Logs.reduce((a, b) => a + b.velHdg, 0) / leg3Logs.length;
  const avgLeg3Speed = leg3Logs.reduce((a, b) => a + b.speedKmh, 0) / leg3Logs.length;
  console.log(`Leg 3 (South) Result: Logged Heading = ${avgLeg3Hdg.toFixed(1)}° (Expected: 180.0°), VelVector Heading = ${avgLeg3VelHdg.toFixed(1)}°, Speed = ${avgLeg3Speed.toFixed(2)} km/h`);

  console.log('\n=== Test 2 Summary & Checks ===');
  const passedLeg1 = Math.abs(avgLeg1Hdg - 90.0) <= 2.0 && Math.abs(avgLeg1VelHdg - 90.0) <= 2.0;
  const passedTurn = Math.abs(postTurnState?.heading! - 180.0) <= 3.0;
  const passedLeg3 = Math.abs(avgLeg3Hdg - 180.0) <= 2.0 && Math.abs(avgLeg3VelHdg - 180.0) <= 2.0;
  const passedSpeed = avgLeg1Speed >= 5.0 && avgLeg3Speed >= 5.0;

  console.log(`1. Leg 1 (East 90°) Heading Alignment: ${passedLeg1 ? 'PASSED' : 'FAILED'} (Hdg: ${avgLeg1Hdg.toFixed(1)}°, VelHdg: ${avgLeg1VelHdg.toFixed(1)}°)`);
  console.log(`2. 90° Turn Tracking: ${passedTurn ? 'PASSED' : 'FAILED'} (Turned to ${postTurnState?.heading.toFixed(1)}°)`);
  console.log(`3. Leg 3 (South 180°) Heading Alignment: ${passedLeg3 ? 'PASSED' : 'FAILED'} (Hdg: ${avgLeg3Hdg.toFixed(1)}°, VelHdg: ${avgLeg3VelHdg.toFixed(1)}°)`);
  console.log(`4. Sustained Brisk Walk Speed: ${passedSpeed ? 'PASSED' : 'FAILED'} (~${avgLeg1Speed.toFixed(2)} km/h)`);

  if (passedLeg1 && passedTurn && passedLeg3 && passedSpeed) {
    console.log('\nTEST 2 (HEADING VS VELOCITY VECTOR ALIGNMENT AT BRISK WALKING PACE) PASSED!');
  } else {
    throw new Error('Test 2 failed.');
  }
}

runBriskWalkHeadingVerification().catch((e) => {
  console.error(e);
  process.exit(1);
});
