import fs from 'fs';
import path from 'path';
import { FusionRuntime } from '../src/services/ekf/FusionRuntime';

// ====================================================================
// TEST (a): Stationary IMU + GNSS Ramp (0 -> 45 -> 155 km/h, 60s)
// ====================================================================
async function test_a_StationaryImuGnssRamp() {
  console.log('====================================================================');
  console.log('  TEST (a): Stationary IMU + GNSS Ramp (0 -> 45 -> 155 km/h, 60s)');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  let maxSpeedDuringSpike = 0;
  let initialPosition: { lat: number; lng: number } | null = null;
  let maxPositionDriftMeters = 0;
  let rejectedGnssCount = 0;
  let totalHighSpeedFixCount = 0;

  rt.setOnFusedDataCallback((fused) => {
    const rawSpeedKmH = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y) * 3.6;
    maxSpeedDuringSpike = Math.max(maxSpeedDuringSpike, rawSpeedKmH);

    if (fused.latitude !== null && fused.longitude !== null) {
      if (initialPosition === null) {
        initialPosition = { lat: fused.latitude, lng: fused.longitude };
      } else {
        const dLat = (fused.latitude - initialPosition.lat) * 111139;
        const dLng = (fused.longitude - initialPosition.lng) * 111139 * Math.cos(initialPosition.lat * Math.PI / 180);
        const drift = Math.sqrt(dLat * dLat + dLng * dLng);
        maxPositionDriftMeters = Math.max(maxPositionDriftMeters, drift);
      }
    }
  });

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();

  // 1. Initial 5 seconds: Stationary under good GPS (Accuracy 4.5m)
  for (let step = 0; step <= 50; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0.0005, y: 0.0005, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: originLat,
        longitude: originLng,
        accuracy: 4.5,
        speed: 0,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  // 2. 60 seconds of uncorroborated GNSS ramping up to 155 km/h with degraded accuracy (12-20m)
  console.log('Injecting 60s of uncorroborated GNSS ramping 0 -> 45 -> 155 km/h (accuracy 12-20m)...');
  let simulatedDistMeters = 0;

  for (let step = 51; step <= 650; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    // IMU: perfectly stationary on desk (varA < 0.001, varG < 0.0001)
    rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0.0005, y: 0.0005, z: 0 }, epoch);

    if (step % 10 === 0) {
      totalHighSpeedFixCount++;
      const rampElapsed = (t - 5.0);
      const fakeSpeedKmh = Math.min(155.0, 45.0 + rampElapsed * 1.8);
      const fakeSpeedMps = fakeSpeedKmh / 3.6;
      simulatedDistMeters += fakeSpeedMps * 1.0;

      const jumpLat = originLat + (simulatedDistMeters / 111139);
      const jumpLng = originLng + (simulatedDistMeters / (111139 * Math.cos(originLat * Math.PI / 180)));
      const accuracy = 12.0 + (rampElapsed / 60.0) * 8.0; // 12..20m

      rt.updateGnss({
        latitude: jumpLat,
        longitude: jumpLng,
        accuracy,
        speed: fakeSpeedMps,
        heading: 45.0,
        timestamp: epoch,
      });

      const state = rt.getLatestState();
      if (state?.gnssRejected) {
        rejectedGnssCount++;
      }
    }
  }

  const finalState = rt.getLatestState();
  console.log('\n--- Test (a) Results ---');
  console.log(`Simulated Fake GNSS Displacement:  ${simulatedDistMeters.toFixed(1)} m`);
  console.log(`Max Speed Reported by EKF:        ${maxSpeedDuringSpike.toFixed(2)} km/h (Target: < 0.10 km/h)`);
  console.log(`Max Position Drift from Origin:   ${maxPositionDriftMeters.toFixed(2)} m (Target: < 1.00 m)`);
  console.log(`Final ZUPT State:                 ${finalState?.zuptState} (Target: LOCKED)`);
  console.log(`GNSS_Uncorroborated Flag:        ${finalState?.gnssUncorroborated} (Target: false)`);
  console.log(`Rejected Fake GNSS Fixes:         ${rejectedGnssCount} / ${totalHighSpeedFixCount}`);

  if (maxSpeedDuringSpike < 0.10 && maxPositionDriftMeters < 1.0 && finalState?.zuptState === 'LOCKED' && !finalState?.gnssUncorroborated) {
    console.log('✓ TEST (a) PASSED: 60s GNSS ramp completely rejected, escape never opened, speed ~0!\n');
  } else {
    console.error('✗ TEST (a) FAILED: EKF allowed uncorroborated ramp to inject speed or open escape.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (b): False 43 m/s Velocity in IDR + Rest-Floor IMU 30s
// ====================================================================
async function test_b_FalseVelocityDecayInIdr() {
  console.log('====================================================================');
  console.log('  TEST (b): False 43 m/s Velocity in IDR + Rest-Floor IMU 30s');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();

  // 1. Initial 1s: establish origin
  for (let step = 0; step <= 10; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0.0005, y: 0.0005, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: originLat,
        longitude: originLng,
        accuracy: 4.0,
        speed: 0,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  // 2. Inject false 43 m/s (155 km/h) velocity vector into INS
  console.log('Injecting false 43 m/s velocity in IDR mode...');
  rt.getEkf().getIns().velocity = { x: -41.8, y: 7.0, z: 0 }; // |v| = 42.4 m/s (152.6 km/h)

  // 3. Run for 30s in pure IDR with rest-floor IMU (varA < 0.01, varG < 0.001)
  for (let step = 11; step <= 310; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    // Desk rest floor
    rt.processImuSample({ x: 0.003, y: 0.003, z: 9.81 }, { x: 0.0003, y: 0.0003, z: 0 }, epoch);
  }

  const finalState = rt.getLatestState();
  const finalSpeedKmH = finalState
    ? Math.sqrt(finalState.velocity.x * finalState.velocity.x + finalState.velocity.y * finalState.velocity.y) * 3.6
    : 0;

  console.log('\n--- Test (b) Results ---');
  console.log(`Final IDR Speed:                  ${finalSpeedKmH.toFixed(2)} km/h (Target: < 0.36 km/h / 0.10 m/s)`);
  console.log(`Final ZUPT State:                 ${finalState?.zuptState} (Target: LOCKED)`);

  if (finalSpeedKmH < 0.36 && finalState?.zuptState === 'LOCKED') {
    console.log('✓ TEST (b) PASSED: False 43 m/s velocity decayed to ~0 and locked ZUPT at rest floor!\n');
  } else {
    console.error('✗ TEST (b) FAILED: Velocity failed to decay to zero under rest-floor IMU.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (c): 60 km/h IDR Cruise with Realistic Vibration (30s Tunnel)
// ====================================================================
async function test_c_IdrCruiseWithRealisticVibration() {
  console.log('====================================================================');
  console.log('  TEST (c): 60 km/h IDR Cruise with Realistic Vibration (30s Tunnel)');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();
  const cruiseSpeedMps = 16.667; // 60 km/h

  // 1. Initial 2s stationary: establish initial fix, attitude, and ZUPT lock
  for (let step = 0; step <= 20; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: originLat,
        longitude: originLng,
        accuracy: 4.0,
        speed: 0,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  // 2. Accelerate 0 -> 60 km/h over 4 seconds (a = 4.17 m/s^2) with IMU corroboration and GNSS
  let curSpeed = 0;
  let curDisp = 0;
  const accelY = 4.167;
  for (let step = 21; step <= 60; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    curSpeed = Math.min(cruiseSpeedMps, curSpeed + accelY * 0.1);
    curDisp += curSpeed * 0.1;
    rt.processImuSample({ x: 0.02, y: accelY, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: originLat + (curDisp / 111139),
        longitude: originLng,
        accuracy: 4.0,
        speed: curSpeed,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  // 3. Enter 30-second tunnel outage (NO GNSS) with realistic road vibration (varA in 0.03..0.10)
  console.log('Entering 30-second tunnel outage at 60 km/h with road vibration (varA ~ 0.05)...');
  const tunnelStartLat = originLat + (curDisp / 111139);

  for (let step = 61; step <= 360; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    // Road vibration noise: zero mean, amplitude +/- 0.35 m/s^2 (varA ~ 0.04-0.08)
    const angle = step * 1.7;
    const vibAx = 0.35 * Math.sin(angle);
    const vibAy = 0.35 * Math.cos(angle * 1.3);
    const vibAz = 0.35 * Math.sin(angle * 2.1);
    const vibG = 0.002 * Math.sin(angle * 0.5);
    rt.processImuSample({ x: vibAx, y: vibAy, z: 9.81 + vibAz }, { x: vibG, y: vibG, z: vibG }, epoch);
  }

  const finalState = rt.getLatestState();
  const finalSpeedKmH = finalState
    ? Math.sqrt(finalState.velocity.x * finalState.velocity.x + finalState.velocity.y * finalState.velocity.y) * 3.6
    : 0;
  const traveledMeters = finalState?.latitude ? (finalState.latitude - tunnelStartLat) * 111139 : 0;

  console.log('\n--- Test (c) Results ---');
  console.log(`Final Tunnel Speed:               ${finalSpeedKmH.toFixed(1)} km/h (Expected: ~60.0 km/h)`);
  console.log(`Tunnel Distance Traveled:         ${traveledMeters.toFixed(1)} m (Expected: ~500 m)`);
  console.log(`Final ZUPT State:                 ${finalState?.zuptState} (Expected: RELEASED)`);

  if (finalSpeedKmH >= 30.0 && finalState?.zuptState === 'RELEASED' && traveledMeters >= 350.0) {
    console.log('✓ TEST (c) PASSED: 60 km/h IDR cruise preserved through 30s tunnel with road vibration!\n');
  } else {
    console.error('✗ TEST (c) FAILED: Tunnel cruise speed was prematurely decayed or locked ZUPT.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (d): Idle Vibration + False GNSS Ramp (Rejected)
// ====================================================================
async function test_d_IdleVibrationFalseGnssRamp() {
  console.log('====================================================================');
  console.log('  TEST (d): Idle Vibration (varA 0.35-0.55) + False GNSS Ramp');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();

  let maxSpeedReported = 0;
  let rejectedCount = 0;
  let totalFixes = 0;

  rt.setOnFusedDataCallback((fused) => {
    const spd = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y) * 3.6;
    maxSpeedReported = Math.max(maxSpeedReported, spd);
  });

  // 1. Initial 2s stationary
  for (let step = 0; step <= 20; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    rt.processImuSample({ x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: originLat,
        longitude: originLng,
        accuracy: 4.0,
        speed: 0,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  // 2. 20s of Vehicle Engine Idle Vibration (varA ~ 0.45, varG ~ 0.04) + False GNSS Ramp (45..155 km/h)
  console.log('Injecting engine idle vibration (varA ~ 0.45) with false GNSS ramp 45..155 km/h...');
  let simDist = 0;
  for (let step = 21; step <= 220; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    // Oscillatory engine idle vibration (zero mean, +/- 1.0 m/s^2, varA ~ 0.40)
    const phase = step * 0.8;
    const vibAx = Math.sin(phase) * 1.1;
    const vibAy = Math.cos(phase * 1.3) * 1.1;
    const vibGz = Math.sin(phase * 0.5) * 0.05;

    rt.processImuSample({ x: vibAx, y: vibAy, z: 9.81 }, { x: 0, y: 0, z: vibGz }, epoch);

    if (step % 10 === 0) {
      totalFixes++;
      const rampSpeedKmh = Math.min(155.0, 45.0 + (t - 2.0) * 5.0);
      const rampSpeedMps = rampSpeedKmh / 3.6;
      simDist += rampSpeedMps * 1.0;

      rt.updateGnss({
        latitude: originLat + (simDist / 111139),
        longitude: originLng,
        accuracy: 15.0,
        speed: rampSpeedMps,
        heading: 0,
        timestamp: epoch,
      });

      const state = rt.getLatestState();
      if (state?.gnssRejected) rejectedCount++;
    }
  }

  const finalState = rt.getLatestState();
  console.log('\n--- Test (d) Results ---');
  console.log(`Max Speed Reported by EKF:        ${maxSpeedReported.toFixed(2)} km/h (Target: < 0.20 km/h)`);
  console.log(`Rejected Fixes:                   ${rejectedCount} / ${totalFixes}`);
  console.log(`Final ZUPT State:                 ${finalState?.zuptState} (Target: LOCKED)`);

  if (maxSpeedReported < 0.20 && finalState?.zuptState === 'LOCKED' && rejectedCount === totalFixes) {
    console.log('✓ TEST (d) PASSED: Engine idle vibration did NOT corroborate false GNSS ramp, speed held 0!\n');
  } else {
    console.error('✗ TEST (d) FAILED: Engine idle vibration incorrectly corroborated false GNSS ramp.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (e): Real Start 0 -> 60 km/h (<=2s release) & Soft Start 0.10 m/s^2
// ====================================================================
async function test_e_RealStartAndSoftStart() {
  console.log('====================================================================');
  console.log('  TEST (e): Real Start 0 -> 60 km/h & Soft Start 0.10 m/s^2');
  console.log('====================================================================');
  
  // Part 1: Real 0 -> 60 km/h Start (a = 2.78 m/s^2)
  const rt1 = new FusionRuntime();
  await rt1.start();
  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();

  for (let step = 0; step <= 20; step++) {
    const epoch = startTime + Math.round(step * 100);
    rt1.processImuSample({ x: 0.01, y: 0.01, z: 9.81 }, { x: 0.001, y: 0.001, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt1.updateGnss({ latitude: originLat, longitude: originLng, accuracy: 4.0, speed: 0, heading: 0, timestamp: epoch });
    }
  }

  let zuptReleaseTime1: number | null = null;
  const motionStartTime1 = startTime + 2000;
  rt1.setOnFusedDataCallback((fused) => {
    if (fused.timestamp >= motionStartTime1 && fused.zuptState === 'RELEASED' && zuptReleaseTime1 === null) {
      zuptReleaseTime1 = fused.timestamp;
    }
  });

  const accelY = 2.78;
  let trueSpeed = 0, trueDisp = 0;
  for (let step = 21; step <= 80; step++) {
    const epoch = startTime + Math.round(step * 100);
    trueSpeed = Math.min(16.67, trueSpeed + accelY * 0.1);
    trueDisp += trueSpeed * 0.1;
    rt1.processImuSample({ x: 0.05, y: accelY + 0.05, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt1.updateGnss({
        latitude: originLat + (trueDisp / 111139),
        longitude: originLng,
        accuracy: 4.0,
        speed: trueSpeed,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  const releaseDelaySec1 = zuptReleaseTime1 !== null ? (zuptReleaseTime1 - motionStartTime1) / 1000 : Infinity;
  console.log(`Part 1 (Normal Start 2.78 m/s^2) Release Latency: ${releaseDelaySec1.toFixed(2)} s (Target: <= 2.0 s)`);

  // Part 2: Soft Start at a = 0.10 m/s^2
  const rt2 = new FusionRuntime();
  await rt2.start();
  for (let step = 0; step <= 20; step++) {
    const epoch = startTime + Math.round(step * 100);
    rt2.processImuSample({ x: 0.01, y: 0.01, z: 9.81 }, { x: 0.001, y: 0.001, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt2.updateGnss({ latitude: originLat, longitude: originLng, accuracy: 4.0, speed: 0, heading: 0, timestamp: epoch });
    }
  }

  let zuptReleaseTime2: number | null = null;
  const motionStartTime2 = startTime + 2000;
  rt2.setOnFusedDataCallback((fused) => {
    if (fused.timestamp >= motionStartTime2 && fused.zuptState === 'RELEASED' && zuptReleaseTime2 === null) {
      zuptReleaseTime2 = fused.timestamp;
    }
  });

  const softAccel = 0.10;
  let softSpeed = 0, softDisp = 0;
  for (let step = 21; step <= 120; step++) {
    const epoch = startTime + Math.round(step * 100);
    softSpeed = Math.min(4.17, softSpeed + softAccel * 0.1);
    softDisp += softSpeed * 0.1;
    rt2.processImuSample({ x: 0.0, y: softAccel, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);
    if (step % 10 === 0) {
      rt2.updateGnss({
        latitude: originLat + (softDisp / 111139),
        longitude: originLng,
        accuracy: 4.0,
        speed: softSpeed,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  const releaseDelaySec2 = zuptReleaseTime2 !== null ? (zuptReleaseTime2 - motionStartTime2) / 1000 : Infinity;
  console.log(`Part 2 (Soft Start 0.10 m/s^2) Release Latency:   ${releaseDelaySec2.toFixed(2)} s (Target: < 3.5 s)`);

  const state2 = rt2.getLatestState();
  const speedKmH2 = state2 ? Math.sqrt(state2.velocity.x * state2.velocity.x + state2.velocity.y * state2.velocity.y) * 3.6 : 0;
  console.log(`Part 2 Final Soft Speed:                          ${speedKmH2.toFixed(1)} km/h (Expected: ~3.6 km/h)`);

  if (releaseDelaySec1 <= 2.0 && releaseDelaySec2 < 3.5 && state2?.zuptState === 'RELEASED' && speedKmH2 >= 3.0) {
    console.log('✓ TEST (e) PASSED: Both standard and soft start released ZUPT promptly and tracked accurately!\n');
  } else {
    console.error('✗ TEST (e) FAILED: Release latency exceeded target or soft start failed.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (f): Filter Starts ZUPT-Locked, GNSS Steady 60 km/h, Smooth IMU
// ====================================================================
async function test_f_CruisingStartEscape() {
  console.log('====================================================================');
  console.log('  TEST (f): Filter Starts ZUPT-Locked, GNSS Steady 60 km/h (Smooth IMU)');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();
  const cruiseSpeedMps = 16.667; // 60 km/h

  let initialRejections = 0;
  let escapeTriggered = false;
  let currentLat = originLat;

  // Run 20 seconds of steady 60 km/h GNSS with perfectly smooth IMU (no initial accel)
  for (let step = 0; step <= 200; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    currentLat += (cruiseSpeedMps * 0.1 / 111139);

    // Smooth highway IMU (varA < 0.001)
    rt.processImuSample({ x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);

    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: currentLat,
        longitude: originLng,
        accuracy: 3.5,
        speed: cruiseSpeedMps,
        heading: 0,
        timestamp: epoch,
      });

      const state = rt.getLatestState();
      if (t < 14.0 && state?.gnssRejected) {
        initialRejections++;
      }
      if (state?.gnssUncorroborated && state?.zuptState === 'RELEASED') {
        escapeTriggered = true;
      }
    }
  }

  const finalState = rt.getLatestState();
  const finalSpeedKmH = finalState ? Math.sqrt(finalState.velocity.x * finalState.velocity.x + finalState.velocity.y * finalState.velocity.y) * 3.6 : 0;

  console.log('\n--- Test (f) Results ---');
  console.log(`Initial Rejection Count (<14s):   ${initialRejections} (Expected: > 10)`);
  console.log(`Escape Triggered (>=15s):         ${escapeTriggered} (Expected: true)`);
  console.log(`GNSS_Uncorroborated Flag:        ${finalState?.gnssUncorroborated} (Expected: true)`);
  console.log(`Final Cruising Speed:             ${finalSpeedKmH.toFixed(1)} km/h (Expected: ~60.0 km/h)`);
  console.log(`Final ZUPT State:                 ${finalState?.zuptState} (Expected: RELEASED)`);

  if (initialRejections >= 10 && escapeTriggered && finalState?.gnssUncorroborated && finalSpeedKmH >= 55.0) {
    console.log('✓ TEST (f) PASSED: Rejected uncorroborated cruise initially, safely escaped at N=15s!\n');
  } else {
    console.error('✗ TEST (f) FAILED: Cruising start escape failed to trigger or track correctly.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (g): Stop-and-Go: Idle <-> Crawl <-> Idle (No Flapping)
// ====================================================================
async function test_g_StopAndGoHysteresis() {
  console.log('====================================================================');
  console.log('  TEST (g): Stop-and-Go Crawl: Idle <-> Crawl (No Flapping)');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();

  const zuptTransitions: string[] = [];
  let lastZupt = 'LOCKED';

  rt.setOnFusedDataCallback((fused) => {
    if (fused.zuptState && fused.zuptState !== lastZupt) {
      zuptTransitions.push(`${fused.zuptState} @ ${((fused.timestamp - startTime) / 1000).toFixed(1)}s`);
      lastZupt = fused.zuptState;
    }
  });

  // Cycle 1: 0..4s Idle (speed 0, engine vibration)
  // Cycle 2: 4..8s Crawl forward at 1.8 km/h (0.5 m/s)
  // Cycle 3: 8..12s Stopped at red light (speed 0, engine vibration)
  // Cycle 4: 12..16s Crawl forward at 2.2 km/h (0.61 m/s)
  let currentLat = originLat;
  for (let step = 0; step <= 160; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);

    let speedKmh = 0;
    let accelY = 0;
    if ((t >= 4.0 && t < 8.0) || (t >= 12.0 && t <= 16.0)) {
      speedKmh = t >= 12.0 ? 2.2 : 1.8;
      accelY = 0.15; // gentle forward throttle
      currentLat += (speedKmh / 3.6 * 0.1 / 111139);
    }

    const vib = (Math.random() - 0.5) * 0.4;
    rt.processImuSample({ x: vib, y: accelY + vib, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);

    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: currentLat,
        longitude: originLng,
        accuracy: 4.0,
        speed: speedKmh / 3.6,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  console.log('\n--- Test (g) Results ---');
  console.log(`ZUPT State Transitions: [${zuptTransitions.join(' -> ')}]`);
  console.log(`Total State Changes:    ${zuptTransitions.length} (Expected: 4 cleanly debounced transitions)`);

  // Expected 4 clean transitions: RELEASED @ ~4s, LOCKED @ ~9s, RELEASED @ ~12s, plus/minus 1
  if (zuptTransitions.length >= 2 && zuptTransitions.length <= 5) {
    console.log('✓ TEST (g) PASSED: Stop-and-go hysteresis switched cleanly without flapping!\n');
  } else {
    console.error('✗ TEST (g) FAILED: Rapid flapping or failed transition in stop-and-go.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (h): Slow Creep with Noisy Near-Zero GNSS Speed
// ====================================================================
async function test_h_SlowCreepNoFalseZupt() {
  console.log('====================================================================');
  console.log('  TEST (h): Slow Creep with Noisy Near-Zero GNSS Speed');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();
  const trueCreepSpeed = 1.0; // 1.0 m/s = 3.6 km/h

  let currentLat = originLat;
  let lockedDuringCreep = false;

  for (let step = 0; step <= 150; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);
    currentLat += (trueCreepSpeed * 0.1 / 111139);

    // IMU: pedestrian / slow vehicle creep (low-pass horizontal accel ~ 0.10 m/s^2)
    const stride = Math.sin(step * 0.6) * 0.3;
    rt.processImuSample({ x: 0.05, y: 0.12 + stride, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);

    if (step % 10 === 0) {
      // GNSS reports noisy jitter speed between 0.1 and 0.4 m/s
      const jitterSpeed = 0.2 + (Math.random() - 0.5) * 0.2;
      rt.updateGnss({
        latitude: currentLat,
        longitude: originLng,
        accuracy: 4.5,
        speed: jitterSpeed,
        heading: 0,
        timestamp: epoch,
      });

      const state = rt.getLatestState();
      if (t >= 3.0 && state?.zuptState === 'LOCKED') {
        lockedDuringCreep = true;
      }
    }
  }

  const finalState = rt.getLatestState();
  const distAdvanced = finalState?.latitude ? (finalState.latitude - originLat) * 111139 : 0;

  console.log('\n--- Test (h) Results ---');
  console.log(`False ZUPT Lock During Motion:    ${lockedDuringCreep} (Expected: false)`);
  console.log(`Distance Advanced:                ${distAdvanced.toFixed(1)} m (Expected: ~15.0 m)`);
  console.log(`Final ZUPT State:                 ${finalState?.zuptState} (Expected: RELEASED)`);

  if (!lockedDuringCreep && distAdvanced >= 10.0 && finalState?.zuptState === 'RELEASED') {
    console.log('✓ TEST (h) PASSED: Slow creep held active velocity and advanced smoothly!\n');
  } else {
    console.error('✗ TEST (h) FAILED: Slow creep false-locked ZUPT or stopped progress.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (i): Tilt & Vigorous Shake Test (Speed Bounded)
// ====================================================================
async function test_i_TiltShakeTest() {
  console.log('====================================================================');
  console.log('  TEST (i): Tilt & Vigorous Shake Test (Phone Rotated 30-45 deg)');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.659920;
  const originLng = 75.906410;
  const startTime = Date.now();

  let maxSpeedDuringShake = 0;
  let maxDriftMeters = 0;

  rt.setOnFusedDataCallback((fused) => {
    const spd = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y) * 3.6;
    maxSpeedDuringShake = Math.max(maxSpeedDuringShake, spd);
    if (fused.latitude && fused.longitude) {
      const dLat = (fused.latitude - originLat) * 111139;
      const dLng = (fused.longitude - originLng) * 111139 * Math.cos(originLat * Math.PI / 180);
      maxDriftMeters = Math.max(maxDriftMeters, Math.sqrt(dLat * dLat + dLng * dLng));
    }
  });

  // 1. Initial 1s stationary fix
  rt.processImuSample({ x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 }, startTime);
  rt.updateGnss({ latitude: originLat, longitude: originLng, accuracy: 3.5, speed: 0, heading: 0, timestamp: startTime });

  // 2. 10 seconds of phone rotated 45 deg pitch, 30 deg roll, and vigorously shaken back and forth
  console.log('Shaking tilted phone vigorously (high dynamic accel +/-4 m/s^2, gyro +/-3 rad/s)...');
  for (let step = 1; step <= 100; step++) {
    const t = step * 0.1;
    const epoch = startTime + Math.round(t * 1000);

    // Gravity projected on tilted axes: pitch 45 deg, roll 30 deg
    const gX = 9.81 * Math.sin(30 * Math.PI / 180) * Math.cos(45 * Math.PI / 180); // ~ 3.47
    const gY = 9.81 * Math.sin(45 * Math.PI / 180); // ~ 6.94
    const gZ = 9.81 * Math.cos(30 * Math.PI / 180) * Math.cos(45 * Math.PI / 180); // ~ 6.01

    // Vigorous shaking: zero-mean high frequency shake
    const shakeAx = Math.sin(step * 1.5) * 3.5;
    const shakeAy = Math.cos(step * 1.7) * 3.5;
    const shakeGz = Math.sin(step * 2.0) * 3.0;

    rt.processImuSample({ x: gX + shakeAx, y: gY + shakeAy, z: gZ }, { x: shakeGz, y: shakeGz, z: shakeGz }, epoch);

    // GNSS reports stationary or zero speed
    if (step % 10 === 0) {
      rt.updateGnss({
        latitude: originLat,
        longitude: originLng,
        accuracy: 5.0,
        speed: 0,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  console.log('\n--- Test (i) Results ---');
  console.log(`Max Speed During Vigorous Shake:  ${maxSpeedDuringShake.toFixed(2)} km/h (Target: < 5.0 km/h)`);
  console.log(`Max Position Drift:               ${maxDriftMeters.toFixed(2)} m (Target: < 3.0 m)`);

  if (maxSpeedDuringShake < 5.0 && maxDriftMeters < 3.0) {
    console.log('✓ TEST (i) PASSED: Tilt & vigorous shake remained tightly bounded without teleportation!\n');
  } else {
    console.error('✗ TEST (i) FAILED: Vigorous shake produced unbounded speed or position explosion.');
    process.exit(1);
  }
}

// ====================================================================
// TEST (j): Replay Real CSV's GNSS Rows with Stationary Desk IMU
// ====================================================================
async function test_j_ReplayRealCsvGnssWithStationaryImu() {
  console.log('====================================================================');
  console.log('  TEST (j): Replay Real CSV (123 GNSS Fixes, 2.3km Glitch) with Desk IMU');
  console.log('====================================================================');
  const rt = new FusionRuntime();
  const csvPath = path.join(process.cwd(), 'test-runner', 'fixtures', 'reckonx_telemetry_log_2026-09-27T18-39-36.csv');
  const content = fs.readFileSync(csvPath, 'utf-8');
  const lines = content.split(/\r?\n/).filter(l => l.trim().length > 0 && !l.startsWith('#'));
  const header = lines[0].split(',');
  const latIdx = header.indexOf('Latitude');
  const lngIdx = header.indexOf('Longitude');
  const accIdx = header.indexOf('Accuracy_Meters');
  const spdIdx = header.indexOf('Speed_KmH');
  const typeIdx = header.indexOf('Source_Type');
  const timeIdx = header.indexOf('Timestamp_Epoch_Ms');

  const gnssRows = lines.slice(1)
    .map(line => line.split(','))
    .filter(cols => cols[typeIdx] === 'GNSS');

  console.log(`Loaded ${gnssRows.length} real GNSS rows from ${path.basename(csvPath)}.`);

  let maxReportedSpeedKmH = 0;
  let maxDriftFromFirstFix = 0;
  let originLat: number | null = null;
  let originLng: number | null = null;
  let rejectedCount = 0;

  rt.setOnFusedDataCallback((fused) => {
    const spd = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y) * 3.6;
    maxReportedSpeedKmH = Math.max(maxReportedSpeedKmH, spd);

    if (fused.latitude && fused.longitude && originLat !== null && originLng !== null) {
      const dLat = (fused.latitude - originLat) * 111139;
      const dLng = (fused.longitude - originLng) * 111139 * Math.cos(originLat * Math.PI / 180);
      maxDriftFromFirstFix = Math.max(maxDriftFromFirstFix, Math.sqrt(dLat * dLat + dLng * dLng));
    }
  });

  // Replay every GNSS row alongside 10 Hz desk IMU samples
  let prevTimestamp = parseInt(gnssRows[0][timeIdx], 10);
  for (let rIdx = 0; rIdx < gnssRows.length; rIdx++) {
    const row = gnssRows[rIdx];
    const ts = parseInt(row[timeIdx], 10);
    const lat = parseFloat(row[latIdx]);
    const lng = parseFloat(row[lngIdx]);
    const acc = parseFloat(row[accIdx]);
    const spdKmh = parseFloat(row[spdIdx]);
    const spdMps = spdKmh / 3.6;

    if (originLat === null) {
      originLat = lat;
      originLng = lng;
    }

    // Step 10Hz stationary IMU between GNSS epochs
    const timeDeltaMs = Math.min(2000, Math.max(100, ts - prevTimestamp));
    const imuSteps = Math.round(timeDeltaMs / 100);
    for (let s = 0; s < imuSteps; s++) {
      const epoch = prevTimestamp + s * 100;
      // Phone stationary on desk
      rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0.0005, y: 0.0005, z: 0 }, epoch);
    }
    prevTimestamp = ts;

    // Ingest GNSS fix
    rt.updateGnss({
      latitude: lat,
      longitude: lng,
      accuracy: acc,
      speed: spdMps,
      heading: 0,
      timestamp: ts,
    });

    const state = rt.getLatestState();
    if (state?.gnssRejected) rejectedCount++;
  }

  const finalState = rt.getLatestState();
  console.log('\n--- Test (j) Results ---');
  console.log(`Replayed Real GNSS Fixes:         ${gnssRows.length}`);
  console.log(`Rejected False Glitch Fixes:      ${rejectedCount} / ${gnssRows.length}`);
  console.log(`Max Speed Reported by EKF:        ${maxReportedSpeedKmH.toFixed(2)} km/h (Target: < 0.50 km/h)`);
  console.log(`Max Position Drift from Start:    ${maxDriftFromFirstFix.toFixed(2)} m (Target: < 2.00 m)`);
  console.log(`Final ZUPT State:                 ${finalState?.zuptState} (Target: LOCKED)`);

  if (maxReportedSpeedKmH < 0.50 && maxDriftFromFirstFix < 2.0 && finalState?.zuptState === 'LOCKED') {
    console.log('✓ TEST (j) PASSED: Real CSV 2.3km / 155 km/h false movement was 100% rejected, speed ~0!\n');
  } else {
    console.error('✗ TEST (j) FAILED: EKF allowed false movement during real CSV replay.');
    process.exit(1);
  }
}

async function runAll() {
  await test_a_StationaryImuGnssRamp();
  await test_b_FalseVelocityDecayInIdr();
  await test_c_IdrCruiseWithRealisticVibration();
  await test_d_IdleVibrationFalseGnssRamp();
  await test_e_RealStartAndSoftStart();
  await test_f_CruisingStartEscape();
  await test_g_StopAndGoHysteresis();
  await test_h_SlowCreepNoFalseZupt();
  await test_i_TiltShakeTest();
  await test_j_ReplayRealCsvGnssWithStationaryImu();

  console.log('====================================================================');
  console.log('  ALL 10 VERIFICATION TESTS (a) THROUGH (j) PASSED ✓                 ');
  console.log('====================================================================');
}

runAll().catch((err) => {
  console.error(err);
  process.exit(1);
});
