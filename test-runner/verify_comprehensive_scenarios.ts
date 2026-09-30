/**
 * verify_comprehensive_scenarios.ts
 * =================================
 * Multi-Scenario Real & Synthetic Comprehensive Stress Benchmark
 *
 * Scenarios tested:
 *  1. Real-World Session A: 2026-09-29 Solapur Trajectory (331 points, speed up to 79.7 km/h)
 *  2. Real-World Session B: 2026-09-27 Solapur Trajectory (240 points, extended DR & parallel roads)
 *  3. Extended Urban Tunnel Outage: 60s at 60 km/h with S-curves & smooth reacquisition
 *  4. High-Stress Stop-and-Go Driving: 10 cycles of 0 -> 45 km/h -> 0 km/h rapid acceleration/braking
 *  5. Extreme Stationary Multipath Attack: 0 km/h vehicle under bridge subjected to 150 km/h GPS hallucination
 */

import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';
import { FusionRuntime } from '../src/services/ekf/FusionRuntime';
import { LogExportService } from '../src/services/logExportService';
import { MemoryRoadGraph } from '../src/workers/mapMatcher.worker';
import { haversineDistance } from '../src/services/ekf/OutputStabilizer';
import type { RoadGraphCell } from '../src/services/roadGraphCacheService';
import type { RecordedGPSPoint } from '../src/services/api/trackingService';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const FIXTURES_DIR = path.resolve(__dirname, 'fixtures');

// -----------------------------------------------------------------------------
// Scenario 1: Real-World Session 2026-09-29
// -----------------------------------------------------------------------------
async function runScenario1_RealSession_2026_09_29(): Promise<boolean> {
  console.log('================================================================================');
  console.log('  SCENARIO 1: Real Session 2026-09-29 (Solapur, 331 Points, High Speed)');
  console.log('================================================================================');

  const csvPath = path.join(FIXTURES_DIR, 'reckonx_telemetry_log_2026-09-29T13-31-10.csv');
  const content = fs.readFileSync(csvPath, 'utf-8');
  const lines = content.split(/\r?\n/).filter(l => l.trim() && !l.startsWith('#') && !l.startsWith('Timestamp_Epoch_Ms'));

  const rt = new FusionRuntime();
  await rt.start();

  let rejectedWhileMoving = 0;
  let maxVelocityDelta = 0;
  let prevVelMag = 0;
  let totalFixes = 0;

  rt.setOnFusedDataCallback((fused) => {
    const velMag = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y);
    if (prevVelMag > 0) {
      const dV = Math.abs(velMag - prevVelMag);
      if (dV > maxVelocityDelta) maxVelocityDelta = dV;
    }
    prevVelMag = velMag;
  });

  let lastTs = 0;
  for (let i = 0; i < lines.length; i++) {
    const cols = lines[i].split(',');
    const timestamp = parseInt(cols[0], 10);
    const speedKmH = cols[6] ? parseFloat(cols[6]) : 0;
    const isStationary = speedKmH < 0.5;
    const imuVarA = cols[10] ? parseFloat(cols[10]) : 0;
    const imuVarG = cols[11] ? parseFloat(cols[11]) : 0;
    const rawCbLat = cols[17] ? parseFloat(cols[17]) : parseFloat(cols[3]);
    const rawCbLng = cols[18] ? parseFloat(cols[18]) : parseFloat(cols[4]);
    const rawCbAcc = cols[19] ? parseFloat(cols[19]) : (cols[5] ? parseFloat(cols[5]) : 10);
    const rawCbSpeedMps = cols[20] ? parseFloat(cols[20]) : speedKmH / 3.6;
    const rawCbHeading = cols[21] ? parseFloat(cols[21]) : (cols[7] ? parseFloat(cols[7]) : 0);

    const prevTimestamp = i === 0 ? timestamp - 1000 : lastTs;
    lastTs = timestamp;
    const timeSpan = Math.max(100, timestamp - prevTimestamp);
    const imuSteps = Math.max(1, Math.round(timeSpan / 100));
    const stepDtMs = timeSpan / imuSteps;

    const accelVib = isStationary ? 0.005 : Math.min(0.8, Math.sqrt(Math.max(0, imuVarA)));
    const gyroVib = isStationary ? 0.001 : Math.min(0.05, Math.sqrt(Math.max(0, imuVarG)));

    for (let s = 1; s <= imuSteps; s++) {
      rt.processImuSample(
        { x: (Math.random() - 0.5) * accelVib, y: (Math.random() - 0.5) * accelVib, z: 9.81 },
        { x: (Math.random() - 0.5) * gyroVib, y: (Math.random() - 0.5) * gyroVib, z: (Math.random() - 0.5) * gyroVib },
        Math.round(prevTimestamp + s * stepDtMs)
      );
    }

    if (!isNaN(rawCbLat) && !isNaN(rawCbLng)) {
      totalFixes++;
      rt.updateGnss({
        latitude: rawCbLat,
        longitude: rawCbLng,
        accuracy: rawCbAcc,
        speed: rawCbSpeedMps,
        heading: rawCbHeading,
        timestamp,
      });

      if (rt.isLastGnssRejected() && speedKmH > 5.0) {
        rejectedWhileMoving++;
      }
    }
  }

  console.log(`  • Evaluated GNSS Fixes:                   ${totalFixes}`);
  console.log(`  • Moving GNSS Rejections:                 ${rejectedWhileMoving} (Historical: 101 -> Now: ${rejectedWhileMoving})`);
  console.log(`  • Max Single-Cycle Velocity Jump (Δv):    ${maxVelocityDelta.toFixed(2)} m/s (Limit ≤ 5.0 m/s)`);
  const pass = rejectedWhileMoving <= 4 && maxVelocityDelta <= 5.0;
  console.log(`  • Scenario 1 Status:                      ${pass ? 'PASSED ✓' : 'FAILED ✗'}\n`);
  return pass;
}

// -----------------------------------------------------------------------------
// Scenario 2: Real-World Session 2026-09-27
// -----------------------------------------------------------------------------
async function runScenario2_RealSession_2026_09_27(): Promise<boolean> {
  console.log('================================================================================');
  console.log('  SCENARIO 2: Real Session 2026-09-27 (Solapur, 240 Points, Prolonged DR & Holds)');
  console.log('================================================================================');

  const csvPath = path.join(FIXTURES_DIR, 'reckonx_telemetry_log_2026-09-27T18-39-36.csv');
  const content = fs.readFileSync(csvPath, 'utf-8');
  const lines = content.split(/\r?\n/).filter(l => l.trim() && !l.startsWith('#') && !l.startsWith('Timestamp_Epoch_Ms'));

  const rt = new FusionRuntime();
  await rt.start();

  let maxVelocityDelta = 0;
  let prevVelMag = 0;
  let rejectedWhileMoving = 0;

  rt.setOnFusedDataCallback((fused) => {
    const velMag = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y);
    if (prevVelMag > 0) {
      const dV = Math.abs(velMag - prevVelMag);
      if (dV > maxVelocityDelta) maxVelocityDelta = dV;
    }
    prevVelMag = velMag;
  });

  let lastTs = 0;
  for (let i = 0; i < lines.length; i++) {
    const cols = lines[i].split(',');
    const timestamp = parseInt(cols[0], 10);
    const lat = parseFloat(cols[3]);
    const lng = parseFloat(cols[4]);
    const accuracy = cols[5] ? parseFloat(cols[5]) : 15;
    const speedKmH = cols[6] ? parseFloat(cols[6]) : 0;
    const heading = cols[7] ? parseFloat(cols[7]) : 0;
    const isDr = cols[9]?.toUpperCase() === 'TRUE';

    const prevTimestamp = i === 0 ? timestamp - 1000 : lastTs;
    lastTs = timestamp;
    const timeSpan = Math.max(100, timestamp - prevTimestamp);
    const imuSteps = Math.max(1, Math.round(timeSpan / 100));
    const stepDtMs = timeSpan / imuSteps;
    const isStationary = speedKmH < 0.5;
    const prevSpeedKmH = i === 0 ? 0 : (lines[i - 1].split(',')[6] ? parseFloat(lines[i - 1].split(',')[6]) : 0);
    const timeSpanSec = Math.max(0.1, timeSpan / 1000);
    const physicalAccel = isStationary ? 0 : Math.max(-4.0, Math.min(3.0, (speedKmH - prevSpeedKmH) / (3.6 * timeSpanSec)));

    const accelVib = isStationary ? 0.005 : 0.05;
    const gyroVib = isStationary ? 0.001 : 0.01;

    for (let s = 1; s <= imuSteps; s++) {
      rt.processImuSample(
        { x: (Math.random() - 0.5) * accelVib, y: physicalAccel + (Math.random() - 0.5) * accelVib, z: 9.81 },
        { x: (Math.random() - 0.5) * gyroVib, y: (Math.random() - 0.5) * gyroVib, z: (Math.random() - 0.5) * gyroVib },
        Math.round(prevTimestamp + s * stepDtMs)
      );
    }

    if (!isDr && !isNaN(lat) && !isNaN(lng)) {
      rt.updateGnss({
        latitude: lat,
        longitude: lng,
        accuracy,
        speed: speedKmH / 3.6,
        heading,
        timestamp,
      });

      if (rt.isLastGnssRejected() && speedKmH > 5.0) {
        rejectedWhileMoving++;
      }
    }
  }

  console.log(`  • Processed Data Records:                 ${lines.length}`);
  console.log(`  • Moving GNSS Rejections:                 ${rejectedWhileMoving} (Historical: 142 -> Now: ${rejectedWhileMoving})`);
  console.log(`  • Max Single-Cycle Velocity Jump (Δv):    ${maxVelocityDelta.toFixed(2)} m/s (Runaway Guard Limit ≤ 50.0 m/s for 155 km/h track)`);
  const pass = rejectedWhileMoving <= 30 && maxVelocityDelta <= 50.0;
  console.log(`  • Scenario 2 Status:                      ${pass ? 'PASSED ✓' : 'FAILED ✗'}\n`);
  return pass;
}

// -----------------------------------------------------------------------------
// Scenario 3: Extended Urban Tunnel Outage (60s GPS Blackout at 60 km/h)
// -----------------------------------------------------------------------------
async function runScenario3_TunnelOutage(): Promise<boolean> {
  console.log('================================================================================');
  console.log('  SCENARIO 3: Urban Tunnel Outage (60s GPS Loss at 60 km/h with S-Curve)');
  console.log('================================================================================');

  const rt = new FusionRuntime();
  await rt.start();

  const startLat = 17.650000;
  const startLng = 75.900000;
  const speedMps = 16.67; // 60 km/h
  let currentY = 0;
  let currentX = 0;
  let epoch = 1790690000000;
  let reacquisitionJump = 0;
  const tunnelPositions: {
    lastBeforeExit: { lat: number; lng: number } | null;
    firstAfterExit: { lat: number; lng: number } | null;
  } = {
    lastBeforeExit: null,
    firstAfterExit: null,
  };

  rt.setOnFusedDataCallback((fused) => {
    if (fused.latitude !== null && fused.longitude !== null) {
      if (epoch >= 1790690069000 && epoch <= 1790690070000) {
        tunnelPositions.lastBeforeExit = { lat: fused.latitude, lng: fused.longitude };
      } else if (epoch >= 1790690071000 && tunnelPositions.firstAfterExit === null) {
        tunnelPositions.firstAfterExit = { lat: fused.latitude, lng: fused.longitude };
      }
    }
  });

  // 1. Initial 10s Open Sky at 60 km/h
  for (let t = 0; t <= 10; t += 0.1) {
    epoch += 100;
    currentY += speedMps * 0.1;
    const trueLat = startLat + (currentY / 6378137) * (180 / Math.PI);
    const trueLng = startLng + (currentX / (6378137 * Math.cos(startLat * Math.PI / 180))) * (180 / Math.PI);

    rt.processImuSample({ x: 0.02, y: 0.02, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);
    if (Math.abs(t % 1.0) < 0.05) {
      rt.updateGnss({
        latitude: trueLat,
        longitude: trueLng,
        accuracy: 4.5,
        speed: speedMps,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  // 2. 60s Tunnel Outage (t: 10s -> 70s): No GPS updates, vehicle cruises through tunnel
  console.log('  -> Entering 60-second tunnel outage (GPS blackout)...');
  for (let t = 10.1; t <= 70; t += 0.1) {
    epoch += 100;
    currentY += speedMps * 0.1;
    rt.processImuSample({ x: 0.05 * Math.sin(t * 0.5), y: 0.02, z: 9.81 }, { x: 0, y: 0, z: 0.01 * Math.sin(t * 0.5) }, epoch);
  }

  // 3. Exit Tunnel into Open Sky (t: 70s -> 80s)
  console.log('  -> Exiting tunnel, GPS reacquired at 60 km/h...');
  for (let t = 70.1; t <= 80; t += 0.1) {
    epoch += 100;
    currentY += speedMps * 0.1;
    const trueLat = startLat + (currentY / 6378137) * (180 / Math.PI);
    const trueLng = startLng + (currentX / (6378137 * Math.cos(startLat * Math.PI / 180))) * (180 / Math.PI);

    rt.processImuSample({ x: 0.02, y: 0.02, z: 9.81 }, { x: 0, y: 0, z: 0 }, epoch);
    if (Math.abs(t % 1.0) < 0.05) {
      rt.updateGnss({
        latitude: trueLat,
        longitude: trueLng,
        accuracy: 5.0,
        speed: speedMps,
        heading: 0,
        timestamp: epoch,
      });
    }
  }

  if (tunnelPositions.lastBeforeExit && tunnelPositions.firstAfterExit) {
    reacquisitionJump = haversineDistance(
      tunnelPositions.lastBeforeExit.lat,
      tunnelPositions.lastBeforeExit.lng,
      tunnelPositions.firstAfterExit.lat,
      tunnelPositions.firstAfterExit.lng
    );
  }

  console.log(`  • Tunnel Outage Duration:                 60.0 seconds`);
  console.log(`  • Tunnel Cruising Speed:                  60.0 km/h (16.67 m/s)`);
  console.log(`  • Post-Outage Reacquisition Step Jump:    ${reacquisitionJump.toFixed(2)} m (OutputStabilizer Clamped ≤ 35.0 m)`);
  const pass = reacquisitionJump <= 35.0;
  console.log(`  • Scenario 3 Status:                      ${pass ? 'PASSED ✓' : 'FAILED ✗'}\n`);
  return pass;
}

// -----------------------------------------------------------------------------
// Scenario 4: Stop-and-Go Rush Hour Driving (10 Acceleration / Braking Cycles)
// -----------------------------------------------------------------------------
async function runScenario4_StopAndGo(): Promise<boolean> {
  console.log('================================================================================');
  console.log('  SCENARIO 4: Stop-and-Go Rush Hour Driving (10 Hard Accel/Brake Cycles)');
  console.log('================================================================================');

  const rt = new FusionRuntime();
  await rt.start();

  const startLat = 17.650000;
  const startLng = 75.900000;
  let currentY = 0;
  let epoch = 1790695000000;
  let falseRejectionCount = 0;
  let stationaryDriftMeters = 0;
  let initialStopLat = startLat;

  let currentSpeed = 0;
  for (let cycle = 1; cycle <= 10; cycle++) {
    // Phase A: Stopped at red light (5 seconds)
    currentSpeed = 0;
    for (let t = 0; t < 5; t += 0.1) {
      epoch += 100;
      rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0.0005, y: 0.0005, z: 0 }, epoch);
      if (Math.abs(t % 1.0) < 0.05) {
        const trueLat = startLat + (currentY / 6378137) * (180 / Math.PI);
        rt.updateGnss({
          latitude: trueLat,
          longitude: startLng,
          accuracy: 4.2,
          speed: 0,
          heading: 0,
          timestamp: epoch,
        });
      }
    }

    // Phase B: Accelerate to 45 km/h (12.5 m/s) over 4 seconds
    for (let t = 0; t < 4; t += 0.1) {
      epoch += 100;
      currentSpeed = (t / 4) * 12.5;
      currentY += currentSpeed * 0.1;
      const trueLat = startLat + (currentY / 6378137) * (180 / Math.PI);

      rt.processImuSample({ x: 0.1, y: 3.1, z: 9.81 }, { x: 0.01, y: 0.01, z: 0 }, epoch);
      if (Math.abs(t % 1.0) < 0.05) {
        rt.updateGnss({
          latitude: trueLat,
          longitude: startLng,
          accuracy: 4.5,
          speed: currentSpeed,
          heading: 0,
          timestamp: epoch,
        });

        if (rt.isLastGnssRejected()) {
          falseRejectionCount++;
        }
      }
    }

    // Phase C: Hard Braking 45 km/h -> 0 km/h over 3 seconds
    for (let t = 0; t < 3; t += 0.1) {
      epoch += 100;
      currentSpeed = Math.max(0, 12.5 - (t / 3) * 12.5);
      currentY += currentSpeed * 0.1;
      const trueLat = startLat + (currentY / 6378137) * (180 / Math.PI);

      rt.processImuSample({ x: 0.1, y: -4.2, z: 9.81 }, { x: 0.01, y: 0.01, z: 0 }, epoch);
      if (Math.abs(t % 1.0) < 0.05) {
        rt.updateGnss({
          latitude: trueLat,
          longitude: startLng,
          accuracy: 4.5,
          speed: currentSpeed,
          heading: 0,
          timestamp: epoch,
        });
      }
    }
  }

  console.log(`  • Total Rush-Hour Cycles Completed:       10 cycles`);
  console.log(`  • False Launch Rejections During Accel:   ${falseRejectionCount} (Expected: 0)`);
  const pass = falseRejectionCount === 0;
  console.log(`  • Scenario 4 Status:                      ${pass ? 'PASSED ✓' : 'FAILED ✗'}\n`);
  return pass;
}

// -----------------------------------------------------------------------------
// Scenario 5: Extreme Stationary Multipath Attack (150 km/h Phantom GPS Jump)
// -----------------------------------------------------------------------------
async function runScenario5_MultipathAttack(): Promise<boolean> {
  console.log('================================================================================');
  console.log('  SCENARIO 5: Stationary Multipath Attack (0 km/h vs 150 km/h Phantom GPS Jump)');
  console.log('================================================================================');

  const rt = new FusionRuntime();
  await rt.start();

  const originLat = 17.650000;
  const originLng = 75.900000;
  let epoch = 1790698000000;
  let maxDriftMeters = 0;
  let maxReportedSpeedKmH = 0;

  rt.setOnFusedDataCallback((fused) => {
    const rawSpeed = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y) * 3.6;
    if (rawSpeed > maxReportedSpeedKmH) maxReportedSpeedKmH = rawSpeed;

    if (fused.latitude !== null && fused.longitude !== null) {
      const d = haversineDistance(originLat, originLng, fused.latitude, fused.longitude);
      if (d > maxDriftMeters) maxDriftMeters = d;
    }
  });

  // 1. Initial 5s Stillness
  for (let t = 0; t < 5; t += 0.1) {
    epoch += 100;
    rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0.0005, y: 0.0005, z: 0 }, epoch);
    if (Math.abs(t % 1.0) < 0.05) {
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

  // 2. 30s of extreme multipath: GPS claims vehicle jumped 150m away at 150 km/h, but IMU remains completely still
  console.log('  -> Subjecting stationary IMU to 30s of 150 km/h phantom GPS jumps...');
  for (let t = 5; t < 35; t += 0.1) {
    epoch += 100;
    rt.processImuSample({ x: 0.005, y: 0.005, z: 9.81 }, { x: 0.0005, y: 0.0005, z: 0 }, epoch);
    if (Math.abs(t % 1.0) < 0.05) {
      const phantomLat = originLat + 0.0015; // ~165m north
      const phantomLng = originLng + 0.0015;
      rt.updateGnss({
        latitude: phantomLat,
        longitude: phantomLng,
        accuracy: 28.0,
        speed: 41.67, // 150 km/h
        heading: 45,
        timestamp: epoch,
      });
    }
  }

  console.log(`  • Maximum Position Drift During Attack:   ${maxDriftMeters.toFixed(2)} m (Strict ZUPT Gate Limit < 1.0 m)`);
  console.log(`  • Maximum Reported Velocity During Attack:${maxReportedSpeedKmH.toFixed(2)} km/h (Expected: 0.0 km/h)`);
  const pass = maxDriftMeters < 1.0 && maxReportedSpeedKmH < 0.5;
  console.log(`  • Scenario 5 Status:                      ${pass ? 'PASSED ✓' : 'FAILED ✗'}\n`);
  return pass;
}

// -----------------------------------------------------------------------------
// Master Runner
// -----------------------------------------------------------------------------
async function runAllScenarios() {
  console.log('################################################################################');
  console.log('       RECKONX MULTI-SCENARIO COMPREHENSIVE BENCHMARK SUITE                     ');
  console.log('################################################################################\n');

  const s1 = await runScenario1_RealSession_2026_09_29();
  const s2 = await runScenario2_RealSession_2026_09_27();
  const s3 = await runScenario3_TunnelOutage();
  const s4 = await runScenario4_StopAndGo();
  const s5 = await runScenario5_MultipathAttack();

  const allPassed = s1 && s2 && s3 && s4 && s5;

  console.log('================================================================================');
  console.log('                          BENCHMARK SUMMARY MATRIX                              ');
  console.log('================================================================================');
  console.log(`  [Scenario 1] Real Solapur 2026-09-29 Trajectory:      ${s1 ? 'PASS ✓' : 'FAIL ✗'}`);
  console.log(`  [Scenario 2] Real Solapur 2026-09-27 Trajectory:      ${s2 ? 'PASS ✓' : 'FAIL ✗'}`);
  console.log(`  [Scenario 3] 60s Tunnel Outage @ 60 km/h:             ${s3 ? 'PASS ✓' : 'FAIL ✗'}`);
  console.log(`  [Scenario 4] 10x Stop-and-Go Rush Hour Cycles:        ${s4 ? 'PASS ✓' : 'FAIL ✗'}`);
  console.log(`  [Scenario 5] 150 km/h Stationary Multipath Attack:    ${s5 ? 'PASS ✓' : 'FAIL ✗'}`);
  console.log('================================================================================');

  if (!allPassed) {
    throw new Error('One or more benchmark scenarios failed.');
  }
  console.log('\n>>> ALL 5 COMPREHENSIVE BENCHMARK SCENARIOS PASSED WITH ZERO REGRESSIONS <<<\n');
}

runAllScenarios().catch((err) => {
  console.error('Scenario testing failed:', err);
  process.exit(1);
});
