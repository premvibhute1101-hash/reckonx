/**
 * frontend_unit_tests.ts
 * =======================
 * Level 1 & 2 Unit Tests for Frontend TypeScript Core Services & Algorithms:
 * - 15-State EkfCore & InsMechanization (Butterworth 2nd-order LPF)
 * - GnssQualityStateMachine & OutputStabilizer
 * - Distance Formatters & Route Progress Calculators
 */

import { InsMechanization } from '../src/services/ekf/InsMechanization';
import { GnssQualityStateMachine } from '../src/services/ekf/GnssQualityStateMachine';
import { OutputStabilizer } from '../src/services/ekf/OutputStabilizer';
import { formatKmDistance } from '../src/utils/distanceFormatter';
import { calculateRemainingRoadDistance } from '../src/utils/routeProgress';

interface TestResult {
  name: string;
  passed: boolean;
  error?: string;
  durationMs: number;
}

const results: TestResult[] = [];

function assert(condition: any, message: string): asserts condition {
  if (!condition) throw new Error(message);
}

function assertAlmostEqual(actual: number, expected: number, delta: number = 0.01, message?: string) {
  if (Math.abs(actual - expected) > delta) {
    throw new Error(message || `Expected ${expected} ± ${delta}, got ${actual}`);
  }
}

async function runTest(name: string, fn: () => void | Promise<void>) {
  const start = Date.now();
  try {
    await fn();
    results.push({ name, passed: true, durationMs: Date.now() - start });
    console.log(`  [PASS] ${name}`);
  } catch (err: any) {
    results.push({ name, passed: false, error: err.message, durationMs: Date.now() - start });
    console.error(`  [FAIL] ${name}: ${err.message}`);
  }
}

export async function runFrontendUnitTests(): Promise<TestResult[]> {
  console.log("\n================================================================================");
  console.log("  FRONTEND & TYPESCRIPT CORE UNIT TESTS");
  console.log("================================================================================");

  // 1. Distance Formatter Tests
  await runTest("DistanceFormatter: formats kilometers accurately", () => {
    assert(formatKmDistance(0.45, 'km') === "0.5 km", `Expected '0.5 km', got '${formatKmDistance(0.45, 'km')}'`);
    assert(formatKmDistance(12.345, 'km') === "12.3 km", `Expected '12.3 km', got '${formatKmDistance(12.345, 'km')}'`);
    assert(formatKmDistance(0, 'km') === "0.0 km", `Expected '0.0 km', got '${formatKmDistance(0, 'km')}'`);
  });

  await runTest("DistanceFormatter: formats imperial miles accurately", () => {
    assert(formatKmDistance(0.1, 'mi') === "0.1 mi", `Expected '0.1 mi', got '${formatKmDistance(0.1, 'mi')}'`);
    assert(formatKmDistance(5.0, 'mi') === "3.1 mi", `Expected '3.1 mi', got '${formatKmDistance(5.0, 'mi')}'`);
  });

  // 2. Route Progress Tests
  await runTest("RouteProgress: calculates accurate remaining road distance along polyline", () => {
    const polyline: [number, number][] = [
      [18.9220, 72.8347],
      [18.9250, 72.8347],
      [18.9300, 72.8347],
    ];
    const currentLoc: [number, number] = [18.9250, 72.8347]; // Midpoint
    const progress = calculateRemainingRoadDistance(currentLoc, polyline);

    assert(progress.remainingDistanceKm > 0, "Remaining distance should be > 0");
    assert(progress.offRouteDistanceMeters < 50, "Off-route distance should be minimal on path");
  });

  await runTest("RouteProgress: detects off-route distance when far from path", () => {
    const polyline: [number, number][] = [
      [18.9220, 72.8347],
      [18.9300, 72.8347],
    ];
    const offRouteLoc: [number, number] = [19.0000, 73.0000]; // Far off
    const progress = calculateRemainingRoadDistance(offRouteLoc, polyline);
    assert(progress.offRouteDistanceMeters > 500, "Vehicle far away should have high off-route meters");
  });

  // 3. InsMechanization & Butterworth LPF Tests
  await runTest("InsMechanization: attitude initialization from gravity vector", () => {
    const ins = new InsMechanization();
    ins.initializeAttitude({ x: 0, y: 0, z: 9.81 });
    assertAlmostEqual(ins.attitude.pitch, 0.0, 0.05);
    assertAlmostEqual(ins.attitude.roll, 0.0, 0.05);
  });

  await runTest("InsMechanization: 2nd-order Butterworth LPF attenuates high-frequency noise", () => {
    const ins = new InsMechanization();
    ins.initializeAttitude({ x: 0, y: 0, z: 9.81 });

    // Step with sudden sharp transient spike (15 m/s^2)
    ins.predict(0.1, { x: 15.0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 });
    const filtered = ins.getFilteredAccel();
    assert(filtered !== null, "Filtered accel should exist");
    assert(filtered!.x < 15.0, "LPF must attenuate sudden spike (expected < 15.0)");
  });

  // 4. GnssQualityStateMachine Tests
  await runTest("GnssQualityStateMachine: updates trust factors and inflation correctly", () => {
    const sm = new GnssQualityStateMachine();
    
    // Good fix (< 10m)
    const rGood = sm.updateState(4.5);
    assert(sm.getState() === 'GOOD', "Expected GOOD state");
    assert(rGood === 1.0, "Expected base trust multiplier 1.0");

    // Degraded fix (10 - 25m)
    const rDeg = sm.updateState(15.0);
    assert(sm.getState() === 'DEGRADED', "Expected DEGRADED state");
    assert(rDeg > 1.0, "Expected inflated R multiplier");

    // Outage (null or > 25m)
    const rLost = sm.updateState(null);
    assert(sm.getState() === 'WEAK_LOST', "Expected WEAK_LOST state");
    assert(rLost >= 1000.0, "Expected high outage inflation");
  });

  // 5. OutputStabilizer Tests
  await runTest("OutputStabilizer: clamps unphysical position jumps exceeding max speed", () => {
    const stabilizer = new OutputStabilizer({ maxSpeedMps: 30.0, maxAccelMps2: 5.0 });
    
    // Initial fix at rest
    const s1 = stabilizer.process({
      lat: 18.9220,
      lon: 72.8347,
      timestamp: 1000,
      gnssState: 'GOOD'
    });
    assert(s1.lat === 18.9220, "Initial lat should match");
    assert(!s1.wasClamped, "Initial fix should not be clamped");

    // Sudden impossible jump (~10 km in 1 second = 10,000 m/s >> 30 m/s limit)
    const s2 = stabilizer.process({
      lat: 19.0000,
      lon: 73.0000,
      timestamp: 2000,
      gnssState: 'GOOD'
    });
    assert(s2.wasClamped, "Stabilizer must clamp impossible velocity jump");
    assert(s2.clampEvent !== undefined, "Clamp event metadata should be present");
  });

  console.log("--------------------------------------------------------------------------------");
  const passed = results.filter(r => r.passed).length;
  console.log(`Frontend Unit Tests: ${passed}/${results.length} Passed`);
  return results;
}

if (import.meta.url.endsWith(process.argv[1]) || process.argv[1]?.includes('frontend_unit_tests')) {
  runFrontendUnitTests();
}
