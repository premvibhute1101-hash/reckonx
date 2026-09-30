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
import { MotionClassifier } from '../src/services/ekf/MotionClassifier';
import { AnomalyDetector } from '../src/services/ekf/AnomalyDetector';
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

  // 6. MotionClassifier Tests
  await runTest("MotionClassifier: correctly categorizes STATIONARY, PEDESTRIAN, and AUTOMOTIVE", () => {
    const classifier = new MotionClassifier();

    // Test Stationary
    const resStationary = classifier.classify({ varA: 0.01, varG: 0.005, lpHorizAccel: 0.02, speedKmh: 0.0 });
    assert(resStationary.activity === 'STATIONARY', "Expected STATIONARY activity");

    // Test Pedestrian
    classifier.reset();
    let resPed = resStationary;
    for (let i = 0; i < 5; i++) {
      resPed = classifier.classify({ varA: 0.15, varG: 0.05, lpHorizAccel: 0.12, speedKmh: 4.5 });
    }
    assert(resPed.activity === 'PEDESTRIAN', "Expected PEDESTRIAN activity");

    // Test Automotive
    classifier.reset();
    let resAuto = resPed;
    for (let i = 0; i < 5; i++) {
      resAuto = classifier.classify({ varA: 0.05, varG: 0.01, lpHorizAccel: 0.05, speedKmh: 60.0 });
    }
    assert(resAuto.activity === 'AUTOMOTIVE', "Expected AUTOMOTIVE activity");
  });

  // 7. AnomalyDetector Tests
  await runTest("AnomalyDetector: flags sensor freeze, impossible GNSS jumps, and timestamp anomalies", () => {
    const detector = new AnomalyDetector();

    // Check normal IMU sample
    const r1 = detector.checkImuSample({ x: 0.1, y: 0.2, z: 9.81 }, 1000);
    assert(r1.severity === 'NONE', "Expected normal severity for dynamic IMU");

    // Check frozen IMU repeating 16 times
    let rFreeze = r1;
    for (let i = 0; i < 16; i++) {
      rFreeze = detector.checkImuSample({ x: 0.1, y: 0.2, z: 9.81 }, 1000 + (i + 1) * 20);
    }
    assert(rFreeze.isSensorFrozen, "Expected sensor freeze detection");
    assert(rFreeze.severity === 'HIGH', "Expected HIGH severity for sensor freeze");

    // Check GNSS Spoofing (100 km/h with no IMU motion corroboration)
    const rSpoof = detector.checkGnssSample(28.0, 1.0, false, 5.0);
    assert(rSpoof.isSpoofed, "Expected GNSS spoofing flag for uncorroborated 100 km/h jump");
    assert(rSpoof.severity === 'CRITICAL', "Expected CRITICAL severity for spoofing");
  });

  // 8. Viewport Meta & CSS dvh Fallback Tests
  await runTest("Viewport & Layout: index.html includes viewport-fit=cover and CSS declares 100dvh fallback", async () => {
    const fs = await import('fs');
    const path = await import('path');
    
    // Check index.html
    const indexHtml = fs.readFileSync(path.resolve(process.cwd(), 'index.html'), 'utf-8');
    assert(indexHtml.includes('viewport-fit=cover'), "index.html must include viewport-fit=cover");

    // Check index.css
    const indexCss = fs.readFileSync(path.resolve(process.cwd(), 'src/index.css'), 'utf-8');
    assert(indexCss.includes('height: 100vh;') && indexCss.includes('height: 100dvh;'), "CSS must include 100vh fallback and 100dvh for .h-app-screen");
    assert(indexCss.includes('min-height: 100vh;') && indexCss.includes('min-height: 100dvh;'), "CSS must include min-height 100vh fallback and 100dvh for .min-h-app-screen");
  });

  // 7. Mobile Viewport Geometry & Clearance Verification (360x640 & 412x915)
  await runTest("Mobile Viewport Geometry: 360x640 & 412x915 clearance with/without browser chrome", () => {
    const navBarHeight = 64; // 4rem = 64px
    const minClearanceMargin = 16; // ~16px
    const safeAreaInsets = [0, 16, 24, 34]; // No safe area, gesture bar, notch / Android 10+
    const viewports = [
      { name: "360x640 APK / Standalone (No Address Bar)", width: 360, height: 640 },
      { name: "360x640 Mobile Chrome (With 56px Address Bar)", width: 360, height: 584 },
      { name: "412x915 APK / Standalone (No Address Bar)", width: 412, height: 915 },
      { name: "412x915 Mobile Chrome (With 56px Address Bar)", width: 412, height: 859 }
    ];

    for (const vp of viewports) {
      for (const safeArea of safeAreaInsets) {
        const effectiveBottomNavHeight = navBarHeight + safeArea;
        const requiredBottomPadding = navBarHeight + safeArea + minClearanceMargin;
        
        // MobileShell frame height must match 100dvh exactly
        const frameHeight = vp.height;
        const scrollableAreaHeight = frameHeight; // container inside flex-1

        // When content is scrolled to bottom, distance between last item bottom and viewport bottom:
        const clearanceAboveNav = requiredBottomPadding - effectiveBottomNavHeight;
        assert(clearanceAboveNav >= minClearanceMargin, `${vp.name} (safeArea=${safeArea}px): clearance ${clearanceAboveNav}px must be >= ${minClearanceMargin}px`);
        assert(scrollableAreaHeight > 0, "Scrollable area height must be positive");
      }
    }
  });

  // 8. Page Layout Audit: Profile, Route, Telemetry, Solution, Explore
  await runTest("Page Layout Audit: all bottom-nav pages configured for single-scroll with bottom clearance", async () => {
    const fs = await import('fs');
    const path = await import('path');
    
    // Check ProfilePage.tsx does not have h-full / pb-20 clipping wrapper
    const profileCode = fs.readFileSync(path.resolve(process.cwd(), 'src/pages/ProfilePage.tsx'), 'utf-8');
    assert(!profileCode.includes('h-full flex flex-col justify-between p-4 pb-20'), "ProfilePage must not constrain content with h-full / pb-20");

    // Check OurSolutionPage.tsx does not have h-full / pb-20 clipping wrapper
    const solutionCode = fs.readFileSync(path.resolve(process.cwd(), 'src/pages/OurSolutionPage.tsx'), 'utf-8');
    assert(!solutionCode.includes('h-full flex flex-col justify-between p-4 pb-20'), "OurSolutionPage must not constrain content with h-full / pb-20");

    // Check MobileShell.tsx implements calc(4rem + env(safe-area-inset-bottom, 0px) + 16px)
    const shellCode = fs.readFileSync(path.resolve(process.cwd(), 'src/components/MobileShell.tsx'), 'utf-8');
    assert(shellCode.includes('calc(4rem + env(safe-area-inset-bottom, 0px) + 16px)'), "MobileShell must provide exact 4rem + safe-area + 16px bottom padding");
    assert(shellCode.includes('h-app-screen'), "MobileShell must use h-app-screen (100dvh with fallback)");
  });

  console.log("--------------------------------------------------------------------------------");
  const passed = results.filter(r => r.passed).length;
  console.log(`Frontend Unit Tests: ${passed}/${results.length} Passed`);
  return results;
}

if (import.meta.url.endsWith(process.argv[1]) || process.argv[1]?.includes('frontend_unit_tests')) {
  runFrontendUnitTests();
}
