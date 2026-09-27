import { FusionRuntime } from '../src/services/ekf/FusionRuntime';
import { LogExportService } from '../src/services/logExportService';
import type { RecordedGPSPoint } from '../src/services/api/trackingService';

async function runMovementSessionTest() {
  console.log('=== Starting 75-Second Moving Walking Trajectory Session ===');
  const rt = new FusionRuntime();
  await rt.start();

  const recordedPoints: RecordedGPSPoint[] = [];
  let currentFusedState: RecordedGPSPoint | null = null;

  rt.setOnFusedDataCallback((fused) => {
    const isIDR = fused.sourceMode === 'IDR';
    const rawSpeedKmH = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y) * 3.6;
    const speedKmH = rawSpeedKmH < 0.3 ? 0 : Math.round(rawSpeedKmH * 10) / 10;

    // Record at 1 Hz simulation cadence
    if (fused.latitude !== null && fused.longitude !== null) {
      currentFusedState = {
        timestamp: fused.timestamp,
        lat: fused.latitude,
        lng: fused.longitude,
        accuracyMeters: (fused.accuracy !== null && fused.accuracy !== undefined) ? fused.accuracy : undefined,
        speedKmH,
        headingDeg: fused.heading,
        isDeadReckoning: isIDR,
        rawInsVelX: fused.velocity.x,
        rawInsVelY: fused.velocity.y,
        aiCorrectedVelX: null,
        aiCorrectedVelY: null,
        aiConfidence: null,
      };
    }
  });

  // Start at origin (Solapur / Reference point: 17.6599, 75.9064)
  const startLat = 17.659920;
  const startLng = 75.906410;
  const startTime = Date.now();
  
  // Walking speed: ~1.3 m/s (~4.68 km/h)
  const walkSpeed = 1.35; // m/s
  let currentX = 0; // East (m)
  let currentY = 0; // North (m)

  // Simulation: 75 seconds (10 Hz IMU = 750 samples, 1 Hz GPS = 75 samples)
  // Stage 1 (0-30s): Walk East (Heading 90°) under clear sky (Accuracy 4.5m - 6.0m) -> GOOD / GNSS
  // Stage 2 (30-50s): Turn North (Heading 0°) near tall building (Accuracy 28m - 45m) -> DEGRADED / GNSS
  // Stage 3 (50-65s): Enter covered walkway (GPS Lost / Outage) -> WEAK_LOST / DEAD_RECKONING (INS propagation)
  // Stage 4 (65-75s): Exit covered walkway back to open sky (Accuracy 5.0m) -> Recovery to GOOD / GNSS

  for (let t = 0; t <= 75; t += 0.1) {
    const currentEpoch = startTime + Math.round(t * 1000);

    let headingDeg = 90; // Default East
    let currentVelX = 0;
    let currentVelY = 0;

    if (t < 30) {
      headingDeg = 90;
      currentVelX = walkSpeed;
      currentVelY = 0;
    } else if (t < 50) {
      // 90-degree turn to North
      headingDeg = 0;
      currentVelX = 0;
      currentVelY = walkSpeed;
    } else if (t < 65) {
      // Continuing North in covered walkway
      headingDeg = 0;
      currentVelX = 0;
      currentVelY = walkSpeed;
    } else {
      // Turn East again in open courtyard
      headingDeg = 90;
      currentVelX = walkSpeed;
      currentVelY = 0;
    }

    currentX += currentVelX * 0.1;
    currentY += currentVelY * 0.1;

    // Convert local ENU (m) to Lat/Lng
    const R_EARTH = 6378137;
    const latRad = startLat * (Math.PI / 180);
    const trueLat = startLat + (currentY / R_EARTH) * (180 / Math.PI);
    const trueLng = startLng + (currentX / (R_EARTH * Math.cos(latRad))) * (180 / Math.PI);

    // IMU sample (10 Hz with realistic walking acceleration cadence ~1.8 Hz footstep oscillations)
    const stepFreq = 1.8;
    const stepAccel = 1.2 * Math.sin(2 * Math.PI * stepFreq * t);
    // Turn left at t=30s (East to North: +1.57 rad/s), Turn right at t=65s (North to East: -1.57 rad/s)
    const gyroYawRate = (t >= 29.5 && t <= 30.5) ? 1.57 : (t >= 64.5 && t <= 65.5) ? -1.57 : 0;

    rt.processImuSample(
      { x: currentVelX > 0 ? stepAccel : 0.05, y: currentVelY > 0 ? stepAccel : 0.05, z: 9.81 },
      { x: 0.01, y: 0.01, z: gyroYawRate },
      currentEpoch
    );

    // GPS sample delivered every 1.0 second (except during blackout t: 50-65s)
    if (Math.abs(t % 1.0) < 0.05) {
      if (t < 30) {
        // Clear sky
        rt.updateGnss({
          latitude: trueLat + (Math.random() - 0.5) * 0.00001,
          longitude: trueLng + (Math.random() - 0.5) * 0.00001,
          accuracy: 4.8 + Math.sin(t) * 0.8,
          speed: walkSpeed,
          heading: headingDeg,
          timestamp: currentEpoch
        });
      } else if (t >= 30 && t < 50) {
        // Degraded accuracy near tall building
        rt.updateGnss({
          latitude: trueLat + (Math.random() - 0.5) * 0.00005,
          longitude: trueLng + (Math.random() - 0.5) * 0.00005,
          accuracy: 32.0 + Math.cos(t) * 6.0,
          speed: walkSpeed,
          heading: headingDeg,
          timestamp: currentEpoch
        });
      } else if (t >= 65) {
        // Recovered clear sky
        rt.updateGnss({
          latitude: trueLat + (Math.random() - 0.5) * 0.00001,
          longitude: trueLng + (Math.random() - 0.5) * 0.00001,
          accuracy: 5.2,
          speed: walkSpeed,
          heading: headingDeg,
          timestamp: currentEpoch
        });
      }

      if (currentFusedState !== null) {
        const stateToRecord: RecordedGPSPoint = currentFusedState;
        recordedPoints.push({ ...stateToRecord, timestamp: currentEpoch });
      }
    }
  }

  console.log(`\n=== Movement Session Finished. Total Points Logged: ${recordedPoints.length} ===\n`);

  // Build and print CSV summary
  const csvHeaders = [
    'Timestamp_Epoch_Ms',
    'Time_ISO',
    'Source_Type',
    'Latitude',
    'Longitude',
    'Accuracy_Meters',
    'Speed_KmH',
    'Heading_Deg',
    'Is_Dead_Reckoning',
    'Raw_DR_VelX_ms',
    'Raw_DR_VelY_ms',
  ];

  console.log(csvHeaders.join(','));
  for (let i = 0; i < recordedPoints.length; i++) {
    const pt = recordedPoints[i];
    const row = [
      pt.timestamp,
      new Date(pt.timestamp).toISOString(),
      pt.isDeadReckoning ? 'DEAD_RECKONING' : 'GNSS',
      pt.lat.toFixed(6),
      pt.lng.toFixed(6),
      pt.accuracyMeters !== undefined && pt.accuracyMeters !== null ? pt.accuracyMeters.toFixed(1) : '',
      pt.speedKmH !== undefined ? pt.speedKmH.toFixed(1) : '',
      pt.headingDeg !== undefined ? pt.headingDeg.toFixed(1) : '',
      pt.isDeadReckoning ? 'TRUE' : 'FALSE',
      pt.rawInsVelX !== undefined ? pt.rawInsVelX.toFixed(4) : '',
      pt.rawInsVelY !== undefined ? pt.rawInsVelY.toFixed(4) : '',
    ];
    // Print every 5th row and all transition rows
    if (i % 5 === 0 || i === 29 || i === 30 || i === 50 || i === 51 || i === 65 || i === 66) {
      console.log(row.join(','));
    }
  }

  // Statistical Verification
  const gnssPoints = recordedPoints.filter(p => !p.isDeadReckoning);
  const drPoints = recordedPoints.filter(p => p.isDeadReckoning);
  const blankAccuracyInGnss = gnssPoints.filter(p => p.accuracyMeters === undefined || p.accuracyMeters === null);
  
  console.log('\n=== CSV Validation Summary ===');
  console.log(`Total Points: ${recordedPoints.length}`);
  console.log(`GNSS Points: ${gnssPoints.length}`);
  console.log(`DEAD_RECKONING Points: ${drPoints.length}`);
  console.log(`GNSS Points with Blank Accuracy: ${blankAccuracyInGnss.length} (Expected: 0)`);
  console.log(`Average Speed: ${(recordedPoints.reduce((s, p) => s + (p.speedKmH || 0), 0) / recordedPoints.length).toFixed(2)} km/h`);
  console.log(`Max Speed: ${Math.max(...recordedPoints.map(p => p.speedKmH || 0)).toFixed(2)} km/h`);
  console.log(`Start Lat/Lng: ${recordedPoints[0].lat.toFixed(6)}, ${recordedPoints[0].lng.toFixed(6)}`);
  console.log(`End Lat/Lng: ${recordedPoints[recordedPoints.length - 1].lat.toFixed(6)}, ${recordedPoints[recordedPoints.length - 1].lng.toFixed(6)}`);
}

runMovementSessionTest().catch(console.error);
