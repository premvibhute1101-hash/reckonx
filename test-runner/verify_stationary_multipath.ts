import { FusionRuntime } from '../src/services/ekf/FusionRuntime';

async function testStationaryMultipathSpike() {
  console.log('=== Starting Stationary GPS Multipath Spike (105-128m) Verification ===');
  const rt = new FusionRuntime();
  await rt.start();

  let maxSpeedDuringSpike = 0;
  let initialPosition: { lat: number; lng: number } | null = null;
  let maxPositionDriftMeters = 0;

  rt.setOnFusedDataCallback((fused) => {
    const rawSpeedKmH = Math.sqrt(fused.velocity.x * fused.velocity.x + fused.velocity.y * fused.velocity.y) * 3.6;
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
  for (let t = 0; t <= 5.0; t += 0.1) {
    const epoch = startTime + Math.round(t * 1000);
    rt.processImuSample({ x: 0.01, y: 0.01, z: 9.81 }, { x: 0.001, y: 0.001, z: 0 }, epoch);
    if (Math.abs(t % 1.0) < 0.05) {
      rt.updateGnss({
        latitude: originLat + (Math.random() - 0.5) * 0.000005,
        longitude: originLng + (Math.random() - 0.5) * 0.000005,
        accuracy: 4.5,
        speed: 0,
        heading: 0,
        timestamp: epoch
      });
    }
  }

  // 2. 10 seconds: Severe Multipath Spike (Accuracy 105 - 128m, GPS coordinates jumping by ~25-40m)
  console.log('Injecting 10s multipath spike: accuracy=105..128m, jump=25..40m...');
  for (let t = 5.1; t <= 15.0; t += 0.1) {
    const epoch = startTime + Math.round(t * 1000);
    rt.processImuSample({ x: 0.01, y: 0.01, z: 9.81 }, { x: 0.001, y: 0.001, z: 0 }, epoch);
    if (Math.abs(t % 1.0) < 0.05) {
      const spikeAccuracy = 105 + Math.random() * 23; // 105..128m
      const jumpMeters = 25 + Math.random() * 15; // 25..40m
      const jumpLat = originLat + (jumpMeters / 111139);
      const jumpLng = originLng + (jumpMeters / (111139 * Math.cos(originLat * Math.PI / 180)));

      rt.updateGnss({
        latitude: jumpLat,
        longitude: jumpLng,
        accuracy: spikeAccuracy,
        speed: 0,
        heading: 0,
        timestamp: epoch
      });

      const state = rt.getLatestState();
      const speedKmH = state ? Math.sqrt(state.velocity.x * state.velocity.x + state.velocity.y * state.velocity.y) * 3.6 : 0;
      maxSpeedDuringSpike = Math.max(maxSpeedDuringSpike, speedKmH);
      console.log(`  [t=${t.toFixed(1)}s] acc=${spikeAccuracy.toFixed(1)}m, speed=${speedKmH.toFixed(2)} km/h, quality=${state?.gnssState}, sourceMode=${state?.sourceMode}`);
    }
  }

  // 3. 5 seconds: Recovery to good GPS (Accuracy 4.5m)
  console.log('Recovering to clear sky...');
  for (let t = 15.1; t <= 20.0; t += 0.1) {
    const epoch = startTime + Math.round(t * 1000);
    rt.processImuSample({ x: 0.01, y: 0.01, z: 9.81 }, { x: 0.001, y: 0.001, z: 0 }, epoch);
    if (Math.abs(t % 1.0) < 0.05) {
      rt.updateGnss({
        latitude: originLat,
        longitude: originLng,
        accuracy: 4.5,
        speed: 0,
        heading: 0,
        timestamp: epoch
      });
    }
  }

  console.log('\n=== Stationary Multipath Spike Summary ===');
  console.log(`Max Speed Reported During Spike: ${maxSpeedDuringSpike.toFixed(2)} km/h (Expected: 0.00 km/h)`);
  console.log(`Max Position Drift: ${maxPositionDriftMeters.toFixed(2)} meters (Expected: < 1.00m)`);

  if (maxSpeedDuringSpike === 0 && maxPositionDriftMeters < 1.0) {
    console.log('PASSED: Bad-accuracy GPS multipath spike was successfully rejected and zero speed held perfectly.');
  } else {
    console.error('FAILED: Leakage occurred during multipath spike.');
    process.exit(1);
  }
}

testStationaryMultipathSpike().catch((err) => {
  console.error(err);
  process.exit(1);
});
