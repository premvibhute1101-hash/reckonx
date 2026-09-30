import { fusionRuntime } from './src/core/FusionRuntime';
import { useSensorStore } from './src/store/useSensorStore';

// Mock Date.now to control time
let currentTime = 1000000;
Date.now = () => currentTime;

let lastLat = 0;
let lastLon = 0;
let mode = '';

fusionRuntime.setOnFusedDataCallback((state) => {
    lastLat = state.latitude as number;
    lastLon = state.longitude as number;
    mode = state.sourceMode;
    console.log(`[Time: ${currentTime}] Mode: ${mode}, Pos: (${lastLat?.toFixed(6)}, ${lastLon?.toFixed(6)}), Vel: (${state.velocity.x.toFixed(2)}, ${state.velocity.y.toFixed(2)})`);
});

async function run() {
    await fusionRuntime.start();

    console.log("=== PHASE 1: GNSS AVAILABLE (10s) ===");
    for (let i = 0; i < 10; i++) {
        currentTime += 1000; // Advance 1 second

        // Trigger GNSS update
        useSensorStore.setState({
            gnss: {
                latitude: 37.7749 + i * 0.0001,
                longitude: -122.4194 + i * 0.0001,
                altitude: 0,
                speed: 10,
                heading: 45,
                accuracy: 5,
                timestamp: currentTime
            }
        });

        // Trigger some IMU updates
        for (let j = 0; j < 10; j++) {
            currentTime += 100;
            useSensorStore.setState({
                accel: { x: 0, y: 0, z: 9.81 },
                gyro: { x: 0, y: 0, z: 0 }
            });
        }
    }

    console.log("=== PHASE 2: BLACKOUT (GNSS LOST) ===");
    for (let i = 0; i < 50; i++) {
        currentTime += 100; // Advance 100ms
        
        // Only IMU updates, no GNSS
        useSensorStore.setState({
            accel: { x: 0.5, y: 0.5, z: 9.81 }, // some acceleration
            gyro: { x: 0, y: 0, z: 0 }
        });
    }

    fusionRuntime.stop();
}

run();
