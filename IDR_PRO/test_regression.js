import { Matrix } from 'ml-matrix';
import { fusionRuntime } from './src/core/FusionRuntime.js';
import { useSensorStore } from './src/store/useSensorStore.js';
import { EkfCore } from './src/core/EkfCore.js';

// Setup Mock Date
let currentTime = 1000000;
Date.now = () => currentTime;

let positions = [];

fusionRuntime.setOnFusedDataCallback((state) => {
    positions.push({
        time: currentTime,
        lat: state.latitude,
        lon: state.longitude,
        vx: state.velocity.x,
        vy: state.velocity.y,
        mode: state.sourceMode
    });
});

async function run() {
    await fusionRuntime.start();

    console.log("=== PHASE 1: GNSS AVAILABLE (10s) ===");
    // In this phase, we provide GNSS position, but NO speed/heading.
    for (let i = 0; i < 10; i++) {
        currentTime += 1000;

        useSensorStore.setState({
            gnss: {
                latitude: 37.7749 + i * 0.0001,
                longitude: -122.4194 + i * 0.0001,
                accuracy: 5,
                timestamp: currentTime
            }
        });

        for (let j = 0; j < 10; j++) {
            currentTime += 100;
            useSensorStore.setState({
                accel: { x: 0.5, y: 0.5, z: 9.81 },
                gyro: { x: 0, y: 0, z: 0 }
            });
        }
    }
    
    let lastGnssPos = positions[positions.length - 1];
    console.log(`Last GNSS-corrected State: pos=(${lastGnssPos.lat?.toFixed(6)}, ${lastGnssPos.lon?.toFixed(6)}), vel=(${lastGnssPos.vx.toFixed(2)}, ${lastGnssPos.vy.toFixed(2)})`);

    console.log("=== PHASE 2: BLACKOUT (GNSS LOST for 10s) ===");
    for (let i = 0; i < 10; i++) {
        for (let j = 0; j < 10; j++) {
            currentTime += 100;
            useSensorStore.setState({
                accel: { x: 0.5, y: 0.5, z: 9.81 }, // Continued acceleration
                gyro: { x: 0, y: 0, z: 0 }
            });
        }
    }

    let blackoutEndPos = positions[positions.length - 1];
    console.log(`Blackout End State: pos=(${blackoutEndPos.lat?.toFixed(6)}, ${blackoutEndPos.lon?.toFixed(6)}), vel=(${blackoutEndPos.vx.toFixed(2)}, ${blackoutEndPos.vy.toFixed(2)})`);

    // Validation
    const posDiffLat = Math.abs(blackoutEndPos.lat - lastGnssPos.lat);
    const posDiffLon = Math.abs(blackoutEndPos.lon - lastGnssPos.lon);
    
    if (posDiffLat > 0 || posDiffLon > 0) {
        console.log("SUCCESS: Position built forward correctly during blackout.");
    } else {
        console.log("FAIL: Position stalled during blackout.");
    }

    fusionRuntime.stop();
}

run();
