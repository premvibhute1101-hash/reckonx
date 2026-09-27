import * as fs from 'fs';
import { fusionRuntime } from './src/core/FusionRuntime';
import { useSensorStore } from './src/store/useSensorStore';

async function run() {
    const csvPath = 'e:\\reckonX_SIH\\idr2.csv';
    const content = fs.readFileSync(csvPath, 'utf-8');
    const lines = content.split('\n').map((l) => l.trim()).filter((l) => l.length > 0);

    let lastGnssTime = 0;
    
    // We will bypass zustand subscription and call methods directly for deterministic testing
    fusionRuntime['isRunning'] = true;
    fusionRuntime['initialLat'] = 17.64626420;
    fusionRuntime['initialLon'] = 75.94641180;
    fusionRuntime['pureIns'].position = { x: 0, y: 0, z: 0 };
    fusionRuntime['pureIns'].velocity = { x: 0, y: 0, z: 0 };

    fusionRuntime.setOnFusedDataCallback((state) => {
        console.log(`FusedVel: [${state.velocity.x.toFixed(2)}, ${state.velocity.y.toFixed(2)}, ${state.velocity.z.toFixed(2)}] ZUPT=${fusionRuntime['wasZuptActiveLastCycle']}`);
    });

    for (let i = 1; i < lines.length; i++) {
        const cols = lines[i].split(',');
        if (cols.length < 10) continue;

        const time = parseInt(cols[0]);
        const accX = parseFloat(cols[1]);
        const accY = parseFloat(cols[2]);
        const accZ = parseFloat(cols[3]);
        const gyrX = parseFloat(cols[4]);
        const gyrY = parseFloat(cols[5]);
        const gyrZ = parseFloat(cols[6]);
        const gnssLat = parseFloat(cols[7]);
        const gnssLon = parseFloat(cols[8]);
        const gnssAcc = parseFloat(cols[9]);

        // simulate IMU
        fusionRuntime['lastImuTimestamp'] = time - 100; 
        // to give dt=0.1s
        const nowBackup = Date.now;
        Date.now = () => time;
        
        fusionRuntime['handleImuUpdate']({
            accel: { x: accX, y: accY, z: accZ },
            gyro: { x: gyrX, y: gyrY, z: gyrZ }
        });

        if (time - lastGnssTime >= 1000) {
            lastGnssTime = time;
            fusionRuntime['handleGnssUpdate']({
                gnss: {
                    latitude: gnssLat,
                    longitude: gnssLon,
                    accuracy: gnssAcc,
                    speed: null,
                    heading: null
                }
            });
        }
        Date.now = nowBackup;
    }
}

run().catch(console.error);
