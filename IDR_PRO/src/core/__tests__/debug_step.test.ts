// @ts-nocheck
import * as fs from 'fs';
import { EkfCore } from '../EkfCore';

describe('Debug EKF', () => {
  it('debugs rows 1 to 60', () => {
    const csvPath = 'e:\\reckonX_SIH\\test2.csv';
    const content = fs.readFileSync(csvPath, 'utf-8');
    const lines = content.split('\n').map((l: string) => l.trim()).filter((l: string) => l.length > 0);
    
    const ekf = new EkfCore();
    let initialLat: number | null = null;
    let initialLon: number | null = null;
    const R_EARTH = 6378137.0;

    let lastGnssLat: number | null = null;
    let lastGnssLon: number | null = null;
    let lastTime = 0;

    for (let i = 1; i < lines.length; i++) {
        const cols = lines[i].split(',');
        const time = parseFloat(cols[0]);
        const accX = parseFloat(cols[1]);
        const accY = parseFloat(cols[2]);
        const accZ = parseFloat(cols[3]);
        const gyrX = parseFloat(cols[4]);
        const gyrY = parseFloat(cols[5]);
        const gyrZ = parseFloat(cols[6]);
        const gnssLat = parseFloat(cols[7]);
        const gnssLon = parseFloat(cols[8]);
        const gnssAcc = parseFloat(cols[9]);

        let dt = lastTime === 0 ? 0.1 : (time - lastTime) / 1000.0;
        if (dt <= 0 || dt > 1.0) dt = 0.1;
        lastTime = time;

        if (i === 1) {
            ekf.getIns().initializeAttitude({ x: accX, y: accY, z: accZ });
        }

        ekf.predict(dt, [accX, accY, accZ], [gyrX, gyrY, gyrZ]);
        const vel = ekf.getIns().velocity;
        const att = ekf.getIns().attitude;
        const biases = ekf.getBiases();

        if (gnssLat !== lastGnssLat || gnssLon !== lastGnssLon || i === 1) {
            if (initialLat === null) {
                initialLat = gnssLat;
                initialLon = gnssLon;
            }
            lastGnssLat = gnssLat;
            lastGnssLon = gnssLon;
            const latRad = initialLat * (Math.PI / 180);
            const dx = (gnssLon - initialLon) * (Math.PI / 180) * R_EARTH * Math.cos(latRad);
            const dy = (gnssLat - initialLat) * (Math.PI / 180) * R_EARTH;

            const posBefore = { ...ekf.getPosition() };
            ekf.updateGnss([dx, dy, 0], null, gnssAcc, []);
            const posAfter = { ...ekf.getPosition() };
            console.log(`[Row ${i}] GNSS fix dx=${dx.toFixed(2)}, dy=${dy.toFixed(2)} | posBefore=(${posBefore.x.toFixed(2)}, ${posBefore.y.toFixed(2)}) | posAfter=(${posAfter.x.toFixed(2)}, ${posAfter.y.toFixed(2)}) | vel=(${vel.x.toFixed(2)}, ${vel.y.toFixed(2)}, ${vel.z.toFixed(2)}) | att=(p:${att.pitch.toFixed(3)}, r:${att.roll.toFixed(3)}, y:${att.yaw.toFixed(3)}) | accelBias=(${biases.accel.x.toFixed(4)}, ${biases.accel.y.toFixed(4)}, ${biases.accel.z.toFixed(4)})`);
        }
        const speed = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        if (speed > 0.5 && speed < 5.0) {
            console.log(`[DIVERGENCE at row ${i}] speed=${speed.toFixed(2)} | pos=(${ekf.getPosition().x.toFixed(2)}, ${ekf.getPosition().y.toFixed(2)}) | vel=(${vel.x.toFixed(2)}, ${vel.y.toFixed(2)}, ${vel.z.toFixed(2)}) | att=(p:${att.pitch.toFixed(3)}, r:${att.roll.toFixed(3)}, y:${att.yaw.toFixed(3)})`);
        }
    }
  });
});
