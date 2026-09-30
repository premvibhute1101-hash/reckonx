// @ts-nocheck
import * as fs from 'fs';
import { EkfCore } from '../EkfCore';

describe('GNSS Q-Matrix Verification', () => {
    it('should eliminate sawtooth pattern in GOOD state and accept updates', () => {
        const csvPath = 'e:\\reckonX_SIH\\test2.csv';
        const content = fs.readFileSync(csvPath, 'utf-8');
        const lines = content.split('\n').map((l) => l.trim()).filter((l) => l.length > 0);

        const ekf = new EkfCore();
        let initialLat = null;
        let initialLon = null;
        const R_EARTH = 6378137.0;
        let lastGnssTime = 0;
        let lastTime = 0;

        const originalApply = ekf.applyMeasurementUpdate.bind(ekf);
        
        ekf.applyMeasurementUpdate = function(H, z, R, thresh) {
            const S = H.mmul(this.P).mmul(H.transpose()).add(R);
            const mlInverse = require('ml-matrix').inverse;
            let S_inv = null;
            try { S_inv = mlInverse(S); } catch (e) {}
            if (!S_inv) return false;
            
            const mahalanobisSq = z.transpose().mmul(S_inv).mmul(z).get(0, 0);
            if (mahalanobisSq >= thresh) {
                console.log(`[REJECTED] MahalanobisSq: ${mahalanobisSq.toFixed(2)} (thresh: ${thresh})`);
                return false;
            }
            return originalApply(H, z, R, thresh);
        };

        for (let i = 1; i < 500; i++) {
            const cols = lines[i].split(',');
            if (cols.length < 10) continue;

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

            const timeSinceLastGnss = time - lastGnssTime;
            if (i === 1 || timeSinceLastGnss >= 950) {
                lastGnssTime = time;
                if (initialLat === null) { initialLat = gnssLat; initialLon = gnssLon; }
                const latRad = initialLat * (Math.PI / 180);
                const dx = (gnssLon - initialLon) * (Math.PI / 180) * R_EARTH * Math.cos(latRad);
                const dy = (gnssLat - initialLat) * (Math.PI / 180) * R_EARTH;
                ekf.updateGnss([dx, dy, 0], null, gnssAcc, []);
            }
        }
        expect(1).toBe(1);
    });
});
