"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.useSensorStore = void 0;
const zustand_1 = require("zustand");
exports.useSensorStore = (0, zustand_1.create)((set) => ({
    isAcquiring: false,
    accel: { x: 0, y: 0, z: 0 },
    gyro: { x: 0, y: 0, z: 0 },
    mag: { x: 0, y: 0, z: 0 },
    gnss: {
        latitude: null,
        longitude: null,
        accuracy: null,
        altitude: null,
        speed: null,
        heading: null,
        timestamp: null,
    },
    gnssQuality: 'UNKNOWN',
    setIsAcquiring: (isAcquiring) => set({ isAcquiring }),
    setAccel: (accel) => set({ accel }),
    setGyro: (gyro) => set({ gyro }),
    setMag: (mag) => set({ mag }),
    setGNSS: (gnss) => {
        let quality = 'LOST';
        if (gnss.accuracy !== null) {
            if (gnss.accuracy < 10) {
                quality = 'GOOD';
            }
            else if (gnss.accuracy >= 10 && gnss.accuracy <= 25) {
                quality = 'DEGRADED';
            }
            else {
                quality = 'LOST';
            }
        }
        set({ gnss, gnssQuality: quality });
    },
}));
