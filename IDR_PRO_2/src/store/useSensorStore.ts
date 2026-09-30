import { create } from 'zustand';

export interface IMUData {
  x: number;
  y: number;
  z: number;
  timestamp?: number;
}

export interface GNSSData {
  latitude: number | null;
  longitude: number | null;
  accuracy: number | null; // horizontal accuracy in meters
  altitude: number | null;
  speed: number | null;
  heading: number | null;
  timestamp: number | null;
}

export type GNSSQuality = 'GOOD' | 'DEGRADED' | 'LOST' | 'UNKNOWN';

interface SensorState {
  isAcquiring: boolean;
  accel: IMUData;
  gyro: IMUData;
  mag: IMUData;
  gnss: GNSSData;
  gnssQuality: GNSSQuality;
  
  setIsAcquiring: (isAcquiring: boolean) => void;
  setAccel: (data: IMUData) => void;
  setGyro: (data: IMUData) => void;
  setMag: (data: IMUData) => void;
  setGNSS: (data: GNSSData) => void;
}

export const useSensorStore = create<SensorState>((set) => ({
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
    let quality: GNSSQuality = 'LOST';
    if (gnss.accuracy !== null) {
      if (gnss.accuracy < 10) {
        quality = 'GOOD';
      } else if (gnss.accuracy >= 10 && gnss.accuracy <= 25) {
        quality = 'DEGRADED';
      } else {
        quality = 'LOST';
      }
    }
    set({ gnss, gnssQuality: quality });
  },
}));
