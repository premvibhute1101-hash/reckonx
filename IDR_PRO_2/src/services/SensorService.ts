import { Accelerometer, Gyroscope, Magnetometer } from 'expo-sensors';
import * as Location from 'expo-location';
import * as FileSystem from 'expo-file-system/legacy';
import { useSensorStore } from '../store/useSensorStore';

const BUFFER_SIZE = 2000;

interface BufferSample {
  timestamp: number;
  type: 'IMU' | 'GNSS';
  data: any;
}

class SensorService {
  private buffer: BufferSample[] = [];
  private accelSubscription: any = null;
  private gyroSubscription: any = null;
  private magSubscription: any = null;
  private locationSubscription: Location.LocationSubscription | null = null;

  constructor() {
    // 10Hz = 100ms
    Accelerometer.setUpdateInterval(100);
    Gyroscope.setUpdateInterval(100);
    // 1Hz = 1000ms
    Magnetometer.setUpdateInterval(1000);
  }

  private addSampleToBuffer(sample: BufferSample) {
    if (this.buffer.length >= BUFFER_SIZE) {
      this.buffer.shift();
    }
    this.buffer.push(sample);
  }

  public async startAcquisition() {
    useSensorStore.getState().setIsAcquiring(true);

    this.accelSubscription = Accelerometer.addListener((data: any) => {
      // expo-sensors gives Accel in G's (approx 9.81 m/s^2)
      // We'll multiply by 9.81 to get m/s^2
      const ts = data.timestamp || Date.now();
      const accelInMs2 = {
        x: data.x * 9.81,
        y: data.y * 9.81,
        z: data.z * 9.81,
        timestamp: ts,
      };
      useSensorStore.getState().setAccel(accelInMs2);
      this.addSampleToBuffer({
        timestamp: ts,
        type: 'IMU',
        data: { sensor: 'accelerometer', ...accelInMs2 }
      });
    });

    this.gyroSubscription = Gyroscope.addListener((data: any) => {
      const ts = data.timestamp || Date.now();
      useSensorStore.getState().setGyro({
        x: data.x,
        y: data.y,
        z: data.z,
        timestamp: ts,
      });
      this.addSampleToBuffer({
        timestamp: ts,
        type: 'IMU',
        data: { sensor: 'gyroscope', ...data }
      });
    });

    this.magSubscription = Magnetometer.addListener((data: any) => {
      const ts = data.timestamp || Date.now();
      useSensorStore.getState().setMag({
        x: data.x,
        y: data.y,
        z: data.z,
        timestamp: ts,
      });
      this.addSampleToBuffer({
        timestamp: ts,
        type: 'IMU',
        data: { sensor: 'magnetometer', ...data }
      });
    });

    try {
      const { status } = await Location.requestForegroundPermissionsAsync();
      if (status !== 'granted') {
        console.warn('Location permission denied');
        return;
      }

      this.locationSubscription = await Location.watchPositionAsync(
        {
          accuracy: Location.Accuracy.BestForNavigation,
          timeInterval: 1000,
          distanceInterval: 0,
        },
        (location) => {
          const gnssData = {
            latitude: location.coords.latitude,
            longitude: location.coords.longitude,
            accuracy: location.coords.accuracy ?? null,
            altitude: location.coords.altitude ?? null,
            speed: location.coords.speed ?? null,
            heading: location.coords.heading ?? null,
            timestamp: location.timestamp,
          };
          useSensorStore.getState().setGNSS(gnssData);
          this.addSampleToBuffer({
            timestamp: Date.now(),
            type: 'GNSS',
            data: gnssData,
          });
        }
      );
    } catch (e) {
      console.error('Failed to start location updates', e);
    }
  }

  public stopAcquisition() {
    useSensorStore.getState().setIsAcquiring(false);

    if (this.accelSubscription) {
      this.accelSubscription.remove();
      this.accelSubscription = null;
    }
    if (this.gyroSubscription) {
      this.gyroSubscription.remove();
      this.gyroSubscription = null;
    }
    if (this.magSubscription) {
      this.magSubscription.remove();
      this.magSubscription = null;
    }
    if (this.locationSubscription) {
      this.locationSubscription.remove();
      this.locationSubscription = null;
    }
  }

  public async exportBuffer() {
    try {
      const fileUri = FileSystem.documentDirectory + 'sensor_data.json';
      const jsonString = JSON.stringify(this.buffer, null, 2);
      await FileSystem.writeAsStringAsync(fileUri, jsonString, {
        encoding: FileSystem.EncodingType.UTF8,
      });
      return fileUri;
    } catch (e) {
      console.error('Failed to export buffer', e);
      throw e;
    }
  }
}

export const sensorService = new SensorService();
