import * as FileSystem from 'expo-file-system/legacy';
import * as Sharing from 'expo-sharing';
import { FusedState } from '../core/FusionRuntime';

class DataLoggerService {
  private isLogging = false;
  private currentFilePath: string | null = null;

  public async startLogging() {
    if (this.isLogging) return;

    try {
      const timestamp = new Date().getTime();
      this.currentFilePath = `${FileSystem.documentDirectory}idr_log_${timestamp}.csv`;
      
      const header = "Timestamp,RawAccelX,RawAccelY,RawAccelZ,RawGyroX,RawGyroY,RawGyroZ,RawGnssLat,RawGnssLon,RawGnssAcc,FusedLat,FusedLon,FusedVelX,FusedVelY,FusedVelZ,Heading,Mode,GnssState,PureInsLat,PureInsLon\n";
      
      await FileSystem.writeAsStringAsync(this.currentFilePath, header, {
        encoding: FileSystem.EncodingType.UTF8
      });
      
      this.isLogging = true;
      console.log(`Started logging to ${this.currentFilePath}`);
    } catch (e) {
      console.error("Failed to start logging:", e);
    }
  }

  public async logTick(rawSensors: any, fusedState: FusedState) {
    if (!this.isLogging || !this.currentFilePath) return;

    try {
      const row = [
        fusedState.timestamp,
        rawSensors.accel.x.toFixed(4),
        rawSensors.accel.y.toFixed(4),
        rawSensors.accel.z.toFixed(4),
        rawSensors.gyro.x.toFixed(4),
        rawSensors.gyro.y.toFixed(4),
        rawSensors.gyro.z.toFixed(4),
        rawSensors.gnss.latitude?.toFixed(8) || '',
        rawSensors.gnss.longitude?.toFixed(8) || '',
        rawSensors.gnss.accuracy?.toFixed(2) || '',
        fusedState.latitude?.toFixed(8) || '',
        fusedState.longitude?.toFixed(8) || '',
        fusedState.velocity.x.toFixed(4),
        fusedState.velocity.y.toFixed(4),
        fusedState.velocity.z.toFixed(4),
        fusedState.heading.toFixed(2),
        fusedState.sourceMode,
        fusedState.gnssState,
        fusedState.pureInsLatitude?.toFixed(8) || '',
        fusedState.pureInsLongitude?.toFixed(8) || ''
      ].join(',') + '\n';

      // Read + append + write. (For production we'd use streams or chunked writes, but for POC this is fine)
      const currentContent = await FileSystem.readAsStringAsync(this.currentFilePath, { encoding: FileSystem.EncodingType.UTF8 });
      await FileSystem.writeAsStringAsync(this.currentFilePath, currentContent + row, { encoding: FileSystem.EncodingType.UTF8 });
      
    } catch (e) {
      console.error("Failed to write tick:", e);
    }
  }

  public stopLogging() {
    this.isLogging = false;
    console.log(`Stopped logging.`);
  }

  public async exportLog() {
    if (!this.currentFilePath) {
      alert("No log file to export.");
      return;
    }

    try {
      const isAvailable = await Sharing.isAvailableAsync();
      if (isAvailable) {
        await Sharing.shareAsync(this.currentFilePath);
      } else {
        alert("Sharing is not available on this platform");
      }
    } catch (e) {
      console.error("Failed to export log:", e);
    }
  }

  public isCurrentlyLogging() {
    return this.isLogging;
  }
}

export const dataLoggerService = new DataLoggerService();
