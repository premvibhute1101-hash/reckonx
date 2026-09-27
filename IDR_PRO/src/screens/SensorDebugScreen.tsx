import React, { useEffect } from 'react';
import { View, Text, StyleSheet, Button, Alert, ScrollView } from 'react-native';
import * as Location from 'expo-location';
import { useSensorStore } from '../store/useSensorStore';
import { sensorService } from '../services/SensorService';

export const SensorDebugScreen = () => {
  const { isAcquiring, accel, gyro, mag, gnss, gnssQuality } = useSensorStore();

  useEffect(() => {
    (async () => {
      const { status: fgStatus } = await Location.requestForegroundPermissionsAsync();
      if (fgStatus !== 'granted') {
        Alert.alert('Permission to access location was denied');
      }
    })();
  }, []);

  const handleStartStop = () => {
    if (isAcquiring) {
      sensorService.stopAcquisition();
    } else {
      sensorService.startAcquisition();
    }
  };

  const handleExport = async () => {
    try {
      const uri = await sensorService.exportBuffer();
      Alert.alert('Export Successful', `Data saved to: ${uri}`);
    } catch (e) {
      Alert.alert('Export Failed', 'An error occurred while exporting data.');
    }
  };

  return (
    <ScrollView contentContainerStyle={styles.container}>
      <Text style={styles.title}>IDR-POC Sensor Debug</Text>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Accelerometer (m/s²)</Text>
        <Text>x: {accel.x.toFixed(4)}</Text>
        <Text>y: {accel.y.toFixed(4)}</Text>
        <Text>z: {accel.z.toFixed(4)}</Text>
      </View>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Gyroscope (rad/s)</Text>
        <Text>x: {gyro.x.toFixed(4)}</Text>
        <Text>y: {gyro.y.toFixed(4)}</Text>
        <Text>z: {gyro.z.toFixed(4)}</Text>
      </View>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Magnetometer (μT)</Text>
        <Text>x: {mag.x.toFixed(4)}</Text>
        <Text>y: {mag.y.toFixed(4)}</Text>
        <Text>z: {mag.z.toFixed(4)}</Text>
      </View>

      <View style={styles.section}>
        <Text style={styles.sectionTitle}>GNSS</Text>
        <Text>Lat: {gnss.latitude ?? 'N/A'}</Text>
        <Text>Lon: {gnss.longitude ?? 'N/A'}</Text>
        <Text>Accuracy (m): {gnss.accuracy ?? 'N/A'}</Text>
        <Text>Speed (m/s): {gnss.speed ?? 'N/A'}</Text>
        <Text>Heading (deg): {gnss.heading ?? 'N/A'}</Text>
        <Text>Timestamp: {gnss.timestamp ?? 'N/A'}</Text>
        <Text style={styles.qualityLabel}>Quality: {gnssQuality}</Text>
      </View>

      <View style={styles.buttonContainer}>
        <Button 
          title={isAcquiring ? "Stop Acquisition" : "Start Acquisition"} 
          onPress={handleStartStop} 
        />
        <View style={{ height: 10 }} />
        <Button 
          title="Export Buffer as JSON" 
          onPress={handleExport} 
          color="#0066cc"
        />
      </View>
    </ScrollView>
  );
};

const styles = StyleSheet.create({
  container: {
    padding: 20,
    paddingTop: 50,
    backgroundColor: '#fff',
    flexGrow: 1,
  },
  title: {
    fontSize: 24,
    fontWeight: 'bold',
    marginBottom: 20,
    textAlign: 'center',
  },
  section: {
    marginBottom: 20,
    padding: 10,
    backgroundColor: '#f5f5f5',
    borderRadius: 8,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '600',
    marginBottom: 5,
  },
  qualityLabel: {
    marginTop: 5,
    fontWeight: 'bold',
    color: '#d35400',
  },
  buttonContainer: {
    marginTop: 10,
    paddingBottom: 40,
  }
});
