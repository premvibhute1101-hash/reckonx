import React, { useEffect, useState, useRef } from 'react';
import { View, Text, StyleSheet, Button, ScrollView } from 'react-native';
import { fusionRuntime, FusedState } from '../core/FusionRuntime';
import { sensorService } from '../services/SensorService';
import { useSensorStore } from '../store/useSensorStore';
import { dataLoggerService } from '../services/DataLoggerService';

export default function FusionDebugScreen() {
  const [fusedState, setFusedState] = useState<FusedState | null>(null);
  const rawSensors = useSensorStore();
  const [isAcquiring, setIsAcquiring] = useState(false);
  const [isLogging, setIsLogging] = useState(false);
  const trace = useRef<{x: number, y: number}[]>([]); // Visual trace coords
  const [traceRender, setTraceRender] = useState<{x: number, y: number}[]>([]);

  useEffect(() => {
    fusionRuntime.setOnFusedDataCallback((data) => {
      setFusedState(data);
      
      if (dataLoggerService.isCurrentlyLogging()) {
        dataLoggerService.logTick(useSensorStore.getState(), data);
      }

      // Keep a local relative trace of movement for visualization
      // Let's pretend 1 degree of lat/lon is ~ 111,000 meters. 
      // We just need a relative offset from the start.
      if (data.latitude !== null && data.longitude !== null) {
        if (trace.current.length === 0) {
          trace.current.push({ x: 0, y: 0 }); // Origin
        } else {
          // Delta in degrees * rough scale to pixels
          const prev = trace.current[trace.current.length - 1];
          // We can use the velocity to integrate a smooth path, or just use lat/lon.
          // Since lat/lon is already integrated by EKF, let's use that but scaled.
          
          // Actually, just for a simple UI trace, we don't have the original anchor here.
          // Let's just store the raw lat/lon and scale it for the SVG/View below.
          trace.current.push({ x: data.longitude, y: data.latitude });
          if (trace.current.length > 200) {
            trace.current.shift();
          }
        }
        setTraceRender([...trace.current]);
      }
    });

    return () => {
      fusionRuntime.stop();
      sensorService.stopAcquisition();
    };
  }, []);

  const toggleAcquisition = async () => {
    if (isAcquiring) {
      fusionRuntime.stop();
      sensorService.stopAcquisition();
      setIsAcquiring(false);
    } else {
      await sensorService.startAcquisition();
      await fusionRuntime.start();
      setIsAcquiring(true);
    }
  };

  const toggleLogging = async () => {
    if (isLogging) {
      dataLoggerService.stopLogging();
      setIsLogging(false);
    } else {
      await dataLoggerService.startLogging();
      setIsLogging(true);
    }
  };

  const exportLog = async () => {
    await dataLoggerService.exportLog();
  };

  const getSourceColor = (mode: string) => {
    return mode === 'GNSS' ? '#4ade80' : '#f87171'; // Green : Red
  };

  // Render a simple trace canvas
  const renderTrace = () => {
    if (traceRender.length < 2) return null;
    
    // Find min/max to normalize points into a 200x200 box
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    traceRender.forEach(p => {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    });

    const rangeX = (maxX - minX) || 1;
    const rangeY = (maxY - minY) || 1;
    const range = Math.max(rangeX, rangeY);

    return (
      <View style={styles.canvas}>
        {traceRender.map((p, i) => {
          const px = ((p.x - minX) / range) * 200;
          const py = 200 - (((p.y - minY) / range) * 200); // Invert Y for screen coords
          return (
            <View 
              key={i} 
              style={[
                styles.point, 
                { left: px, top: py, backgroundColor: i === traceRender.length - 1 ? 'blue' : 'gray' }
              ]} 
            />
          );
        })}
      </View>
    );
  };

  return (
    <ScrollView contentContainerStyle={styles.container}>
      <Text style={styles.title}>IDR Fusion Engine (Step 4)</Text>

      <Button
        title={isAcquiring ? "Stop Engine" : "Start Engine"}
        onPress={toggleAcquisition}
        color={isAcquiring ? "#ef4444" : "#3b82f6"}
      />

      <View style={{ flexDirection: 'row', justifyContent: 'space-around', width: '100%', marginVertical: 15 }}>
        <Button
          title={isLogging ? "Stop Logging" : "Start Logging"}
          onPress={toggleLogging}
          color={isLogging ? "#f59e0b" : "#10b981"}
          disabled={!isAcquiring}
        />
        {!isLogging && (
          <Button
            title="Export Last Log"
            onPress={exportLog}
            color="#8b5cf6"
          />
        )}
      </View>

      {isAcquiring && (
        <View style={styles.card}>
          <Text style={styles.label}>Raw Hardware Sensors:</Text>
          <Text style={styles.dataSmall}>Accel: [{rawSensors.accel.x.toFixed(2)}, {rawSensors.accel.y.toFixed(2)}, {rawSensors.accel.z.toFixed(2)}] m/s²</Text>
          <Text style={styles.dataSmall}>Gyro: [{rawSensors.gyro.x.toFixed(2)}, {rawSensors.gyro.y.toFixed(2)}, {rawSensors.gyro.z.toFixed(2)}] rad/s</Text>
          <Text style={styles.dataSmall}>GNSS Acc: {rawSensors.gnss.accuracy?.toFixed(1) || 'N/A'}m</Text>
        </View>
      )}

      {fusedState && (
        <View style={styles.card}>
          <View style={styles.headerRow}>
            <Text style={styles.label}>Fusion Mode:</Text>
            <View style={[styles.badge, { backgroundColor: getSourceColor(fusedState.sourceMode) }]}>
              <Text style={styles.badgeText}>{fusedState.sourceMode}</Text>
            </View>
          </View>
          <View style={styles.headerRow}>
            <Text style={styles.label}>GNSS Signal State:</Text>
            <View style={[styles.badge, { backgroundColor: fusedState.gnssState === 'GOOD' ? '#4ade80' : fusedState.gnssState === 'DEGRADED' ? '#fbbf24' : '#ef4444' }]}>
              <Text style={styles.badgeText}>{fusedState.gnssState}</Text>
            </View>
          </View>
          
          <Text style={styles.data}>Pure GNSS Lat: {rawSensors.gnss.latitude?.toFixed(7) || 'Wait...'}</Text>
          <Text style={styles.data}>Pure GNSS Lon: {rawSensors.gnss.longitude?.toFixed(7) || 'Wait...'}</Text>
          <Text style={styles.data}>Fused Lat (AI/GNSS): {fusedState.latitude?.toFixed(7) || 'Wait...'}</Text>
          <Text style={styles.data}>Fused Lon (AI/GNSS): {fusedState.longitude?.toFixed(7) || 'Wait...'}</Text>
          <Text style={styles.data}>Pure INS Lat: {fusedState.pureInsLatitude?.toFixed(7) || 'Wait...'}</Text>
          <Text style={styles.data}>Pure INS Lon: {fusedState.pureInsLongitude?.toFixed(7) || 'Wait...'}</Text>
          <Text style={styles.data}>Vel X (East): {fusedState.velocity.x.toFixed(2)} m/s</Text>
          <Text style={styles.data}>Vel Y (North): {fusedState.velocity.y.toFixed(2)} m/s</Text>
          <Text style={styles.data}>Heading: {fusedState.heading.toFixed(1)}°</Text>
          
          <View style={{marginTop: 10}}>
            <Text style={styles.label}>Estimated Biases:</Text>
            <Text style={styles.dataSmall}>Accel: [{fusedState.accelBias?.x.toFixed(3)}, {fusedState.accelBias?.y.toFixed(3)}, {fusedState.accelBias?.z.toFixed(3)}]</Text>
            <Text style={styles.dataSmall}>Gyro: [{fusedState.gyroBias?.x.toFixed(3)}, {fusedState.gyroBias?.y.toFixed(3)}, {fusedState.gyroBias?.z.toFixed(3)}]</Text>
          </View>
        </View>
      )}

      <Text style={styles.traceLabel}>Recent Path Trace (Last 200 pts)</Text>
      {renderTrace()}

    </ScrollView>
  );
}

const styles = StyleSheet.create({
  container: {
    padding: 20,
    alignItems: 'center',
    backgroundColor: '#111827',
    flexGrow: 1,
  },
  title: {
    fontSize: 24,
    fontWeight: 'bold',
    color: '#fff',
    marginBottom: 20,
    marginTop: 40,
  },
  card: {
    backgroundColor: '#1f2937',
    padding: 20,
    borderRadius: 12,
    width: '100%',
    marginVertical: 20,
  },
  headerRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 10,
  },
  label: {
    color: '#9ca3af',
    fontSize: 16,
  },
  badge: {
    paddingHorizontal: 12,
    paddingVertical: 4,
    borderRadius: 16,
  },
  badgeText: {
    color: '#fff',
    fontWeight: 'bold',
  },
  data: {
    color: '#f3f4f6',
    fontSize: 18,
    fontFamily: 'monospace',
    marginVertical: 4,
  },
  dataSmall: {
    color: '#d1d5db',
    fontSize: 14,
    fontFamily: 'monospace',
    marginVertical: 2,
  },
  traceLabel: {
    color: '#9ca3af',
    fontSize: 16,
    marginBottom: 10,
  },
  canvas: {
    width: 200,
    height: 200,
    backgroundColor: '#374151',
    borderRadius: 8,
    position: 'relative',
    overflow: 'hidden',
  },
  point: {
    position: 'absolute',
    width: 4,
    height: 4,
    borderRadius: 2,
  }
});
