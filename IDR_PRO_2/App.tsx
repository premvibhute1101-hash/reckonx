import React from 'react';
import { StyleSheet, View } from 'react-native';
import FusionDebugScreen from './src/screens/FusionDebugScreen';

export default function App() {
  return (
    <View style={styles.container}>
      <FusionDebugScreen />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#fff',
  },
});
