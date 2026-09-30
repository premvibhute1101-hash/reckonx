import React, { useState } from 'react';
import { StyleSheet, View, Text, TouchableOpacity, SafeAreaView } from 'react-native';
import App from './App';
import MapDemoScreen from './src/screens/MapDemoScreen';

export default function AppRoot() {
  const [activeTab, setActiveTab] = useState<'engine' | 'map'>('engine');

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.tabBar}>
        <TouchableOpacity 
          style={[styles.tab, activeTab === 'engine' && styles.activeTab]} 
          onPress={() => setActiveTab('engine')}
        >
          <Text style={[styles.tabText, activeTab === 'engine' && styles.activeTabText]}>Engine</Text>
        </TouchableOpacity>
        <TouchableOpacity 
          style={[styles.tab, activeTab === 'map' && styles.activeTab]} 
          onPress={() => setActiveTab('map')}
        >
          <Text style={[styles.tabText, activeTab === 'map' && styles.activeTabText]}>Map</Text>
        </TouchableOpacity>
      </View>
      
      <View style={styles.content}>
        <View style={[styles.screen, activeTab !== 'engine' && { display: 'none' }]}>
          <App />
        </View>
        <View style={[styles.screen, activeTab !== 'map' && { display: 'none' }]}>
          <MapDemoScreen />
        </View>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#111827',
  },
  tabBar: {
    flexDirection: 'row',
    backgroundColor: '#1f2937',
    padding: 10,
    marginTop: 40,
  },
  tab: {
    flex: 1,
    paddingVertical: 10,
    alignItems: 'center',
    borderRadius: 8,
  },
  activeTab: {
    backgroundColor: '#3b82f6',
  },
  tabText: {
    color: '#9ca3af',
    fontSize: 16,
    fontWeight: 'bold',
  },
  activeTabText: {
    color: '#fff',
  },
  content: {
    flex: 1,
  },
  screen: {
    flex: 1,
    width: '100%',
    height: '100%',
  }
});
