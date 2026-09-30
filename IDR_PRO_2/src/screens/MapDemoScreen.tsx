import React, { useEffect, useRef } from 'react';
import { StyleSheet, View } from 'react-native';
import { WebView } from 'react-native-webview';
import { fusionRuntime, FusedState } from '../core/FusionRuntime';

const htmlContent = `
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    body { padding: 0; margin: 0; }
    html, body, #map { height: 100%; width: 100%; }
  </style>
</head>
<body>
  <div id="map"></div>
  <script>
    var map = L.map('map', { zoomControl: false }).setView([0, 0], 2);
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; OpenStreetMap contributors',
      maxZoom: 19
    }).addTo(map);

    var marker = null;

    window.updatePos = function(lat, lon, mode) {
      if (isNaN(lat) || isNaN(lon)) return;
      
      var color = 'red';
      if (mode === 'GNSS_GOOD') color = '#4ade80'; // green
      else if (mode === 'GNSS_DEGRADED') color = '#fbbf24'; // orange
      else color = '#f87171'; // red

      if (!marker) {
        marker = L.circleMarker([lat, lon], {
          radius: 8,
          fillColor: color,
          color: '#fff',
          weight: 2,
          opacity: 1,
          fillOpacity: 0.8
        }).addTo(map);
        map.setView([lat, lon], 18);
      } else {
        marker.setLatLng([lat, lon]);
        marker.setStyle({ fillColor: color });
        map.setView([lat, lon], 18); // keep centered
      }
    };
  </script>
</body>
</html>
`;

export default function MapDemoScreen() {
  const webviewRef = useRef<WebView>(null);
  const lastInjectTime = useRef(0);

  useEffect(() => {
    const handleFusedData = (data: FusedState) => {
      const now = Date.now();
      if (now - lastInjectTime.current < 200) return; // Max 5 Hz

      if (data.latitude !== null && data.longitude !== null && !isNaN(data.latitude) && !isNaN(data.longitude)) {
        let modeStr = 'IDR';
        if (data.sourceMode === 'GNSS') {
          modeStr = data.gnssState === 'GOOD' ? 'GNSS_GOOD' : 'GNSS_DEGRADED';
        }
        
        const js = `window.updatePos(${data.latitude}, ${data.longitude}, '${modeStr}'); true;`;
        webviewRef.current?.injectJavaScript(js);
        lastInjectTime.current = now;
      }
    };

    fusionRuntime.setOnFusedDataCallback(handleFusedData);

    return () => {
      fusionRuntime.removeOnFusedDataCallback(handleFusedData);
    };
  }, []);

  return (
    <View style={styles.container}>
      <WebView
        ref={webviewRef}
        source={{ html: htmlContent }}
        style={styles.webview}
        scrollEnabled={false}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#fff',
  },
  webview: {
    flex: 1,
  },
});
