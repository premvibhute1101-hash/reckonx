import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'
import basicSsl from '@vitejs/plugin-basic-ssl'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), tailwindcss(), basicSsl()],
  define: {
    // Shim process.env for CommonJS packages like long.js / protobufjs used by TF.js
    'process.env': {},
  },
  resolve: {
    alias: {
      // Alias tfjs-tflite to its standalone Flat ESM bundle (prevents broken unbundled task client imports)
      '@tensorflow/tfjs-tflite': '@tensorflow/tfjs-tflite/dist/tf-tflite.fesm.js',
    },
  },
  optimizeDeps: {
    include: ['long'],
  },
  server: {
    host: true, // Exposes server to local network (0.0.0.0) for physical mobile testing
    port: 5173,
    headers: {
      // Allow cross-origin Leaflet map tiles and APIs while maintaining isolated context
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'credentialless',
    },
  },
  assetsInclude: ['**/*.tflite'],
})
