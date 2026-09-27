# ReckonX - Central Architectural Brain

## 1. Project Overview
- **Project Name & Purpose**: ReckonX is a robust, resilient navigation and telematics application designed to provide real-time routing and position tracking using GPS, sensor fusion, and dead reckoning for environments with intermittent connectivity.
- **Target Users/Actors**: Local Operators, Telematics Drivers, and Autonomous System Monitors.
- **Core Principles & Integrity Standards**:
  - **No Fake Data**: The application relies strictly on real hardware measurements (accel, gyro, compass, GPS).
  - **Fail-safe Logic**: Never silently fall back between routing modes (e.g., if a Bike route fails, do not return a Car route).
  - **Chronological Integrity**: Never accept an older cached GPS event as newer.
  - **Coordinate Sanity**: Strict boundary checking (`LocationService.isValidCoordinate`) to reject invalid lat/lng inputs.
- **Core Features**:
  - Live GPS and Sensor Tracking (`LocationService`, `SensorService`).
  - Dead Reckoning with ZUPT (`DeadReckoningEngine`).
  - Heading Fusion combining GNSS, Compass, and Gyro (`OrientationService`).
  - Profile-aware dynamic routing via OSRM (`RouteService`).
  - Offline POI search and tile caching for network outages.
- **Major Technologies**:
  - **Frontend**: React ^19.2.8, React Router v7
  - **Build/Tooling**: Vite ^8.2.2, TypeScript ~6.0.2, Oxlint
  - **Styling**: Tailwind CSS ^4.3.3
  - **Maps**: Leaflet ^1.9.4
  - **Icons**: Lucide React

## 2. Project Architecture

### Main System Data Flow
```text
[Device Hardware] 
       │
       ├─> (Geolocation API) ───────> LocationService ─────┐
       │                                                   │
       ├─> (DeviceMotion API) ──────> SensorService ───────┼─> NavigationContext (State Engine)
       │                                                   │         │                     │
       └─> (DeviceOrientation API) ─> SensorService ───────┘         │                     v
                                                                      v               BackendService
                                                             UI Components                 │
                                                             (MapView, HUD)                v
                                                                                     Backend Server
                                                                                  (Fastify + SQLite)
```

### Secondary Data Flows (Routing, DR & Backend Sync)
```text
NavigationContext
       │
       ├─> RouteService ────> (Network) ────> OSRM API (Car/Bike/Walking)
       │
       ├─> DeadReckoningEngine / OrientationService / AIErrorCorrectionService
       │     (Fuses IMU + GNSS + In-Browser TFLite AI for position & velocity correction)
       │
       └─> BackendService ───> (HTTP Async) ──> Fastify API ──> Prisma / SQLite DB
             (Non-blocking persistence of telemetry points & trip sessions)
```

**Key Architectural Decisions**:
- **Centralized Context**: `NavigationContext.tsx` handles all state, providing a single source of truth and preventing race conditions using refs and abort controllers.
- **Service Layering**: Logic is strictly separated into independent stateless services (`LocationService`, `SensorService`, `RouteService`) to keep the Context purely for orchestration.
- **Operational Matrix**: System operates in 4 distinct scenarios (GPS ON/OFF + Net ON/OFF) defined in `OperationalMatrixScenario`, automatically adjusting its reliance on Dead Reckoning and Offline Caches.

## 3. File/Service Responsibilities

- `src/services/LocationService.ts`: Manages GPS watcher, strictly validates coordinates, reverse geocodes via Nominatim, and provides offline POI search fallback.
- `src/services/SensorService.ts`: Interfaces with browser APIs (`DeviceMotionEvent`, `DeviceOrientationEvent`, `Magnetometer`) to stream real hardware data.
- `src/services/RouteService.ts`: Concurrency-controlled, cache-backed routing service for Car, Bike, and Walking profiles using OSRM. Enforces strict profile matching.
- `src/services/deadReckoningEngine.ts`: Implements IMU linear acceleration integration and Zero Velocity Update (ZUPT) algorithms to suppress drift when stationary.
- `src/services/OrientationService.ts`: Fuses magnetometer, GNSS track, and gyro rate to provide a smoothed, accurate heading using shortest-path angular interpolation.
- `src/services/BackendService.ts`: Non-blocking HTTP sync client with retry logic and graceful offline fallback for data persistence.
- `src/context/NavigationContext.tsx`: The heart of the application; orchestrates state, binds services together, and calculates the active operational scenario.
- `src/components/MapView.tsx`: Renders the Leaflet map, tracking vehicle position, rendering active routes, and handling tile caching.
- `backend/`: Lightweight Node.js/Fastify + TypeScript backend service with Prisma ORM and SQLite database for storing sessions, batched telemetry, and caching OSRM routes.

## 4. State Management
- **Where**: `src/context/NavigationContext.tsx`
- **Shape**: State is logically segmented into multiple `useState` hooks:
  - `currentLocation` (`CurrentLocationData`)
  - `realSensors` (`RealSensorData`)
  - `routeState` (`RouteState`)
  - `telemetry` (`TelemetryData`)
  - `sensorStatus` (`SensorStatus`)
  - `aiCorrectionStatus` (`'active' | 'fallback' | 'unavailable'`)
  - `backendConnectionStatus` (`'connected' | 'offline' | 'error'`)
- **Updates**: Through exposed context functions (e.g., `calculateDynamicRoute`, `acquireLiveLocation`). 
- **Ref Usage**: Uses `useRef` heavily (e.g., `routeStateRef`, `activeRouteAbortControllerRef`, `routeRequestIdRef`) to avoid recreation loops and handle out-of-order network responses.

## 5. Core Algorithms
- **ZUPT (Zero Velocity Update)**: Suppresses dead reckoning drift when the vehicle is stationary. Condition: `|acceleration| < 0.25 m/s²` for > 0.5 seconds. Found in `DeadReckoningEngine`.
- **Heading Fusion**: Blends sources based on speed in `OrientationService`:
  - > 5 km/h: 70% GNSS Track + 30% Trajectory Bearing.
  - 2-5 km/h: 50% Compass + 50% Trajectory.
  - <= 2 km/h: 100% Compass.
- **Shortest-Path Angular Interpolation**: Normalizes headings to `[0, 360)` and calculates the shortest rotational arc to prevent wrap-around spinning glitches.

## 6. Dependencies & Data Integrity Standards
- **External APIs**: 
  - Nominatim for Geocoding (Debounced at 400ms, LRU cached).
  - OSRM for Routing (Primary and fallback nodes).
- **Integrity Rules**:
  - `LocationService.isValidCoordinate(lat, lng)` must be called before processing any position.
  - `isSecureContext()` checks are enforced for sensor access.

## 7. Critical Invariants / Rules AI Agents Must Never Break
1. **Never silently fall back between routing modes**. If a user requests a 'bike' route and it fails, throw an error. Do NOT return a 'car' route (`RouteService.ts`).
2. **Never accept older cached GPS events as newer**. Always verify chronological integrity `if (ts < lastAcceptedTimestamp) return;` (`LocationService.ts`).
3. **Always respect coordinate ordering conventions**. OSRM API expects/returns `[lng, lat]`. Leaflet and the internal app state expect `[lat, lng]`. You must strictly map coordinates during API ingestion.
4. **Do not put routing calculations into render loops**. Use `activeRouteAbortControllerRef` and `routeRequestIdRef` in `NavigationContext` to prevent race conditions during rapid state changes.
5. **AI correction must never run without a fallback path**. `AIErrorCorrectionService.correct()` returns `null` on any failure. The caller (NavigationHudPage 10Hz ticker) MUST check for `null` and continue with raw DR output. Never use the AI result as the sole path.
6. **AI correction applies ADDITION, not subtraction**. `corrected_vel = ins_vel + predicted_err`. Getting this backwards silently inverts every correction.
7. **Do not re-load the AI model on re-renders**. `AIErrorCorrectionService.loadModel()` is called exactly once in a `useEffect(..., [])` in `NavigationProvider`. Never call it from component renders or repeating effects.

## 8. Verification & Test Procedures
1. **Sensor Integration Test**: Access the app on a mobile device, enable sensors, and verify `TelemetryChart` displays live Accel/Gyro data. Place the device flat and verify ZUPT triggers (drift stops).
2. **Routing Profile Test**: Request a route in 'Walking' mode across a major highway. Ensure it routes around the highway or fails properly without using the highway (Car mode fallback).
3. **Offline Fallback Test**: Disconnect the network, query "Mumbai" in the search bar, and verify the offline POI database returns local results.
4. **AI Model Fallback Test**: Open the HUD page in a desktop browser (no DeviceMotion). Confirm the AI badge shows `AI —` (unavailable), the app fully navigates with raw DR, and no console errors are thrown.
5. **AI Correction Active Test**: Open the HUD on a physical mobile device, grant all sensor permissions, drive or walk for 2+ seconds (to fill the 20-sample buffer). Confirm the badge transitions: `AI —` → `AI ⋯` (model loaded, buffer filling) → `AI XX%` (active with confidence). Export the session CSV and verify `AI_Corrected_VelX_ms` and `AI_Confidence` columns are populated.
6. **CSV Export Integrity Test**: Start a session, collect ≥5 GNSS + ≥5 DR points, export CSV. Verify 18 columns are present, GNSS rows have `GPS_GroundTruth_Lat/Lng`, DR rows have `Raw_DR_VelX_ms` values, and AI columns are blank only for GNSS rows.
7. **Backend Persistence & Offline Fallback Test**:
   - Start backend server (`cd backend && npm run dev`). Open HUD page. Observe top badge shows `API Sync` (connected). Start tracking session and record points. Complete session. Verify points and session appear in SQLite (`GET http://localhost:3001/api/sessions`).
   - Stop backend server. Observe HUD badge transitions to `Local Only` (offline). Verify live navigation, dead reckoning, and TFLite AI correction continue seamlessly with zero errors or UI blocking.
8. **Real-Time Physical Mobile Device Sensor Fusion Test**:
   - Connect smartphone to same WiFi network as host PC.
   - Run `npm run dev` on PC — terminal automatically displays network IP (`https://<local-ip>:5173`) and an ASCII QR code.
   - Scan QR code with phone camera or navigate to `https://<local-ip>:5173`.
   - Accept self-signed SSL warning (`Advanced` → `Proceed`).
   - Tap `Grant Permissions & Open Map` or open `Mobile Test` modal in HUD. Grant `DeviceMotion` & `DeviceOrientation` permissions.
   - Walk 5–10 steps holding smartphone. Verify live 3-axis Accel/Gyro data stream, live ZUPT stop detection, and AI badge transition (`AI —` → `AI ⋯` → `AI XX%`).

## 9. Build & Run Instructions
- **One-Click Windows Launcher**: Double-click `start-reckonx.bat` (or run `run.bat`) to automatically spin up both backend and frontend servers in separate windows.
- **Frontend Dev Server (with Terminal QR Code & SSL)**: `npm run dev` (listens on `0.0.0.0:5173`, prints QR code and HTTPS URLs)
- **Terminal QR Code Helper**: `npm run qr` (or `node scripts/generate-qr.js`)
- **Backend Dev Server**: `cd backend && npm run dev` (port 3001)
- **Backend DB Setup**: `cd backend && npx prisma db push`
- **Production Build**: `npm run build`
- **Linting**: `npm run lint`
- **Dependencies for AI model**: `npm install @tensorflow/tfjs @tensorflow/tfjs-tflite --legacy-peer-deps`
- **HTTPS & Mobile Testing Dependencies**: `@vitejs/plugin-basic-ssl`, `qrcode-terminal`
- **WASM Note**: `@tensorflow/tfjs-tflite` loads WASM binaries at runtime from CDN. The dev server adds `COOP/COEP` headers (in `vite.config.ts`) for SharedArrayBuffer support. Production deployment must also set these headers.
- **AI model fallback**: If `@tensorflow/tfjs-tflite` WASM is unavailable (network blocked, old browser), `aiCorrectionStatus` stays `'unavailable'` and raw DR continues unaffected.

## 10. AI QUICK CONTEXT
**Project**: ReckonX | **Stack**: React 19, TypeScript, Vite, Tailwind 4, Leaflet, Fastify, Prisma, SQLite
**Architecture**: Sensor Fusion & Navigation State Engine -> UI & Non-blocking Backend Sync
**Entry**: `src/main.tsx` -> `src/App.tsx` -> `src/context/NavigationContext.tsx`
**Key Files**:
- `LocationService.ts`: Strict GPS & Geocode management.
- `RouteService.ts`: Profile-strict OSRM routing.
- `OrientationService.ts` / `deadReckoningEngine.ts`: Sensor math and ZUPT algorithms.
- `BackendService.ts`: Non-blocking HTTP sync client for backend persistence & offline status tracking.
- `backend/src/index.ts`: Fastify backend server providing `/api/telemetry`, `/api/sessions`, `/api/routes` endpoints.
- `scripts/generate-qr.js`: CLI script that auto-detects local network IP and prints ASCII QR code for physical mobile device testing over HTTPS.
- `vite.config.ts`: Vite setup configured with `host: true` (0.0.0.0) and `@vitejs/plugin-basic-ssl` for mobile sensor access.
- `MobileTestGuideModal.tsx`: Interactive modal component providing step-by-step physical device testing setup, HTTPS SSL acceptance, and iOS/Android sensor permission steps.
- `NavigationContext.tsx`: Concurrency-safe central state. Loads AI model once on mount. Pushes IMU samples into AIErrorCorrectionService buffer on each DeviceMotion event. Manages `backendConnectionStatus`.
- `AIErrorCorrectionService.ts`: **In-browser TFLite AI error correction.** Loads `ins_error_model.tflite` once, maintains a 20-sample IMU FIFO, runs inference in the HUD 10Hz ticker, falls back to null on any failure.
- `NavigationHudPage.tsx`: 10Hz DR ticker — calls `AIErrorCorrectionService.correct()` after each `DeadReckoningEngine.stepKinematics()` call, updates `aiCorrectionStatus` via context, displays `API Sync` / `Local Only` backend status badge, and opens `MobileTestGuideModal`.
- `logExportService.ts`: Exports CSV with 18 columns including raw DR, AI-corrected velocity, AI confidence, and GPS ground truth for post-analysis.
- `public/deadreckon-idr/ins_error_model.tflite`: TFLite model for in-browser AI dead reckoning correction.
- `public/deadreckon-idr/metadata.json`: **Read this before any model integration work** — defines tensor shapes, normalization constants, and the CRITICAL sign convention.
**Core Features**: Live GPS Tracking, Dead Reckoning, Multi-profile Routing, Offline Fallback, AI IMU Error Correction (TFLite in-browser), Session & Telemetry Backend Persistence, Physical Device QR Code & SSL Testing Setup.
**State**: `aiCorrectionStatus: 'active' | 'fallback' | 'unavailable'`, `backendConnectionStatus: 'connected' | 'offline' | 'error'`.
**Invariants**: NEVER fallback routing profiles silently. Maintain `[lng, lat]` vs `[lat, lng]` mappings. Ensure chronological ordering of GPS events. AI correction uses ADDITION (`corrected = ins + predicted_err`) NOT subtraction. AI model inference stays 100% in-browser. Backend operates in non-blocking offline-first mode. Physical device testing requires HTTPS or secure context for iOS Safari & Android Chrome motion sensors.

---

## 11. AI Dead Reckoning Model (`deadreckon-idr`)

### Model Purpose
A trained Conv1D-LSTM hybrid neural network that predicts **INS velocity error** in ENU (East-North-Up) coordinates. It acts as an additional error-correction layer on top of the classical `DeadReckoningEngine.stepKinematics()` pipeline. It does NOT replace dead reckoning — it corrects it.

### File Layout After Separation (Part 0)
```
ReckonX/                               ← App repo root
├── public/
│   └── deadreckon-idr/                ← Browser-served model assets (Vite static)
│       ├── ins_error_model.tflite     ← Trained model (TFLite Float32, 315 KB)
│       ├── normalization_stats.npz    ← Training z-score constants (binary, loaded at runtime)
│       └── metadata.json             ← Tensor contract: shapes, units, normalization, sign convention
│
└── deadreckon-idr/                    ← Python training project (NOT shipped to browser)
    ├── ins_error_ai/                  ← Python package
    │   ├── config.py                  ← Source of truth for all hyperparameters
    │   ├── src/                       ← Training, evaluation, export scripts
    │   │   ├── model.py               ← Network architecture definition
    │   │   ├── train.py               ← Training run (generates normalization_stats.npz)
    │   │   ├── inference.py           ← Python reference inference implementation
    │   │   └── export_tflite.py       ← Converts .keras → .tflite
    │   └── outputs/
    │       └── models/                ← Original .tflite + .npz (Python pipeline reference)
    └── Data/                          ← Empty — dataset moved to ml-training/

E:\ml-training\
└── deadreckon-idr-dataset/            ← Sibling folder OUTSIDE the app repo
    ├── IO-VNBD/                       ← Raw benchmark dataset (IO-VNBD S/V files)
    ├── processed/                     ← Labeled training CSVs (*_labeled.csv, ~73 sessions)
    ├── cache/                         ← Preprocessed numpy JSON caches
    ├── training-logs/                 ← training_log.csv (loss curves per epoch)
    ├── plots/                         ← Trajectory diagnostic plots
    ├── results/                       ← Evaluation metrics
    ├── ins_error_model_best.keras     ← Best checkpoint (training artifact)
    └── ins_error_model_final.keras    ← Final checkpoint (training artifact)
```

### Model Tensor Contract (from `metadata.json`)
| Tensor | Shape | Description |
|--------|-------|-------------|
| `imu_window` (input 1) | `[1, 20, 6]` | 2-second rolling buffer of IMU samples at 10 Hz |
| `ins_state` (input 2) | `[1, 2]` | INS-estimated `[vel_x, vel_y]` at window end (m/s ENU) |
| `predicted_error` (output) | `[1, 2]` | Predicted `[err_vel_x, err_vel_y]` (m/s ENU) |

**IMU feature order (6 channels per timestep):** `acc_x, acc_y, acc_z` (m/s², gravity REMOVED), `gyro_yaw, gyro_pitch, gyro_roll` (rad/s)

### Normalization (z-score, apply BEFORE inference)
```
X_imu_normalized   = (X_imu   - imu_mean)   / imu_std
X_state_normalized = (X_state - state_mean) / state_std
output_real        = output_normalized * y_std + y_mean
```
Exact constants are embedded in `public/deadreckon-idr/metadata.json` AND `normalization_stats.npz`.

### ⚠️ CRITICAL Sign Convention
```
corrected_vel = ins_vel + predicted_err    ← ADDITION, not subtraction
```
If the model outputs `err_vel_x = +5 m/s` → the INS is 5 m/s too slow → add `+5` to correct.

### Critical Invariants for Model Integration
1. **Do not invoke the model until the rolling buffer has exactly 20 samples.** Use a FIFO with `maxlen=20`; check `length === 20` before calling inference.
2. **Always identify TFLite input tensors by name** (`imu_window`, `ins_state`), never by positional index — TFLite may order them alphabetically.
3. **Gravity must be removed from accelerometer readings** before feeding to the model. Use `event.acceleration` (gravity-compensated), NOT `event.accelerationIncludingGravity`.
4. **Device axes must be rotated to ENU frame** using device orientation angles before the model sees them. The Python reference is `ins_mechanization.py`.
5. **Replace NaN/Inf with 0.0** before normalization and after denormalization. A zeroed sample is safer than NaN propagation through the LSTM.
6. **Never retrain and hot-swap the model without updating `normalization_stats.npz` AND `metadata.json`** simultaneously — stale normalization silently corrupts corrections.
7. **AI corrects velocity, not position directly.** The corrected velocity is then integrated by the EKF or the DR engine. Do not apply the correction as a raw position jump.
