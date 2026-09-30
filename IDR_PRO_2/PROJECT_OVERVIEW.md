# IDR-PoC (Inertial Dead Reckoning - Proof of Concept)

## Overview
This project is a Proof of Concept (PoC) for an **EKF Fused Navigation System** designed to maintain highly accurate continuous position and velocity estimates, even during GNSS (GPS) blackouts or degradations (e.g., in tunnels or urban canyons). 

The system leverages a hybrid architecture combining traditional **Extended Kalman Filtering (EKF)** with **Artificial Intelligence (CNN-LSTM)** to achieve robust sensor fusion on mobile edge devices.

## Key Features
- **Sensor Fusion Engine:** Integrates 10Hz IMU (Accelerometer + Gyroscope) data with GNSS signals using a 15-state Extended Kalman Filter (`EkfCore.ts`).
- **GNSS Quality State Machine:** Dynamically categorizes GNSS signals into `GOOD` (< 10m accuracy), `DEGRADED` (10-25m accuracy), or `WEAK/LOST` (> 25m accuracy) and scales the measurement trust (R-matrix) appropriately.
- **AI-Powered Dead Reckoning:** When GNSS is lost or weak, an active CNN-LSTM AI model (`ins_error_model_best.tflite`) kicks in to predict and correct velocity errors, keeping the Inertial Navigation System (INS) from diverging.
- **Edge Inference:** The AI models run directly on the mobile edge device via TensorFlow Lite (`react-native-fast-tflite`), preserving battery when not in use.
- **Mahalanobis Distance Gating:** Prevents "teleporting" and filters out outlier GNSS fixes upon signal reacquisition.

## Technology Stack
- **Frameworks & Libraries:** React Native, Expo, React
- **Languages:** TypeScript / JavaScript (Frontend & Fusion Math), Python (for AI model conversion)
- **Math & Algorithms:** `ml-matrix` for rigorous EKF matrix operations
- **AI & Machine Learning:** TensorFlow Lite for edge inference

## Architecture Highlights
The core of the system lives in `src/core/`:
- `EkfCore.ts`: The mathematical heart of the system. Manages the 15-state covariance matrices, process noise (Q), measurement noise (R), and state propagation.
- `FusionRuntime.ts`: Orchestrates the stream of IMU and GNSS data into the EKF and handles coordinates projections (Lat/Lon to ENU).
- `GnssQualityStateMachine.ts`: Handles the contextual awareness of the GNSS signal health.
- `AiMotionModel.ts`: Interfaces with the TFLite model to feed error corrections back into the EKF when GNSS is lost.

## Future Roadmap (Planned Layers)
As mapped out in the overarching technical approach, future layers may include:
- **Map Matching:** Integrating Hidden Markov Models (HMM) and OpenStreetMap to dynamically snap fused coordinates to known road networks.
- **Backend Infrastructure:** A complete MERN (MongoDB, Express, React, Node) stack to handle trip logs, offline map caching, and remote telemetry APIs.
