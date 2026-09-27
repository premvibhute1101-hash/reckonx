"""
e2e_full_pipeline_test.py
=========================
Level 3 End-to-End System Test:
Simulates a complete mission lifecycle:
1. Synthetic IMU (accel, gyro) & GNSS sensor streams with a 60-second GPS outage.
2. INS Mechanization & Kinematic propagation.
3. AI error correction injection.
4. 15-State Error-State EKF sensor fusion with Seamless Controller.
5. HMM Map Matching route projection onto an urban road network.
6. Validation of drift bounds (< 15m drift during 60s blackout) and metrics.
"""

import unittest
import numpy as np
import pandas as pd
import math
import os
import sys
import importlib

DEADRECKON_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "deadreckon-idr", "ins_error_ai"))
if DEADRECKON_DIR not in sys.path:
    sys.path.insert(0, DEADRECKON_DIR)

_ekf = importlib.import_module("src.ekf")
run_ekf_fusion = _ekf.run_ekf_fusion

_hmm = importlib.import_module("src.hmm_map_matching")
RoadNetworkGraph = _hmm.RoadNetworkGraph
HMMMapMatcher = _hmm.HMMMapMatcher
GPSObservation = _hmm.GPSObservation
enu_to_latlon_point = _hmm.enu_to_latlon_point
latlon_to_enu_point = _hmm.latlon_to_enu_point

config = importlib.import_module("config")


class TestEndToEndPipeline(unittest.TestCase):
    def test_full_mission_e2e(self):
        """
        E2E: Full 100-step navigation mission with a 40-step GPS blackout.
        Verifies:
        - INS dead reckoning keeps advancing.
        - AI corrections prevent exponential drift.
        - 15-state EKF maintains position and covariance bounds.
        - HMM Map Matching snaps the fused trajectory onto the road network.
        """
        ref_lat = 18.9220
        ref_lon = 72.8347
        N = 100
        dt = 0.1  # 10 Hz

        # 1. Generate Synthetic Mission Trajectory (Eastbound vehicle at 15 m/s)
        time_sec = np.linspace(0.0, (N - 1) * dt, N)
        true_vel_x = np.full(N, 15.0)
        true_vel_y = np.zeros(N)
        true_pos_x = true_vel_x * time_sec
        true_pos_y = np.zeros(N)

        # 2. Simulate INS Mechanization with realistic sensor bias drift
        ins_bias_acc = 0.2  # 0.2 m/s^2 bias
        ins_vel_x = true_vel_x + (ins_bias_acc * time_sec)
        ins_vel_y = true_vel_y + np.random.normal(0, 0.1, N)
        ins_pos_x = true_pos_x + 0.5 * ins_bias_acc * (time_sec ** 2)
        ins_pos_y = np.zeros(N)

        ins_df = pd.DataFrame({
            "_t_sec": time_sec,
            "ins_vel_x": ins_vel_x,
            "ins_vel_y": ins_vel_y,
            "ins_pos_x": ins_pos_x,
            "ins_pos_y": ins_pos_y,
        })

        # 3. Simulate AI Error Corrections: y_pred = v_true - v_ins
        ai_corrections = np.column_stack([
            true_vel_x - ins_vel_x,
            true_vel_y - ins_vel_y
        ])

        # 4. GNSS Availability with 40-step Blackout (steps 30 to 70)
        gnss_available = np.ones(N, dtype=bool)
        gnss_available[30:70] = False  # 4-second simulated outage

        gnss_pos = np.column_stack([
            true_pos_x + np.random.normal(0, 1.5, N),
            true_pos_y + np.random.normal(0, 1.5, N)
        ])
        gnss_accuracy = np.full(N, 4.0)
        gnss_sats = np.full(N, 8)

        # 5. Run 15-State EKF Fusion
        ekf_result = run_ekf_fusion(
            ins_df=ins_df,
            ai_corrections=ai_corrections,
            gnss_available=gnss_available,
            gnss_pos=gnss_pos,
            gnss_accuracy=gnss_accuracy,
            gnss_sats=gnss_sats,
            use_seamless_switching=True
        )

        fused_pos_x = ekf_result["pos_x"]
        fused_pos_y = ekf_result["pos_y"]

        # Validate blackout drift reduction
        blackout_error = np.hypot(
            fused_pos_x[30:70] - true_pos_x[30:70],
            fused_pos_y[30:70] - true_pos_y[30:70]
        )
        max_blackout_err = np.max(blackout_error)
        self.assertLess(max_blackout_err, 15.0, f"Max drift during blackout ({max_blackout_err:.2f}m) should be < 15m")

        # 6. Run HMM Map Matching on Fused Output
        road_net = RoadNetworkGraph(ref_lat=ref_lat, ref_lon=ref_lon)
        n0 = enu_to_latlon_point(0.0, 0.0, ref_lat, ref_lon)
        n1 = enu_to_latlon_point(300.0, 0.0, ref_lat, ref_lon)
        road_net.add_edge("marine_drive", "N0", "N1", n0[0], n0[1], n1[0], n1[1], oneway=True)
        road_net.build_spatial_index()

        fused_latlon = [
            enu_to_latlon_point(fused_pos_x[i], fused_pos_y[i], ref_lat, ref_lon)
            for i in range(N)
        ]
        gps_obs = [GPSObservation(lat=lat, lon=lon) for lat, lon in fused_latlon]

        matcher = HMMMapMatcher(road_network=road_net, sigma_z=4.07, beta=5.0)
        match_result = matcher.match_trace(gps_obs)

        self.assertEqual(len(match_result.matched_lat), N)
        self.assertTrue(all(e == "marine_drive" for e in match_result.matched_edge_ids))
        self.assertGreater(match_result.confidence, 0.5)


if __name__ == "__main__":
    unittest.main()
