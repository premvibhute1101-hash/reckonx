"""
python_unit_tests.py
====================
Level 1 Unit & Component Integration Tests for Python DeadReckon AI Engine:
- 15-State Error-State Extended Kalman Filter (EKF)
- HMM Map Matching (Newson & Krumm 2009)
- INS Mechanization & Kinematic Physics
- Seamless GNSS Quality State Machine
- Dataset Windowing & Label Generators
"""

import unittest
import numpy as np
import math
import os
import sys
import importlib

DEADRECKON_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "deadreckon-idr", "ins_error_ai"))
if DEADRECKON_DIR not in sys.path:
    sys.path.insert(0, DEADRECKON_DIR)

_ekf = importlib.import_module("src.ekf")
NavigationEKF = _ekf.NavigationEKF

_hmm = importlib.import_module("src.hmm_map_matching")
RoadNetworkGraph = _hmm.RoadNetworkGraph
HMMMapMatcher = _hmm.HMMMapMatcher
GPSObservation = _hmm.GPSObservation
enu_to_latlon_point = _hmm.enu_to_latlon_point
latlon_to_enu_point = _hmm.latlon_to_enu_point
haversine_distance = _hmm.haversine_distance
EmissionModel = _hmm.EmissionModel
TransitionModel = _hmm.TransitionModel

_sc = importlib.import_module("src.seamless_controller")
GNSSQualityStateMachine = _sc.GNSSQualityStateMachine
NavigationMode = _sc.NavigationMode
check_innovation_gate = _sc.check_innovation_gate

_im = importlib.import_module("src.ins_mechanization")
euler_to_rotmat = _im.euler_to_rotmat
quat_to_rotmat = _im.quat_to_rotmat
quat_normalize = _im.quat_normalize
run_ins_mechanization = _im.run_ins_mechanization

config = importlib.import_module("config")


class TestPythonDeadReckon(unittest.TestCase):
    def setUp(self):
        self.ref_lat = 18.9220
        self.ref_lon = 72.8347

    # -----------------------------------------------------------------------
    # HMM Map Matching Tests
    # -----------------------------------------------------------------------
    def test_hmm_straight_road_trivial(self):
        """HMM Unit: Exact snapping onto a straight road segment."""
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)
        a_lat, a_lon = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        b_lat, b_lon = enu_to_latlon_point(400.0, 0.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("road_main", "A", "B", a_lat, a_lon, b_lat, b_lon, oneway=False)
        road_net.build_spatial_index()

        gps_points = [
            GPSObservation(lat=enu_to_latlon_point(x, 0.0, self.ref_lat, self.ref_lon)[0],
                           lon=enu_to_latlon_point(x, 0.0, self.ref_lat, self.ref_lon)[1])
            for x in [50.0, 100.0, 150.0, 200.0, 250.0]
        ]

        matcher = HMMMapMatcher(road_network=road_net)
        res = matcher.match_trace(gps_points)
        self.assertEqual(len(res.matched_lat), 5)
        self.assertTrue(all(e.startswith("road_main") for e in res.matched_edge_ids))

    def test_hmm_gps_noise_filtering(self):
        """HMM Unit: Lateral GPS noise filtered and snapped to road centerline."""
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)
        a_lat, a_lon = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        b_lat, b_lon = enu_to_latlon_point(300.0, 0.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("road_1", "A", "B", a_lat, a_lon, b_lat, b_lon)
        road_net.build_spatial_index()

        noisy_points = [
            GPSObservation(lat=enu_to_latlon_point(x, dy, self.ref_lat, self.ref_lon)[0],
                           lon=enu_to_latlon_point(x, dy, self.ref_lat, self.ref_lon)[1], accuracy=6.0)
            for x, dy in [(30.0, 8.0), (90.0, -7.0), (150.0, 9.0), (210.0, -6.0)]
        ]

        matcher = HMMMapMatcher(road_network=road_net, sigma_z=5.0)
        res = matcher.match_trace(noisy_points)
        for i in range(len(noisy_points)):
            m_east, m_north = latlon_to_enu_point(res.matched_lat[i], res.matched_lon[i], self.ref_lat, self.ref_lon)
            self.assertAlmostEqual(m_north, 0.0, delta=0.5)

    def test_hmm_parallel_road_disambiguation(self):
        """HMM Unit: Viterbi transition penalty chooses continuous highway over closer dead-end."""
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)
        h1 = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        h2 = enu_to_latlon_point(400.0, 0.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("highway", "H1", "H2", h1[0], h1[1], h2[0], h2[1], oneway=True)

        s1 = enu_to_latlon_point(50.0, 15.0, self.ref_lat, self.ref_lon)
        s2 = enu_to_latlon_point(100.0, 15.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("service_deadend", "S1", "S2", s1[0], s1[1], s2[0], s2[1], oneway=True)
        road_net.build_spatial_index()

        gps_trace = [(25.0, 1.0), (75.0, 11.0), (150.0, 0.5), (225.0, -1.0)]
        gps_obs = [
            GPSObservation(lat=enu_to_latlon_point(x, y, self.ref_lat, self.ref_lon)[0],
                           lon=enu_to_latlon_point(x, y, self.ref_lat, self.ref_lon)[1])
            for x, y in gps_trace
        ]

        matcher = HMMMapMatcher(road_network=road_net, sigma_z=5.0, beta=5.0, search_radius_m=30.0)
        res = matcher.match_trace(gps_obs)
        self.assertTrue(all(e == "highway" for e in res.matched_edge_ids))

    def test_hmm_edge_cases(self):
        """HMM Unit: Handles stationary points, zero candidates, and broken transitions."""
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)
        a = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        b = enu_to_latlon_point(100.0, 0.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("e1", "A", "B", a[0], a[1], b[0], b[1])
        road_net.build_spatial_index()

        obs = [
            GPSObservation(lat=enu_to_latlon_point(20.0, 0.0, self.ref_lat, self.ref_lon)[0],
                           lon=enu_to_latlon_point(20.0, 0.0, self.ref_lat, self.ref_lon)[1]),
            GPSObservation(lat=enu_to_latlon_point(20.0, 0.0, self.ref_lat, self.ref_lon)[0],
                           lon=enu_to_latlon_point(20.0, 0.0, self.ref_lat, self.ref_lon)[1]),
            GPSObservation(lat=enu_to_latlon_point(600.0, 600.0, self.ref_lat, self.ref_lon)[0],
                           lon=enu_to_latlon_point(600.0, 600.0, self.ref_lat, self.ref_lon)[1]),
        ]
        matcher = HMMMapMatcher(road_network=road_net, search_radius_m=30.0)
        res = matcher.match_trace(obs)
        self.assertEqual(len(res.matched_lat), 3)
        self.assertEqual(res.matched_edge_ids[0], "e1")
        self.assertEqual(res.matched_edge_ids[1], "e1")

    # -----------------------------------------------------------------------
    # 15-State EKF Tests
    # -----------------------------------------------------------------------
    def test_ekf_15_state_initialization(self):
        """EKF Unit: Verify 15-state state vector dimension and covariance diagonal."""
        ekf = NavigationEKF()
        self.assertEqual(ekf.x.shape, (15,))
        self.assertEqual(ekf.P.shape, (15, 15))
        self.assertAlmostEqual(ekf.P[0, 0], 0.1) # pos
        self.assertAlmostEqual(ekf.P[3, 3], 0.1) # vel
        self.assertAlmostEqual(ekf.P[6, 6], 0.001) # att

    def test_ekf_predict_propagation(self):
        """EKF Unit: Prediction propagates position forward using velocity state."""
        ekf = NavigationEKF()
        ekf.reset(pos_x=0.0, pos_y=0.0, vel_x=10.0, vel_y=0.0)
        ekf.predict(dt=1.0)
        self.assertAlmostEqual(ekf.position[0], 10.0, delta=0.5)
        self.assertAlmostEqual(ekf.position[1], 0.0, delta=0.5)

    def test_ekf_gnss_update_and_chi2_gating(self):
        """EKF Unit: GNSS update corrects state; large outlier rejected by Chi-Square gate."""
        ekf = NavigationEKF()
        ekf.reset(pos_x=0.0, pos_y=0.0)
        
        # Normal update
        updated = ekf.update_gnss(np.array([5.0, 0.0, 0.0]), accuracy_m=2.0)
        self.assertTrue(updated)
        self.assertGreater(ekf.position[0], 0.0)

        # Extreme outlier (500m jump) -> Rejected
        rejected = ekf.update_gnss(np.array([500.0, 500.0, 0.0]), accuracy_m=2.0)
        self.assertFalse(rejected)

    def test_ekf_ai_velocity_update(self):
        """EKF Unit: AI velocity correction updates velocity state."""
        ekf = NavigationEKF()
        ekf.reset(vel_x=2.0, vel_y=0.0)
        # AI reports vehicle is actually moving at 5.0 m/s
        ekf.update_ai_velocity(np.array([5.0, 0.0]), r_std=0.5)
        self.assertGreater(ekf.velocity[0], 2.0)

    def test_ekf_zupt_stationary(self):
        """EKF Unit: Zero Velocity Update (ZUPT) clamps velocity to 0."""
        ekf = NavigationEKF()
        ekf.reset(vel_x=1.5, vel_y=-1.0)
        ekf.update_zupt()
        self.assertAlmostEqual(ekf.velocity[0], 0.0)
        self.assertAlmostEqual(ekf.velocity[1], 0.0)
        self.assertTrue(ekf.was_zupt_active)

    def test_ekf_nhc_lateral_constraint(self):
        """EKF Unit: Non-Holonomic Constraint (NHC) suppresses cross-track velocity."""
        ekf = NavigationEKF()
        # Heading East (0 rad), but has North drift (vy = 2.0 m/s)
        ekf.reset(vel_x=10.0, vel_y=2.0)
        ekf.update_nhc(heading_rad=0.0, r_std=0.1)
        self.assertLess(ekf.velocity[1], 2.0)

    # -----------------------------------------------------------------------
    # Seamless Quality Controller & INS Kinematics Tests
    # -----------------------------------------------------------------------
    def test_seamless_controller_states(self):
        """Seamless Controller: Evaluates GOOD -> DEGRADED -> LOST hysteresis transitions."""
        sm = GNSSQualityStateMachine()
        # Initial state: GOOD GNSS
        s1 = sm.evaluate_sample(gps_ok=True, accuracy_m=5.0, sats_count=8)
        self.assertEqual(s1["mode"], NavigationMode.GOOD)
        self.assertFalse(s1["ai_active"])  # AI sleeps to save battery

        # Degraded signal
        s2 = sm.evaluate_sample(gps_ok=True, accuracy_m=18.0, sats_count=5)
        # Lost signal (outage)
        for _ in range(12):
            s_lost = sm.evaluate_sample(gps_ok=False, accuracy_m=100.0, sats_count=0)
        self.assertEqual(s_lost["mode"], NavigationMode.LOST)
        self.assertTrue(s_lost["ai_active"])  # AI wakes up

    def test_ins_mechanization_kinematics(self):
        """INS Mechanization: Euler to rotation matrix and quaternion kinematics."""
        R = euler_to_rotmat(0.0, 0.0, 0.0)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-6)

        # 90-degree yaw rotation
        R_yaw90 = euler_to_rotmat(90.0, 0.0, 0.0)
        np.testing.assert_allclose(R_yaw90[0, 1], -1.0, atol=1e-6)
        np.testing.assert_allclose(R_yaw90[1, 0], 1.0, atol=1e-6)

        # Quaternion normalization and rotation matrix conversion
        q = quat_normalize(np.array([1.0, 0.0, 0.0, 0.0]))
        R_quat = quat_to_rotmat(q)
        np.testing.assert_allclose(R_quat, np.eye(3), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
