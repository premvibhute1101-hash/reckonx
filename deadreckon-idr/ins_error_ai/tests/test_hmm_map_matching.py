"""
test_hmm_map_matching.py
========================
Unit test suite for the HMM Map Matching module (Newson & Krumm 2009).

Tests:
1. Synthetic straight road: trivial exact matching along a known segment.
2. Simulated GPS noise: noisy perturbations snapped back to the road centerline.
3. Ambiguous parallel roads: HMM favors path-connected sequence over closer disconnected road.
4. Edge cases: zero candidates, stationary points, broken network transitions.
"""

import unittest
import numpy as np
import math
import os
import sys

# Add project root to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hmm_map_matching import (
    RoadNetworkGraph,
    HMMMapMatcher,
    GPSObservation,
    haversine_distance,
    latlon_to_enu_point,
    enu_to_latlon_point,
    project_point_to_line_segment_2d,
    EmissionModel,
    TransitionModel,
)


class TestHMMMapMatching(unittest.TestCase):
    def setUp(self):
        # Anchor reference point (e.g. London / Oxford area)
        self.ref_lat = 51.7520
        self.ref_lon = -1.2577

    def test_straight_road_trivial(self):
        """
        Test 1: Synthetic straight road.
        GPS points lie along a straight road segment from node A to node B.
        """
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)

        # Segment along Easting from 0 to 500 meters
        a_lat, a_lon = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        b_lat, b_lon = enu_to_latlon_point(500.0, 0.0, self.ref_lat, self.ref_lon)

        road_net.add_edge(
            edge_id="road_main",
            u="A",
            v="B",
            u_lat=a_lat,
            u_lon=a_lon,
            v_lat=b_lat,
            v_lon=b_lon,
            oneway=False,
            name="Main St"
        )
        road_net.build_spatial_index()

        # Generate GPS points along the road at x = 50, 100, 150, 200, 250m
        gps_points = []
        for x in [50.0, 100.0, 150.0, 200.0, 250.0]:
            lat, lon = enu_to_latlon_point(x, 0.0, self.ref_lat, self.ref_lon)
            gps_points.append(GPSObservation(lat=lat, lon=lon))

        matcher = HMMMapMatcher(road_network=road_net)
        result = matcher.match_trace(gps_points)

        self.assertEqual(len(result.matched_lat), len(gps_points))
        self.assertTrue(all(edge == "road_main" for edge in result.matched_edge_ids))

        # Snapped points should match original points within millimeter precision
        for i in range(len(gps_points)):
            dist = haversine_distance(
                gps_points[i].lat, gps_points[i].lon,
                result.matched_lat[i], result.matched_lon[i]
            )
            self.assertLess(dist, 0.1)

    def test_simulated_gps_noise(self):
        """
        Test 2: Simulated GPS noise.
        Synthetic road with lateral GPS noise (+- 10m).
        The matcher should snap points back onto the road line.
        """
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)

        a_lat, a_lon = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        b_lat, b_lon = enu_to_latlon_point(300.0, 0.0, self.ref_lat, self.ref_lon)

        road_net.add_edge("seg_1", "A", "B", a_lat, a_lon, b_lat, b_lon)
        road_net.build_spatial_index()

        # Generate points with lateral noise in North direction: +10m, -8m, +12m
        xs = [20.0, 80.0, 140.0, 200.0, 260.0]
        y_noise = [10.0, -8.0, 12.0, -9.0, 7.0]

        gps_points = []
        for x, dy in zip(xs, y_noise):
            lat, lon = enu_to_latlon_point(x, dy, self.ref_lat, self.ref_lon)
            gps_points.append(GPSObservation(lat=lat, lon=lon, accuracy=5.0))

        matcher = HMMMapMatcher(road_network=road_net, sigma_z=5.0)
        result = matcher.match_trace(gps_points)

        self.assertEqual(len(result.matched_lat), len(gps_points))
        # Verify matched coordinates are projected onto y=0 (North=0)
        for i in range(len(gps_points)):
            m_east, m_north = latlon_to_enu_point(
                result.matched_lat[i], result.matched_lon[i],
                self.ref_lat, self.ref_lon
            )
            self.assertAlmostEqual(m_north, 0.0, delta=0.5)
            self.assertAlmostEqual(m_east, xs[i], delta=0.5)

    def test_ambiguous_parallel_roads(self):
        """
        Test 3: Ambiguous parallel roads.
        Road 1: Main Highway (continuous from x=0 to x=400m at y=0m).
        Road 2: Service Road (short dead-end from x=50m to x=100m at y=15m).

        A vehicle drives along Main Highway. At x=75m, GPS noise places the reading
        slightly closer to the Service Road (e.g. y=12m).
        A naive nearest-neighbor algorithm would incorrectly snap to the Service Road.
        The HMM algorithm should correctly choose Main Highway because transition
        distance for a route via the service road has huge routing penalty.
        """
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)

        # Road 1: Continuous Main Highway
        h1_lat, h1_lon = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        h2_lat, h2_lon = enu_to_latlon_point(400.0, 0.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("highway", "H1", "H2", h1_lat, h1_lon, h2_lat, h2_lon, oneway=True)

        # Road 2: Disconnected Service Road (dead end parallel alley)
        s1_lat, s1_lon = enu_to_latlon_point(50.0, 15.0, self.ref_lat, self.ref_lon)
        s2_lat, s2_lon = enu_to_latlon_point(100.0, 15.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("service_deadend", "S1", "S2", s1_lat, s1_lon, s2_lat, s2_lon, oneway=True)

        road_net.build_spatial_index()

        # Trace along Highway, but at t=1 (x=75m), GPS is at y=10m (closer to service road y=15 than highway y=0)
        gps_trace = [
            (25.0, 1.0),
            (75.0, 11.0),  # Closer to service road (dist=4m) than highway (dist=11m)
            (150.0, 0.5),
            (225.0, -1.0),
            (300.0, 0.0)
        ]

        gps_obs = [
            GPSObservation(lat=enu_to_latlon_point(x, y, self.ref_lat, self.ref_lon)[0],
                           lon=enu_to_latlon_point(x, y, self.ref_lat, self.ref_lon)[1])
            for x, y in gps_trace
        ]

        matcher = HMMMapMatcher(
            road_network=road_net,
            sigma_z=5.0,
            beta=5.0,
            search_radius_m=30.0
        )
        result = matcher.match_trace(gps_obs)

        # HMM must choose 'highway' across all timesteps due to transition connectivity
        self.assertTrue(all(e == "highway" for e in result.matched_edge_ids),
                        f"Expected highway for all steps, got {result.matched_edge_ids}")

    def test_edge_cases(self):
        """
        Test 4: Edge cases.
        - Zero candidates (far off-road point)
        - Stationary duplicate GPS points
        - Disconnected graph paths
        """
        road_net = RoadNetworkGraph(ref_lat=self.ref_lat, ref_lon=self.ref_lon)
        a_lat, a_lon = enu_to_latlon_point(0.0, 0.0, self.ref_lat, self.ref_lon)
        b_lat, b_lon = enu_to_latlon_point(100.0, 0.0, self.ref_lat, self.ref_lon)
        road_net.add_edge("e1", "A", "B", a_lat, a_lon, b_lat, b_lon)
        road_net.build_spatial_index()

        # Trace with stationary point + far off-road point (>500m away)
        p1 = enu_to_latlon_point(20.0, 0.0, self.ref_lat, self.ref_lon)
        p2 = enu_to_latlon_point(20.0, 0.0, self.ref_lat, self.ref_lon) # stationary duplicate
        p3 = enu_to_latlon_point(50.0, 0.0, self.ref_lat, self.ref_lon)
        p4 = enu_to_latlon_point(500.0, 500.0, self.ref_lat, self.ref_lon) # off-road (0 candidates)

        obs = [
            GPSObservation(lat=p1[0], lon=p1[1]),
            GPSObservation(lat=p2[0], lon=p2[1]),
            GPSObservation(lat=p3[0], lon=p3[1]),
            GPSObservation(lat=p4[0], lon=p4[1]),
        ]

        matcher = HMMMapMatcher(road_network=road_net, search_radius_m=30.0)
        result = matcher.match_trace(obs)

        self.assertEqual(len(result.matched_lat), 4)
        self.assertEqual(result.matched_edge_ids[0], "e1")
        self.assertEqual(result.matched_edge_ids[1], "e1")
        self.assertEqual(result.matched_edge_ids[2], "e1")
        self.assertTrue("offroad" in str(result.matched_edge_ids[3]))


if __name__ == "__main__":
    unittest.main()
