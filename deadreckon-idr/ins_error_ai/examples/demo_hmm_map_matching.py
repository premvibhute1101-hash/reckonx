"""
demo_hmm_map_matching.py
========================
Standalone runnable demonstration of HMM Map Matching on a simulated urban trajectory.
Demonstrates:
- Building a road graph with multiple interconnected segments (main avenue + cross street)
- Ingesting a noisy GPS trace
- Running HMM Map Matching (Viterbi decoding)
- Inspecting candidate projections, matched edges, and confidence scores
"""

import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.hmm_map_matching import (
    RoadNetworkGraph,
    HMMMapMatcher,
    GPSObservation,
    enu_to_latlon_point,
    latlon_to_enu_point,
    haversine_distance,
)


def run_demo():
    print("=" * 70)
    print("  HMM MAP MATCHING DEMONSTRATION (Newson & Krumm 2009)")
    print("=" * 70)

    # 1. Anchor Reference
    ref_lat = 51.7520
    ref_lon = -1.2577
    road_net = RoadNetworkGraph(ref_lat=ref_lat, ref_lon=ref_lon)

    # 2. Build road network: Main St (West to East) & Cross St (South to North)
    # Node N0: (0, 0), Node N1: (200, 0), Node N2: (400, 0) -> Main St
    # Node N3: (200, 200) -> Cross St
    n0_lat, n0_lon = enu_to_latlon_point(0.0, 0.0, ref_lat, ref_lon)
    n1_lat, n1_lon = enu_to_latlon_point(200.0, 0.0, ref_lat, ref_lon)
    n2_lat, n2_lon = enu_to_latlon_point(400.0, 0.0, ref_lat, ref_lon)
    n3_lat, n3_lon = enu_to_latlon_point(200.0, 200.0, ref_lat, ref_lon)

    road_net.add_edge("main_w_e_1", "N0", "N1", n0_lat, n0_lon, n1_lat, n1_lon, oneway=True, name="Main St Part 1")
    road_net.add_edge("main_w_e_2", "N1", "N2", n1_lat, n1_lon, n2_lat, n2_lon, oneway=True, name="Main St Part 2")
    road_net.add_edge("cross_s_n", "N1", "N3", n1_lat, n1_lon, n3_lat, n3_lon, oneway=True, name="Cross St")
    road_net.build_spatial_index()

    print(f"Road graph built with {len(road_net.edges)} directed edges and {len(road_net.graph.nodes)} nodes.\n")

    # 3. Simulate vehicle route: traveling on Main St, turning left onto Cross St
    # True vehicle trajectory coordinates in local ENU:
    true_traj_enu = [
        (50.0, 0.0),    # on Main St 1
        (120.0, 0.0),   # on Main St 1
        (190.0, 0.0),   # near junction N1
        (200.0, 50.0),  # on Cross St
        (200.0, 120.0), # on Cross St
        (200.0, 180.0), # on Cross St
    ]

    # Add simulated Gaussian GPS noise (sigma = 7.0 meters)
    np.random.seed(42)
    gps_observations = []
    print("Step-by-step Input GPS Trace vs Snapped HMM Output:")
    print("-" * 70)
    print(f"{'Step':<5} | {'Raw (East, North)':<20} | {'Matched (East, North)':<22} | {'Matched Edge':<15}")
    print("-" * 70)

    for x_true, y_true in true_traj_enu:
        x_noisy = x_true + np.random.normal(0, 6.0)
        y_noisy = y_true + np.random.normal(0, 6.0)
        lat_obs, lon_obs = enu_to_latlon_point(x_noisy, y_noisy, ref_lat, ref_lon)
        gps_observations.append(GPSObservation(lat=lat_obs, lon=lon_obs, accuracy=6.0))

    # 4. Run HMM Map Matching
    matcher = HMMMapMatcher(
        road_network=road_net,
        sigma_z=4.07,
        beta=5.0,
        max_candidates=5,
        search_radius_m=50.0,
    )
    result = matcher.match_trace(gps_observations)

    # 5. Display results
    for t in range(len(gps_observations)):
        raw_e, raw_n = latlon_to_enu_point(result.raw_lat[t], result.raw_lon[t], ref_lat, ref_lon)
        mat_e, mat_n = latlon_to_enu_point(result.matched_lat[t], result.matched_lon[t], ref_lat, ref_lon)
        edge_id = result.matched_edge_ids[t]
        print(f"{t+1:<5} | ({raw_e:6.1f}m, {raw_n:6.1f}m)    | ({mat_e:6.1f}m, {mat_n:6.1f}m)      | {edge_id:<15}")

    print("-" * 70)
    print(f"Total Log-Likelihood: {result.log_likelihood:.3f}")
    print(f"Confidence Score:     {result.confidence * 100:.1f}%\n")
    print("=" * 70)


if __name__ == "__main__":
    run_demo()
