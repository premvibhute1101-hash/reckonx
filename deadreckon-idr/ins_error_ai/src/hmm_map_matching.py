"""
hmm_map_matching.py
===================
Hidden Markov Model (HMM) based Map Matching module for vehicle trajectory estimation,
implementing the Newson & Krumm (2009) algorithm.

Algorithm Overview:
-------------------
Given a sequence of noisy GPS/inertial position measurements z_1, z_2, ..., z_T:
1. Candidate Generation:
   For each observation z_t, find candidate road segments within search radius (e.g., 50m)
   using spatial indexing, and compute perpendicular projections r_t,i onto segments.

2. Emission Probability (Gaussian Observation Noise):
   Models the likelihood of observing GPS fix z_t given that the true vehicle position
   is candidate point r_i on the road:
       p(z_t | r_i) = (1 / (sqrt(2*pi) * sigma_z)) * exp(-0.5 * (dist(z_t, r_i) / sigma_z)^2)
   sigma_z is configurable (default 4.07m per Newson & Krumm, or adapted from GPS accuracy).

3. Transition Probability (Exponential Routing Divergence):
   Models the likelihood of transitioning from candidate r_i at t-1 to r_j at t:
       delta = | d_route(r_i, r_j) - d_great_circle(r_i, r_j) |
       p(r_i -> r_j) = (1 / beta) * exp(-delta / beta)
   where d_route is the shortest path network distance on the road graph (via Dijkstra/A*),
   and d_great_circle is the Haversine distance between candidate projections.
   beta is configurable (default 5.0m or adaptive median delta).

4. Viterbi Decoding:
   Log-space dynamic programming to find the globally optimal state sequence:
       V_t(j) = max_i [ V_{t-1}(i) + log p(r_{t-1,i} -> r_{t,j}) ] + log p(z_t | r_{t,j})
   Handles edge cases:
       - No candidates in radius (off-road / tunnel)
       - Stationary / duplicate GPS points
       - Broken / disconnected road paths (graceful log-prob penalty)

References:
-----------
Newson, P., & Krumm, J. (2009). "Hidden Markov Map Matching Through Noise and Sparseness."
ACM SIGSPATIAL International Conference on Advances in Geographic Information Systems (GIS '09).
"""

from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Any, Union
import math
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config


# ---------------------------------------------------------------------------
# Geodesic & Projection Utilities
# ---------------------------------------------------------------------------

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Compute great-circle distance between two points in meters using the Haversine formula.
    """
    R = getattr(config, "EARTH_RADIUS_M", 6371000.0)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def latlon_to_enu_point(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """
    Convert a single lat/lon to local ENU coordinates in meters relative to (lat0, lon0).
    """
    R = getattr(config, "EARTH_RADIUS_M", 6371000.0)
    lat0_rad = math.radians(lat0)
    north = math.radians(lat - lat0) * R
    east = math.radians(lon - lon0) * R * math.cos(lat0_rad)
    return east, north


def enu_to_latlon_point(east: float, north: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """
    Convert a single local ENU coordinate (east, north) in meters to (lat, lon).
    """
    R = getattr(config, "EARTH_RADIUS_M", 6371000.0)
    lat0_rad = math.radians(lat0)
    lat = lat0 + math.degrees(north / R)
    lon = lon0 + math.degrees(east / (R * math.cos(lat0_rad)))
    return lat, lon


def project_point_to_line_segment_2d(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float
) -> Tuple[float, float, float, float]:
    """
    Project point P(px, py) onto line segment AB(ax, ay) -> (bx, by) in 2D Euclidean space.

    Returns:
    --------
    proj_x, proj_y : float
        Coordinates of the closest point on segment AB.
    dist : float
        Euclidean distance from P to the projected point.
    t_clamped : float
        Fraction along segment [0.0, 1.0].
    """
    dx = bx - ax
    dy = by - ay
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq < 1e-12:
        # Segment is a single point
        dist = math.hypot(px - ax, py - ay)
        return ax, ay, dist, 0.0

    # Project vector AP onto AB
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t_clamped = max(0.0, min(1.0, t))

    proj_x = ax + t_clamped * dx
    proj_y = ay + t_clamped * dy
    dist = math.hypot(px - proj_x, py - proj_y)

    return proj_x, proj_y, dist, t_clamped


# ---------------------------------------------------------------------------
# Data Models
# ---------------------------------------------------------------------------

@dataclass
class GPSObservation:
    """Represents a single raw GPS fix."""
    lat: float
    lon: float
    timestamp: float = 0.0
    accuracy: Optional[float] = None  # 1-sigma uncertainty in meters


@dataclass
class RoadEdge:
    """
    Represents a directed or undirected edge/segment in the road network.
    """
    edge_id: Union[int, str]
    u: Union[int, str]  # Start node ID
    v: Union[int, str]  # End node ID
    u_lat: float
    u_lon: float
    v_lat: float
    v_lon: float
    length_m: float
    oneway: bool = False
    name: str = ""
    # Local ENU coordinates cached for fast projection
    u_east: float = 0.0
    u_north: float = 0.0
    v_east: float = 0.0
    v_north: float = 0.0


@dataclass
class CandidateState:
    """
    HMM state candidate for a GPS observation snapped to a road edge.
    """
    edge: RoadEdge
    projected_lat: float
    projected_lon: float
    dist_to_gps_m: float
    offset_m: float  # Distance from edge.u to projection along the edge
    proj_east: float = 0.0
    proj_north: float = 0.0


@dataclass
class MatchResult:
    """
    Final output of HMM Map Matching.
    """
    matched_lat: np.ndarray
    matched_lon: np.ndarray
    matched_edge_ids: List[Union[int, str]]
    log_likelihood: float
    confidence: float
    candidates_per_step: List[List[Dict[str, Any]]] = field(default_factory=list)
    raw_lat: np.ndarray = field(default_factory=lambda: np.array([]))
    raw_lon: np.ndarray = field(default_factory=lambda: np.array([]))


# ---------------------------------------------------------------------------
# Road Graph & Spatial Index
# ---------------------------------------------------------------------------

class RoadNetworkGraph:
    """
    Represents a routable road network with spatial candidate lookup.
    """
    def __init__(self, ref_lat: float, ref_lon: float):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.graph = nx.DiGraph()
        self.edges: List[RoadEdge] = []
        self.edge_map: Dict[Union[int, str], RoadEdge] = {}
        self.kdtree: Optional[cKDTree] = None
        self.segment_midpoints_enu: Optional[np.ndarray] = None

    def add_edge(
        self,
        edge_id: Union[int, str],
        u: Union[int, str],
        v: Union[int, str],
        u_lat: float,
        u_lon: float,
        v_lat: float,
        v_lon: float,
        oneway: bool = False,
        name: str = ""
    ) -> RoadEdge:
        """Add a road edge between node u and node v."""
        u_east, u_north = latlon_to_enu_point(u_lat, u_lon, self.ref_lat, self.ref_lon)
        v_east, v_north = latlon_to_enu_point(v_lat, v_lon, self.ref_lat, self.ref_lon)
        length_m = math.hypot(v_east - u_east, v_north - u_north)
        if length_m < 1e-3:
            length_m = haversine_distance(u_lat, u_lon, v_lat, v_lon)

        edge = RoadEdge(
            edge_id=edge_id,
            u=u,
            v=v,
            u_lat=u_lat,
            u_lon=u_lon,
            v_lat=v_lat,
            v_lon=v_lon,
            length_m=length_m,
            oneway=oneway,
            name=name,
            u_east=u_east,
            u_north=u_north,
            v_east=v_east,
            v_north=v_north,
        )

        self.edges.append(edge)
        self.edge_map[edge_id] = edge

        # Add to NetworkX graph for routing
        self.graph.add_node(u, lat=u_lat, lon=u_lon, east=u_east, north=u_north)
        self.graph.add_node(v, lat=v_lat, lon=v_lon, east=v_east, north=v_north)
        self.graph.add_edge(u, v, weight=length_m, edge_id=edge_id, edge_obj=edge)

        if not oneway:
            # Also add reverse directed edge
            rev_id = f"{edge_id}_rev" if isinstance(edge_id, str) else -int(edge_id)
            rev_edge = RoadEdge(
                edge_id=rev_id,
                u=v,
                v=u,
                u_lat=v_lat,
                u_lon=v_lon,
                v_lat=u_lat,
                v_lon=u_lon,
                length_m=length_m,
                oneway=False,
                name=name,
                u_east=v_east,
                u_north=v_north,
                v_east=u_east,
                v_north=u_north,
            )
            self.edges.append(rev_edge)
            self.edge_map[rev_id] = rev_edge
            self.graph.add_edge(v, u, weight=length_m, edge_id=rev_id, edge_obj=rev_edge)

        return edge

    def build_spatial_index(self):
        """Build KDTree index on segment midpoints for candidate querying."""
        if not self.edges:
            return
        mids = []
        for edge in self.edges:
            mid_east = 0.5 * (edge.u_east + edge.v_east)
            mid_north = 0.5 * (edge.u_north + edge.v_north)
            mids.append([mid_east, mid_north])
        self.segment_midpoints_enu = np.array(mids)
        self.kdtree = cKDTree(self.segment_midpoints_enu)

    def find_candidates(
        self,
        gps: GPSObservation,
        max_candidates: int = 5,
        search_radius_m: float = 50.0
    ) -> List[CandidateState]:
        """
        Find candidate road segments within search radius and compute projections.
        """
        if not self.edges:
            return []

        gx, gy = latlon_to_enu_point(gps.lat, gps.lon, self.ref_lat, self.ref_lon)

        # Broad phase: query KDTree within radius + max half-segment length margin
        candidate_indices = []
        if self.kdtree is not None:
            # Expand search radius slightly to account for long segments
            indices = self.kdtree.query_ball_point([gx, gy], r=search_radius_m + 150.0)
            candidate_indices = indices
        else:
            candidate_indices = range(len(self.edges))

        candidates: List[CandidateState] = []
        for idx in candidate_indices:
            edge = self.edges[idx]
            px, py, dist, t_frac = project_point_to_line_segment_2d(
                gx, gy,
                edge.u_east, edge.u_north,
                edge.v_east, edge.v_north
            )
            if dist <= search_radius_m:
                proj_lat, proj_lon = enu_to_latlon_point(px, py, self.ref_lat, self.ref_lon)
                offset_m = t_frac * edge.length_m
                candidates.append(CandidateState(
                    edge=edge,
                    projected_lat=proj_lat,
                    projected_lon=proj_lon,
                    dist_to_gps_m=dist,
                    offset_m=offset_m,
                    proj_east=px,
                    proj_north=py,
                ))

        # Sort by distance and return top k
        candidates.sort(key=lambda c: c.dist_to_gps_m)
        return candidates[:max_candidates]

    def shortest_path_distance(self, u: Union[int, str], v: Union[int, str]) -> float:
        """
        Compute shortest path distance along the graph in meters using Dijkstra.
        Returns infinity if no path exists.
        """
        if u == v:
            return 0.0
        try:
            return nx.shortest_path_length(self.graph, source=u, target=v, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return float("inf")


# ---------------------------------------------------------------------------
# Probability Models (Newson & Krumm 2009)
# ---------------------------------------------------------------------------

class EmissionModel:
    """
    Zero-mean Gaussian emission probability model:
        p(z_t | r_i) = (1 / (sqrt(2*pi) * sigma_z)) * exp(-0.5 * (dist / sigma_z)^2)
    In log-space:
        log p(z_t | r_i) = -ln(sqrt(2*pi) * sigma_z) - 0.5 * (dist / sigma_z)^2
    """
    def __init__(self, default_sigma_z: float = 4.07):
        self.default_sigma_z = default_sigma_z

    def log_emission_probability(self, candidate: CandidateState, gps: GPSObservation) -> float:
        # Use GPS accuracy if available and reasonable, else default_sigma_z
        sigma_z = self.default_sigma_z
        if gps.accuracy is not None and gps.accuracy > 1.0:
            sigma_z = max(2.0, min(gps.accuracy, 30.0))

        dist = candidate.dist_to_gps_m
        log_norm = -math.log(math.sqrt(2.0 * math.pi) * sigma_z)
        log_prob = log_norm - 0.5 * ((dist / sigma_z) ** 2)
        return log_prob


class TransitionModel:
    """
    Exponential transition probability model:
        delta = | d_route(r_i, r_j) - d_great_circle(r_i, r_j) |
        p(r_i -> r_j) = (1 / beta) * exp(-delta / beta)
    In log-space:
        log p(r_i -> r_j) = -ln(beta) - (delta / beta)
    """
    def __init__(self, beta: float = 5.0, broken_path_log_penalty: float = -50.0):
        self.beta = beta
        self.broken_path_log_penalty = broken_path_log_penalty

    def compute_route_distance(
        self,
        road_net: RoadNetworkGraph,
        c1: CandidateState,
        c2: CandidateState
    ) -> float:
        """
        Compute shortest routing distance between two candidate positions.
        """
        # Case 1: Same edge
        if c1.edge.edge_id == c2.edge.edge_id:
            if c2.offset_m >= c1.offset_m:
                # Forward motion along the edge
                return c2.offset_m - c1.offset_m
            elif not c1.edge.oneway:
                # Two-way road backward motion on same segment
                return c1.offset_m - c2.offset_m
            else:
                # Oneway road: slight backward displacement is typically GPS measurement noise
                backward_delta = c1.offset_m - c2.offset_m
                if backward_delta < 25.0:
                    return backward_delta * 1.5  # Modest directional penalty for noise jitter

        # Case 2: Different edges (or large loop on one-way)
        dist_to_u_end = max(0.0, c1.edge.length_m - c1.offset_m)
        graph_dist = road_net.shortest_path_distance(c1.edge.v, c2.edge.u)

        if math.isinf(graph_dist):
            return float("inf")

        dist_from_v_start = max(0.0, c2.offset_m)
        return dist_to_u_end + graph_dist + dist_from_v_start

    def log_transition_probability(
        self,
        road_net: RoadNetworkGraph,
        c1: CandidateState,
        c2: CandidateState,
        beta_override: Optional[float] = None
    ) -> float:
        beta = beta_override if beta_override is not None else self.beta

        # Great-circle Euclidean distance between projections
        d_gc = haversine_distance(
            c1.projected_lat, c1.projected_lon,
            c2.projected_lat, c2.projected_lon
        )

        d_route = self.compute_route_distance(road_net, c1, c2)

        if math.isinf(d_route):
            # Broken transition or disconnected component
            return self.broken_path_log_penalty

        delta = abs(d_route - d_gc)
        log_prob = -math.log(beta) - (delta / beta)
        return log_prob


# ---------------------------------------------------------------------------
# Viterbi Algorithm & Map Matcher
# ---------------------------------------------------------------------------

class HMMMapMatcher:
    """
    Full HMM Map Matching Engine based on Newson & Krumm (2009).
    """
    def __init__(
        self,
        road_network: RoadNetworkGraph,
        sigma_z: float = 4.07,
        beta: float = 5.0,
        max_candidates: int = 5,
        search_radius_m: float = 50.0,
        adaptive_beta: bool = False
    ):
        self.road_network = road_network
        self.emission_model = EmissionModel(default_sigma_z=sigma_z)
        self.transition_model = TransitionModel(beta=beta)
        self.max_candidates = max_candidates
        self.search_radius_m = search_radius_m
        self.adaptive_beta = adaptive_beta

    def match_trace(self, observations: List[GPSObservation]) -> MatchResult:
        """
        Snap a sequence of GPS observations onto the road network using Viterbi decoding.

        Parameters
        ----------
        observations : List[GPSObservation]
            Input sequence of raw GPS fixes.

        Returns
        -------
        MatchResult with matched coordinates, chosen edge IDs, and confidence metrics.
        """
        T = len(observations)
        if T == 0:
            return MatchResult(
                matched_lat=np.array([]),
                matched_lon=np.array([]),
                matched_edge_ids=[],
                log_likelihood=0.0,
                confidence=0.0,
            )

        # 1. Candidate Generation per timestep
        candidates_per_timestep: List[List[CandidateState]] = []
        for t, obs in enumerate(observations):
            cands = self.road_network.find_candidates(
                obs,
                max_candidates=self.max_candidates,
                search_radius_m=self.search_radius_m
            )
            # Edge case handling: zero candidates within radius
            if not cands:
                # Fallback: create a dummy candidate at nearest node or raw GPS point
                dummy_edge = RoadEdge(
                    edge_id=f"offroad_{t}",
                    u=f"off_{t}_u",
                    v=f"off_{t}_v",
                    u_lat=obs.lat,
                    u_lon=obs.lon,
                    v_lat=obs.lat,
                    v_lon=obs.lon,
                    length_m=0.0
                )
                cands = [CandidateState(
                    edge=dummy_edge,
                    projected_lat=obs.lat,
                    projected_lon=obs.lon,
                    dist_to_gps_m=0.0,
                    offset_m=0.0
                )]
            candidates_per_timestep.append(cands)

        # 2. Viterbi Initialization (t = 0)
        num_cands_0 = len(candidates_per_timestep[0])
        V = [np.zeros(num_cands_0, dtype=float)]
        backpointers: List[np.ndarray] = []

        for i, c_i in enumerate(candidates_per_timestep[0]):
            V[0][i] = self.emission_model.log_emission_probability(c_i, observations[0])

        # 3. Viterbi Forward Recursion (t = 1 .. T-1)
        for t in range(1, T):
            obs_prev = observations[t - 1]
            obs_curr = observations[t]
            cands_prev = candidates_per_timestep[t - 1]
            cands_curr = candidates_per_timestep[t]

            v_t = np.full(len(cands_curr), -np.inf, dtype=float)
            bp_t = np.zeros(len(cands_curr), dtype=int)

            # Check for stationary/duplicate point
            is_stationary = (
                haversine_distance(obs_prev.lat, obs_prev.lon, obs_curr.lat, obs_curr.lon) < 0.5
            )

            for j, c_j in enumerate(cands_curr):
                log_emission = self.emission_model.log_emission_probability(c_j, obs_curr)
                best_val = -np.inf
                best_i = 0

                for i, c_i in enumerate(cands_prev):
                    if is_stationary and c_i.edge.edge_id == c_j.edge.edge_id:
                        log_trans = 0.0  # Zero penalty for maintaining state when stationary
                    else:
                        log_trans = self.transition_model.log_transition_probability(
                            self.road_network, c_i, c_j
                        )

                    total_score = V[t - 1][i] + log_trans
                    if total_score > best_val:
                        best_val = total_score
                        best_i = i

                v_t[j] = best_val + log_emission
                bp_t[j] = best_i

            V.append(v_t)
            backpointers.append(bp_t)

        # 4. Termination & Backtracking
        best_last_idx = int(np.argmax(V[-1]))
        total_log_likelihood = float(V[-1][best_last_idx])

        chosen_states: List[CandidateState] = [None] * T
        chosen_states[-1] = candidates_per_timestep[-1][best_last_idx]

        curr_idx = best_last_idx
        for t in range(T - 1, 0, -1):
            curr_idx = backpointers[t - 1][curr_idx]
            chosen_states[t - 1] = candidates_per_timestep[t - 1][curr_idx]

        # 5. Build Result
        matched_lat = np.array([s.projected_lat for s in chosen_states])
        matched_lon = np.array([s.projected_lon for s in chosen_states])
        matched_edges = [s.edge.edge_id for s in chosen_states]

        # Confidence: normalize average log emission + transition
        avg_log_prob = total_log_likelihood / max(1, T)
        confidence = float(np.clip(np.exp(avg_log_prob / 10.0), 0.0, 1.0))

        debug_candidates = []
        for t in range(T):
            step_debug = []
            for c in candidates_per_timestep[t]:
                step_debug.append({
                    "edge_id": c.edge.edge_id,
                    "dist_m": c.dist_to_gps_m,
                    "proj_lat": c.projected_lat,
                    "proj_lon": c.projected_lon,
                })
            debug_candidates.append(step_debug)

        return MatchResult(
            matched_lat=matched_lat,
            matched_lon=matched_lon,
            matched_edge_ids=matched_edges,
            log_likelihood=total_log_likelihood,
            confidence=confidence,
            candidates_per_step=debug_candidates,
            raw_lat=np.array([o.lat for o in observations]),
            raw_lon=np.array([o.lon for o in observations]),
        )


# ---------------------------------------------------------------------------
# Bridge with OSMnx Graph (when osmnx is installed)
# ---------------------------------------------------------------------------

def build_road_graph_from_osmnx(G, ref_lat: float, ref_lon: float) -> RoadNetworkGraph:
    """
    Convert an OSMnx graph into a RoadNetworkGraph with spatial index.
    """
    road_net = RoadNetworkGraph(ref_lat=ref_lat, ref_lon=ref_lon)

    # Add edges from OSMnx graph
    for u, v, k, data in G.edges(keys=True, data=True):
        u_lat, u_lon = G.nodes[u]["y"], G.nodes[u]["x"]
        v_lat, v_lon = G.nodes[v]["y"], G.nodes[v]["x"]
        edge_id = data.get("osmid", f"{u}_{v}_{k}")
        if isinstance(edge_id, list):
            edge_id = str(edge_id[0])
        oneway = data.get("oneway", False)
        name = data.get("name", "")
        if isinstance(name, list):
            name = name[0]

        road_net.add_edge(
            edge_id=edge_id,
            u=u,
            v=v,
            u_lat=u_lat,
            u_lon=u_lon,
            v_lat=v_lat,
            v_lon=v_lon,
            oneway=bool(oneway),
            name=str(name) if name else "",
        )

    road_net.build_spatial_index()
    return road_net
