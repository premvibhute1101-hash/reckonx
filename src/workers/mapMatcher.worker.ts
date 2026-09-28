/**
 * mapMatcher.worker.ts
 * =====================
 * Offline HMM-based Map Matching Web Worker implementing the Newson & Krumm (2009)
 * algorithm ported and enhanced for Dead Reckoning (DR) spans with dynamic uncertainty,
 * robust candidate projection, continuity preservation, and plausibility filtering.
 *
 * Runs in a dedicated background worker thread with IndexedDB access to 'IDR_Road_Graph_DB'.
 */

import { RoadGraphCacheService, type RoadGraphCell, type RoadEdge, type RoadNode } from '../services/roadGraphCacheService';
import { haversineDistance } from '../services/ekf/OutputStabilizer';
import { projectPointOnSegment } from '../utils/routeProgress';
import type { RecordedGPSPoint } from '../services/api/trackingService';

export interface MatchSuccessResult {
  matched: true;
  points: { lat: number; lng: number }[];
}

export interface MatchFailureResult {
  matched: false;
  reason: string;
}

export type MapMatchResult = MatchSuccessResult | MatchFailureResult;

export interface CandidateState {
  edge: RoadEdge;
  projectedLat: number;
  projectedLng: number;
  distToGpsM: number;
  offsetM: number;
}

// ---------------------------------------------------------------------------
// Min Priority Queue for Dijkstra
// ---------------------------------------------------------------------------
class PriorityQueue<T> {
  private items: { item: T; priority: number }[] = [];

  push(item: T, priority: number) {
    this.items.push({ item, priority });
    this.items.sort((a, b) => a.priority - b.priority);
  }

  pop(): T | undefined {
    return this.items.shift()?.item;
  }

  isEmpty(): boolean {
    return this.items.length === 0;
  }
}

// ---------------------------------------------------------------------------
// In-Memory Graph & Dijkstra Routing
// ---------------------------------------------------------------------------
export class MemoryRoadGraph {
  edges: Map<string, RoadEdge> = new Map();
  nodes: Map<string, RoadNode> = new Map();
  adjacency: Map<string, { toNode: string; weight: number; edgeId: string }[]> = new Map();

  addCell(cell: RoadGraphCell) {
    for (const node of cell.nodes) {
      this.nodes.set(node.id, node);
    }
    for (const edge of cell.edges) {
      if (!this.edges.has(edge.id)) {
        this.edges.set(edge.id, edge);

        // Add forward edge in adjacency
        this.addAdjacencyEdge(edge.fromNodeId, edge.toNodeId, edge.lengthMeters, edge.id);

        // If bidirectional and reverse edge not explicitly provided, add reverse edge
        if (!edge.oneway) {
          const revId = `${edge.id}_rev`;
          this.addAdjacencyEdge(edge.toNodeId, edge.fromNodeId, edge.lengthMeters, revId);

          if (!this.edges.has(revId)) {
            const revGeom = [...edge.geometry].reverse() as [number, number][];
            const revEdge: RoadEdge = {
              id: revId,
              fromNodeId: edge.toNodeId,
              toNodeId: edge.fromNodeId,
              geometry: revGeom,
              lengthMeters: edge.lengthMeters,
              roadType: edge.roadType,
              oneway: false,
            };
            this.edges.set(revId, revEdge);
          }
        }
      }
    }
  }

  private addAdjacencyEdge(from: string, to: string, weight: number, edgeId: string) {
    let adj = this.adjacency.get(from);
    if (!adj) {
      adj = [];
      this.adjacency.set(from, adj);
    }
    const exists = adj.some((e) => e.toNode === to && e.edgeId === edgeId);
    if (!exists) {
      adj.push({ toNode: to, weight, edgeId });
    }
  }

  shortestPathDistance(sourceNodeId: string, targetNodeId: string, maxSearchDistM = 3000): number {
    if (sourceNodeId === targetNodeId) return 0;
    if (!this.adjacency.has(sourceNodeId) || !this.adjacency.has(targetNodeId)) {
      return Infinity;
    }

    const distMap = new Map<string, number>();
    const pq = new PriorityQueue<string>();

    distMap.set(sourceNodeId, 0);
    pq.push(sourceNodeId, 0);

    let visitedCount = 0;
    const MAX_VISITED = 2000;

    while (!pq.isEmpty() && visitedCount++ < MAX_VISITED) {
      const u = pq.pop()!;
      const d_u = distMap.get(u) ?? Infinity;

      if (u === targetNodeId) {
        return d_u;
      }
      if (d_u > maxSearchDistM) {
        break;
      }

      const neighbors = this.adjacency.get(u);
      if (!neighbors) continue;

      for (const edge of neighbors) {
        const alt = d_u + edge.weight;
        const currentDist = distMap.get(edge.toNode) ?? Infinity;
        if (alt < currentDist) {
          distMap.set(edge.toNode, alt);
          pq.push(edge.toNode, alt);
        }
      }
    }

    return distMap.get(targetNodeId) ?? Infinity;
  }

  findCandidates(
    pt: { lat: number; lng: number; accuracyMeters?: number },
    searchRadiusM = 60.0,
    maxCandidates = 8
  ): CandidateState[] {
    const candidates: CandidateState[] = [];

    for (const edge of this.edges.values()) {
      const geom = edge.geometry;
      if (!geom || geom.length < 2) continue;

      let bestDist = Infinity;
      let bestProjLat = pt.lat;
      let bestProjLng = pt.lng;
      let bestOffsetAlongEdge = 0;
      let accumulatedLength = 0;

      for (let i = 0; i < geom.length - 1; i++) {
        const p1 = geom[i];
        const p2 = geom[i + 1];
        const segLen = haversineDistance(p1[0], p1[1], p2[0], p2[1]);
        const proj = projectPointOnSegment([pt.lat, pt.lng], p1, p2);
        const dist = haversineDistance(pt.lat, pt.lng, proj[0], proj[1]);

        if (dist < bestDist) {
          bestDist = dist;
          bestProjLat = proj[0];
          bestProjLng = proj[1];
          const distFromSegStart = haversineDistance(p1[0], p1[1], proj[0], proj[1]);
          bestOffsetAlongEdge = accumulatedLength + distFromSegStart;
        }
        accumulatedLength += segLen;
      }

      if (bestDist <= searchRadiusM) {
        candidates.push({
          edge,
          projectedLat: bestProjLat,
          projectedLng: bestProjLng,
          distToGpsM: bestDist,
          offsetM: bestOffsetAlongEdge,
        });
      }
    }

    candidates.sort((a, b) => a.distToGpsM - b.distToGpsM);
    return candidates.slice(0, maxCandidates);
  }
}

// ---------------------------------------------------------------------------
// HMM Models (Newson & Krumm 2009 with Adaptive Uncertainty)
// ---------------------------------------------------------------------------
const SIGMA_Z_DEFAULT = 4.07;
const BETA_DEFAULT = 5.0;
const BROKEN_PATH_LOG_PENALTY = -60.0;
const MIN_LOG_PROB = -100.0;

/**
 * Calculates adaptive emission sigma accounting for GPS accuracy and DR drift.
 * Capped at 20m to prevent wide snapping to wrong parallel roads.
 */
export function calculateEmissionSigma(
  pt: RecordedGPSPoint,
  drStepIndex: number
): number {
  if (pt.isDeadReckoning) {
    const baseAccuracy = pt.accuracyMeters && pt.accuracyMeters > 0 ? pt.accuracyMeters : 10.0;
    // DR drift grows with duration of outage, capped at 20m
    const drGrowth = Math.min(8.0, drStepIndex * 0.4);
    return Math.max(6.0, Math.min(20.0, Math.max(baseAccuracy, 8.0 + drGrowth)));
  }

  let sigma = SIGMA_Z_DEFAULT;
  if (pt.accuracyMeters && pt.accuracyMeters > 1.0) {
    sigma = Math.max(2.0, Math.min(pt.accuracyMeters, 8.0));
  }
  return sigma;
}

export function logEmissionProbability(
  candidate: CandidateState,
  pt: RecordedGPSPoint,
  drStepIndex: number
): number {
  const sigma = calculateEmissionSigma(pt, drStepIndex);
  const dist = candidate.distToGpsM;
  const logNorm = -Math.log(Math.sqrt(2.0 * Math.PI) * sigma);
  const logProb = logNorm - 0.5 * Math.pow(dist / sigma, 2);
  return Math.max(MIN_LOG_PROB, logProb);
}

export function computeRouteDistance(
  graph: MemoryRoadGraph,
  c1: CandidateState,
  c2: CandidateState
): number {
  // Case 1: Same directed edge
  if (c1.edge.id === c2.edge.id) {
    if (c2.offsetM >= c1.offsetM) {
      return c2.offsetM - c1.offsetM;
    } else {
      const backwardDelta = c1.offsetM - c2.offsetM;
      if (backwardDelta < 30.0) {
        // Small backward jitter allowed with directional penalty
        return backwardDelta * 1.5;
      }
      return Infinity;
    }
  }

  // Case 2: Different edges
  const distToExit = Math.max(0, c1.edge.lengthMeters - c1.offsetM);
  const graphDist = graph.shortestPathDistance(c1.edge.toNodeId, c2.edge.fromNodeId);

  if (!isFinite(graphDist)) {
    return Infinity;
  }

  const distFromEntry = Math.max(0, c2.offsetM);
  return distToExit + graphDist + distFromEntry;
}

export function logTransitionProbability(
  graph: MemoryRoadGraph,
  c1: CandidateState,
  c2: CandidateState,
  ptPrev: RecordedGPSPoint,
  ptCurr: RecordedGPSPoint
): number {
  const dtSec = Math.max(0.1, (ptCurr.timestamp - ptPrev.timestamp) / 1000.0) || 1.0;
  const speedMps = Math.max((ptCurr.speedKmH ?? 0) / 3.6, (ptPrev.speedKmH ?? 0) / 3.6);

  // Scale beta dynamically with speed, time step, and DR state
  const isDr = Boolean(ptCurr.isDeadReckoning || ptPrev.isDeadReckoning);
  const beta = Math.max(
    5.0,
    Math.min(25.0, BETA_DEFAULT + 0.25 * speedMps * dtSec + (isDr ? 4.0 : 0.0))
  );

  const d_gc = haversineDistance(
    c1.projectedLat,
    c1.projectedLng,
    c2.projectedLat,
    c2.projectedLng
  );

  const d_route = computeRouteDistance(graph, c1, c2);

  if (!isFinite(d_route)) {
    return BROKEN_PATH_LOG_PENALTY;
  }

  const delta = Math.abs(d_route - d_gc);
  const logProb = -Math.log(beta) - delta / beta;
  return Math.max(MIN_LOG_PROB, logProb);
}

// ---------------------------------------------------------------------------
// Map Matcher Execution Pipeline
// ---------------------------------------------------------------------------
export async function matchSegment(points: RecordedGPSPoint[]): Promise<MapMatchResult> {
  if (!points || points.length === 0) {
    return { matched: false, reason: 'empty_points' };
  }

  // 1. Identify all required cell keys
  const primaryCellKeys = new Set<string>();
  for (const pt of points) {
    primaryCellKeys.add(RoadGraphCacheService.cellKey(pt.lat, pt.lng));
  }

  // 2. Load all primary and neighbor cached cells
  const graph = new MemoryRoadGraph();
  const allLoadedKeys = new Set<string>();

  for (const pKey of primaryCellKeys) {
    const cached = await RoadGraphCacheService.getCell(pKey);
    if (!cached || !cached.cell) {
      return { matched: false, reason: 'no_coverage' };
    }
    graph.addCell(cached.cell);
    allLoadedKeys.add(pKey);

    const neighbors = RoadGraphCacheService.getNeighborCellKeys(pKey);
    for (const nKey of neighbors) {
      if (!allLoadedKeys.has(nKey)) {
        allLoadedKeys.add(nKey);
        const nCached = await RoadGraphCacheService.getCell(nKey);
        if (nCached && nCached.cell) {
          graph.addCell(nCached.cell);
        }
      }
    }
  }

  if (graph.edges.size === 0) {
    return { matched: false, reason: 'no_edges_in_cells' };
  }

  return matchSegmentWithGraph(points, graph);
}

interface CompressedObservation {
  point: RecordedGPSPoint;
  originalIndices: number[];
  isAnchor: boolean;
}

/**
 * Pure map matching engine given a populated road graph.
 * Includes stationary compression, GNSS anchoring, capped radius/sigma,
 * robust freeze guard, and DO-NO-HARM endpoint validation.
 */
export function matchSegmentWithGraph(
  points: RecordedGPSPoint[],
  graph: MemoryRoadGraph
): MapMatchResult {
  const N = points.length;
  if (N === 0 || graph.edges.size === 0) {
    return { matched: false, reason: 'no_data' };
  }

  // 1. Compress stationary rows (speed < 1.0 km/h or step displacement < 0.5m)
  // Preserve bounding anchors (points where isDeadReckoning === false)
  const compressed: CompressedObservation[] = [];
  let i = 0;

  while (i < N) {
    const pt = points[i];
    const isAnchor = pt.isDeadReckoning === false;

    if (isAnchor) {
      compressed.push({
        point: pt,
        originalIndices: [i],
        isAnchor: true,
      });
      i++;
    } else if ((pt.speedKmH ?? 0) < 1.0) {
      // Group contiguous stationary DR samples
      const groupIndices: number[] = [i];
      let j = i + 1;
      while (
        j < N &&
        points[j].isDeadReckoning !== false &&
        (points[j].speedKmH ?? 0) < 1.0
      ) {
        groupIndices.push(j);
        j++;
      }
      // Use the last stationary sample as representative
      compressed.push({
        point: points[j - 1],
        originalIndices: groupIndices,
        isAnchor: false,
      });
      i = j;
    } else {
      compressed.push({
        point: pt,
        originalIndices: [i],
        isAnchor: false,
      });
      i++;
    }
  }

  const T = compressed.length;
  if (T === 0) {
    return { matched: false, reason: 'no_observations' };
  }

  // 2. Generate candidate states per compressed timestep with capped radius
  const candidatesPerStep: CandidateState[][] = [];
  let currentDrIndex = 0;

  for (let t = 0; t < T; t++) {
    const obs = compressed[t];
    const pt = obs.point;

    if (obs.isAnchor) {
      currentDrIndex = 0;
    } else {
      currentDrIndex++;
    }

    const sigma = calculateEmissionSigma(pt, currentDrIndex);
    // Search radius capped <= 60m unless zero candidates
    const searchRadius = Math.min(60.0, Math.max(30.0, sigma * 2.5));

    let cands = graph.findCandidates(pt, searchRadius, 8);

    // Fallback search up to 80m if no candidates found
    if (cands.length === 0) {
      cands = graph.findCandidates(pt, 80.0, 8);
    }

    // Continuity fallback: if still zero candidates, project onto previous candidate edge
    if (cands.length === 0) {
      if (t > 0 && candidatesPerStep[t - 1].length > 0) {
        cands = [];
        for (const prevCand of candidatesPerStep[t - 1]) {
          const geom = prevCand.edge.geometry;
          let bestDist = Infinity;
          let bestLat = pt.lat;
          let bestLng = pt.lng;
          let bestOffset = 0;
          let accLen = 0;

          for (let segIdx = 0; segIdx < geom.length - 1; segIdx++) {
            const p1 = geom[segIdx];
            const p2 = geom[segIdx + 1];
            const segLen = haversineDistance(p1[0], p1[1], p2[0], p2[1]);
            const proj = projectPointOnSegment([pt.lat, pt.lng], p1, p2);
            const dist = haversineDistance(pt.lat, pt.lng, proj[0], proj[1]);

            if (dist < bestDist) {
              bestDist = dist;
              bestLat = proj[0];
              bestLng = proj[1];
              bestOffset = accLen + haversineDistance(p1[0], p1[1], proj[0], proj[1]);
            }
            accLen += segLen;
          }

          cands.push({
            edge: prevCand.edge,
            projectedLat: bestLat,
            projectedLng: bestLng,
            distToGpsM: bestDist,
            offsetM: bestOffset,
          });
        }
      } else {
        // Fallback: search closest edge across loaded graph
        let nearestEdge: RoadEdge | null = null;
        let minEdgeDist = Infinity;
        let nProjLat = pt.lat;
        let nProjLng = pt.lng;
        let nOffset = 0;

        for (const edge of graph.edges.values()) {
          const geom = edge.geometry;
          let accLen = 0;
          for (let segIdx = 0; segIdx < geom.length - 1; segIdx++) {
            const p1 = geom[segIdx];
            const p2 = geom[segIdx + 1];
            const segLen = haversineDistance(p1[0], p1[1], p2[0], p2[1]);
            const proj = projectPointOnSegment([pt.lat, pt.lng], p1, p2);
            const dist = haversineDistance(pt.lat, pt.lng, proj[0], proj[1]);

            if (dist < minEdgeDist) {
              minEdgeDist = dist;
              nearestEdge = edge;
              nProjLat = proj[0];
              nProjLng = proj[1];
              nOffset = accLen + haversineDistance(p1[0], p1[1], proj[0], proj[1]);
            }
            accLen += segLen;
          }
        }

        if (nearestEdge) {
          cands = [
            {
              edge: nearestEdge,
              projectedLat: nProjLat,
              projectedLng: nProjLng,
              distToGpsM: minEdgeDist,
              offsetM: nOffset,
            },
          ];
        } else {
          return { matched: false, reason: 'no_edges_available' };
        }
      }
    }

    candidatesPerStep.push(cands);
  }

  // 3. Viterbi Initialization (t = 0)
  const numCands0 = candidatesPerStep[0].length;
  const V: number[][] = [];
  const backpointers: number[][] = [];

  const v0 = new Array<number>(numCands0);
  for (let cIdx = 0; cIdx < numCands0; cIdx++) {
    v0[cIdx] = logEmissionProbability(candidatesPerStep[0][cIdx], compressed[0].point, 0);
  }
  V.push(v0);

  // 4. Viterbi Forward Recursion (t = 1 .. T-1)
  let drCount = 0;
  for (let t = 1; t < T; t++) {
    const ptPrev = compressed[t - 1].point;
    const ptCurr = compressed[t].point;
    const candsPrev = candidatesPerStep[t - 1];
    const candsCurr = candidatesPerStep[t];

    if (!compressed[t].isAnchor) {
      drCount++;
    } else {
      drCount = 0;
    }

    const v_t = new Array<number>(candsCurr.length).fill(-Infinity);
    const bp_t = new Array<number>(candsCurr.length).fill(0);

    let bestPrevIdx = 0;
    let maxPrevScore = -Infinity;
    for (let pIdx = 0; pIdx < candsPrev.length; pIdx++) {
      if (V[t - 1][pIdx] > maxPrevScore) {
        maxPrevScore = V[t - 1][pIdx];
        bestPrevIdx = pIdx;
      }
    }

    for (let j = 0; j < candsCurr.length; j++) {
      const c_j = candsCurr[j];
      const logEmission = logEmissionProbability(c_j, ptCurr, drCount);
      let bestVal = -Infinity;
      let bestI = 0;

      for (let pIdx = 0; pIdx < candsPrev.length; pIdx++) {
        const c_i = candsPrev[pIdx];
        const logTrans = logTransitionProbability(graph, c_i, c_j, ptPrev, ptCurr);
        const score = V[t - 1][pIdx] + logTrans;
        if (score > bestVal) {
          bestVal = score;
          bestI = pIdx;
        }
      }

      // Underflow protection
      if (!isFinite(bestVal)) {
        bestVal = (isFinite(maxPrevScore) ? maxPrevScore : -50.0) + BROKEN_PATH_LOG_PENALTY;
        bestI = bestPrevIdx;
      }

      v_t[j] = bestVal + logEmission;
      bp_t[j] = bestI;
    }

    V.push(v_t);
    backpointers.push(bp_t);
  }

  // 5. Termination & Backtracking
  const lastV = V[T - 1];
  let bestLastIdx = 0;
  let maxScore = -Infinity;
  for (let j = 0; j < lastV.length; j++) {
    if (lastV[j] > maxScore) {
      maxScore = lastV[j];
      bestLastIdx = j;
    }
  }

  const chosenCompressedStates: CandidateState[] = new Array(T);
  chosenCompressedStates[T - 1] = candidatesPerStep[T - 1][bestLastIdx];

  let currIdx = bestLastIdx;
  for (let t = T - 1; t > 0; t--) {
    currIdx = backpointers[t - 1][currIdx];
    chosenCompressedStates[t - 1] = candidatesPerStep[t - 1][currIdx];
  }

  // 6. Expand matched points back to original full point sequence
  const expandedPoints: { lat: number; lng: number }[] = new Array(N);
  const expandedStates: CandidateState[] = new Array(N);

  for (let t = 0; t < T; t++) {
    const obs = compressed[t];
    const st = chosenCompressedStates[t];
    for (const origIdx of obs.originalIndices) {
      expandedPoints[origIdx] = {
        lat: st.projectedLat,
        lng: st.projectedLng,
      };
      expandedStates[origIdx] = st;
    }
  }

  // 7. Plausibility & Freeze Guard on full trajectory
  let consecutiveFrozenSamples = 0;
  let consecutiveClampedRepairs = 0;

  for (let t = 1; t < N; t++) {
    const pPrev = expandedStates[t - 1];
    const rawPrev = points[t - 1];
    const rawCurr = points[t];

    const dtSec = Math.max(0.2, (rawCurr.timestamp - rawPrev.timestamp) / 1000.0) || 1.0;
    const distM = haversineDistance(
      expandedPoints[t - 1].lat,
      expandedPoints[t - 1].lng,
      expandedPoints[t].lat,
      expandedPoints[t].lng
    );
    const impliedSpeedKmh = (distM / dtSec) * 3.6;
    const loggedSpeedKmh = Math.max(rawCurr.speedKmH || 0, rawPrev.speedKmH || 0);
    const maxAllowedSpeedKmh = Math.max(2.0 * loggedSpeedKmh, 65.0);

    // Freeze detection during vehicle motion (logged speed > 5 km/h)
    if (distM < 0.5 && loggedSpeedKmh > 5.0) {
      consecutiveFrozenSamples++;
      if (consecutiveFrozenSamples >= 3) {
        return {
          matched: false,
          reason: `frozen_output_detected_at_step_${t}_(${consecutiveFrozenSamples}_consecutive_samples)`,
        };
      }
    } else {
      consecutiveFrozenSamples = 0;
    }

    if (impliedSpeedKmh > maxAllowedSpeedKmh) {
      // Unphysical jump detected! Attempt local edge projection repair
      const geomPrev = pPrev.edge.geometry;
      let repairedDist = Infinity;
      let repLat = expandedPoints[t].lat;
      let repLng = expandedPoints[t].lng;
      let repOffset = 0;
      let acc = 0;

      for (let segIdx = 0; segIdx < geomPrev.length - 1; segIdx++) {
        const seg1 = geomPrev[segIdx];
        const seg2 = geomPrev[segIdx + 1];
        const segLen = haversineDistance(seg1[0], seg1[1], seg2[0], seg2[1]);
        const proj = projectPointOnSegment([rawCurr.lat, rawCurr.lng], seg1, seg2);
        const d = haversineDistance(rawCurr.lat, rawCurr.lng, proj[0], proj[1]);

        if (d < repairedDist) {
          repairedDist = d;
          repLat = proj[0];
          repLng = proj[1];
          repOffset = acc + haversineDistance(seg1[0], seg1[1], proj[0], proj[1]);
        }
        acc += segLen;
      }

      const repairedJumpM = haversineDistance(
        expandedPoints[t - 1].lat,
        expandedPoints[t - 1].lng,
        repLat,
        repLng
      );
      const repairedSpeedKmh = (repairedJumpM / dtSec) * 3.6;

      // Ensure repair does not cause a continuous freeze clamp during motion
      if (repairedJumpM < 0.5 && loggedSpeedKmh > 5.0) {
        consecutiveClampedRepairs++;
        if (consecutiveClampedRepairs >= 2) {
          return {
            matched: false,
            reason: `unphysical_jump_and_frozen_clamp_at_step_${t}`,
          };
        }
      } else {
        consecutiveClampedRepairs = 0;
      }

      if (repairedSpeedKmh <= maxAllowedSpeedKmh) {
        expandedPoints[t] = { lat: repLat, lng: repLng };
        expandedStates[t] = {
          edge: pPrev.edge,
          projectedLat: repLat,
          projectedLng: repLng,
          distToGpsM: repairedDist,
          offsetM: repOffset,
        };
      } else {
        return {
          matched: false,
          reason: `unphysical_speed_jump_${impliedSpeedKmh.toFixed(1)}_kmh_at_step_${t}`,
        };
      }
    }
  }

  // 8. DO-NO-HARM RULE: Check endpoint accuracy and progress against GNSS anchor
  // If the last point in the sequence is a GNSS anchor (or if bounding GNSS exists)
  const hasTrailingGnss = points[N - 1].isDeadReckoning === false && N > 1;
  if (hasTrailingGnss) {
    const trailingGnss = points[N - 1];
    // Compare last DR point (index N - 2) against trailing GNSS
    const lastDrRaw = points[N - 2];
    const lastDrMatched = expandedPoints[N - 2];

    const rawEndpointError = haversineDistance(
      lastDrRaw.lat,
      lastDrRaw.lng,
      trailingGnss.lat,
      trailingGnss.lng
    );
    const matchedEndpointError = haversineDistance(
      lastDrMatched.lat,
      lastDrMatched.lng,
      trailingGnss.lat,
      trailingGnss.lng
    );

    // Compute progress
    let totalRawProgress = 0;
    let totalMatchedProgress = 0;
    for (let t = 1; t < N - 1; t++) {
      totalRawProgress += haversineDistance(
        points[t - 1].lat,
        points[t - 1].lng,
        points[t].lat,
        points[t].lng
      );
      totalMatchedProgress += haversineDistance(
        expandedPoints[t - 1].lat,
        expandedPoints[t - 1].lng,
        expandedPoints[t].lat,
        expandedPoints[t].lng
      );
    }

    const progressRatio = totalRawProgress > 5.0 ? totalMatchedProgress / totalRawProgress : 1.0;

    if (matchedEndpointError > rawEndpointError + 1.0) {
      return {
        matched: false,
        reason: `do_no_harm_rejected_endpoint_error_${matchedEndpointError.toFixed(1)}m_worse_than_raw_${rawEndpointError.toFixed(1)}m`,
      };
    }

    if (progressRatio < 0.70) {
      return {
        matched: false,
        reason: `do_no_harm_rejected_progress_ratio_${progressRatio.toFixed(2)}_below_0.70`,
      };
    }
  }

  return {
    matched: true,
    points: expandedPoints,
  };
}

// ---------------------------------------------------------------------------
// Worker Message Handler
// ---------------------------------------------------------------------------
if (typeof self !== 'undefined' && typeof (self as any).postMessage === 'function') {
  self.addEventListener('message', async (e: MessageEvent) => {
    const { type, payload, messageId } = e.data;

    if (type === 'MATCH_SEGMENT') {
      try {
        const result = await matchSegment(payload.points);
        (self as any).postMessage({ type: 'MATCH_RESULT', messageId, result });
      } catch (err: any) {
        (self as any).postMessage({
          type: 'MATCH_RESULT',
          messageId,
          result: { matched: false, reason: err?.message || 'unknown_error' },
        });
      }
    }
  });
}
