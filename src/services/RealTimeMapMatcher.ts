/**
 * RealTimeMapMatcher.ts
 * ======================
 * Ported from IDR_PRO_2/src/mapmatch/MapMatcher.ts.
 *
 * Real-time, per-position road snapping for live navigation.
 * Unlike the batch HMM in mapMatcher.worker.ts (post-session),
 * this runs synchronously on every fused position update during IDR mode.
 *
 * Usage:
 *   RealTimeMapMatcher.loadSegments(segments);
 *   const snap = RealTimeMapMatcher.snap(lat, lon, headingRad, speedMps);
 *   if (snap.confidence > 0.4) { use snap.lat, snap.lon }
 */

import { OsmConverter, type RoadSegment, type Point2D } from './OsmConverter';

export interface SnapResult {
  lat: number;
  lon: number;
  segmentId: string | null;
  confidence: number; // 0 (no match) to 1 (exact match)
}

// Grid cell size in degrees (~111 m per 0.001°)
const GRID_SIZE = 0.001;
// Max perpendicular distance to snap (meters)
const MAX_SNAP_DIST = 20.0;

let segments: RoadSegment[] = [];
let grid: Map<string, RoadSegment[]> = new Map();
let isLoaded = false;

function cellKey(x: number, y: number): string {
  return `${x}_${y}`;
}

function buildGrid(segs: RoadSegment[]): void {
  grid = new Map();
  for (const seg of segs) {
    const x1 = Math.floor(seg.p1.lon / GRID_SIZE);
    const y1 = Math.floor(seg.p1.lat / GRID_SIZE);
    const x2 = Math.floor(seg.p2.lon / GRID_SIZE);
    const y2 = Math.floor(seg.p2.lat / GRID_SIZE);
    const minX = Math.min(x1, x2);
    const maxX = Math.max(x1, x2);
    const minY = Math.min(y1, y2);
    const maxY = Math.max(y1, y2);
    for (let x = minX; x <= maxX; x++) {
      for (let y = minY; y <= maxY; y++) {
        const k = cellKey(x, y);
        if (!grid.has(k)) grid.set(k, []);
        grid.get(k)!.push(seg);
      }
    }
  }
}

function projectPointToSegment(
  lat: number, lon: number, seg: RoadSegment
): { pt: Point2D; distMeters: number } {
  const latRad = (lat * Math.PI) / 180;
  const cosLat = Math.cos(latRad);
  const px = lon * cosLat;
  const py = lat;
  const ax = seg.p1.lon * cosLat;
  const ay = seg.p1.lat;
  const bx = seg.p2.lon * cosLat;
  const by = seg.p2.lat;
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  let t = 0;
  if (lenSq > 0) {
    t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lenSq));
  }
  const projLon = (ax + t * dx) / cosLat;
  const projLat = ay + t * dy;
  return {
    pt: { lat: projLat, lon: projLon },
    distMeters: OsmConverter.haversine(lat, lon, projLat, projLon),
  };
}

function angleDiff(a: number, b: number): number {
  let d = a - b;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

export const RealTimeMapMatcher = {
  /** Load road segments into the spatial index. Call once per area. */
  loadSegments(segs: RoadSegment[]): void {
    segments = segs;
    buildGrid(segs);
    isLoaded = segs.length > 0;
    console.log(`[RealTimeMapMatcher] Loaded ${segs.length} road segments.`);
  },

  get isReady(): boolean {
    return isLoaded;
  },

  get segmentCount(): number {
    return segments.length;
  },

  /**
   * Snap (lat, lon) to the nearest road segment, considering heading.
   * @param headingRad Vehicle heading in radians (North=0, East=PI/2)
   * @param speedMps   Vehicle speed in m/s (used to gate heading check)
   */
  snap(lat: number, lon: number, headingRad: number, speedMps: number): SnapResult {
    if (!isLoaded) return { lat, lon, segmentId: null, confidence: 0 };

    const gridX = Math.floor(lon / GRID_SIZE);
    const gridY = Math.floor(lat / GRID_SIZE);

    // Gather candidates from 3×3 neighborhood
    const candidates = new Set<RoadSegment>();
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const segs = grid.get(cellKey(gridX + dx, gridY + dy));
        if (segs) segs.forEach((s) => candidates.add(s));
      }
    }

    const searchSet = candidates.size > 0 ? candidates : new Set(segments);

    let best: SnapResult = { lat, lon, segmentId: null, confidence: 0 };
    let bestCost = Infinity;

    for (const seg of searchSet) {
      const { pt, distMeters } = projectPointToSegment(lat, lon, seg);
      if (distMeters > MAX_SNAP_DIST) continue;

      // Heading gate — only active when moving
      let headingPenalty = 0;
      if (speedMps > 1.0) {
        const fwdDiff = Math.abs(angleDiff(headingRad, seg.headingRad));
        const revDiff = Math.abs(angleDiff(headingRad, seg.headingRad + Math.PI));
        const minDiff = seg.oneway ? fwdDiff : Math.min(fwdDiff, revDiff);
        if (minDiff > Math.PI / 4) continue; // >45° mismatch — reject
        headingPenalty = minDiff * 10;
      }

      const cost = distMeters + headingPenalty;
      if (cost < bestCost) {
        bestCost = cost;
        best = {
          lat: pt.lat,
          lon: pt.lon,
          segmentId: seg.id,
          confidence: Math.max(0, 1 - distMeters / MAX_SNAP_DIST),
        };
      }
    }

    return best;
  },

  /** Fetch segments from Overpass for a bounding box and load them. */
  async loadFromOverpass(
    minLat: number, minLon: number,
    maxLat: number, maxLon: number
  ): Promise<void> {
    try {
      const segs = await OsmConverter.fetchAndConvert(minLat, minLon, maxLat, maxLon);
      RealTimeMapMatcher.loadSegments(segs);
    } catch (err) {
      console.warn('[RealTimeMapMatcher] Overpass fetch failed:', err);
    }
  },
};
