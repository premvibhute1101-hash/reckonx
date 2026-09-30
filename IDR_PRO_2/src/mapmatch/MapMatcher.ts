import { RoadSegment, Point2D, OsmConverter } from './OsmConverter';

export interface SnapResult {
  lat: number;
  lon: number;
  segmentId: string | null;
  confidence: number;
}

export class MapMatcher {
  private segments: RoadSegment[] = [];
  
  // Spatial index: simple grid (lat/lon to cell)
  // Grid size ~0.001 degrees (~111m)
  private readonly GRID_SIZE = 0.001;
  private grid: Map<string, RoadSegment[]> = new Map();
  
  private minLat = Infinity;
  private minLon = Infinity;
  private maxLat = -Infinity;
  private maxLon = -Infinity;

  public loadSegments(segments: RoadSegment[]) {
    this.segments = segments;
    this.grid.clear();
    
    for (const seg of segments) {
      this.minLat = Math.min(this.minLat, seg.p1.lat, seg.p2.lat);
      this.minLon = Math.min(this.minLon, seg.p1.lon, seg.p2.lon);
      this.maxLat = Math.max(this.maxLat, seg.p1.lat, seg.p2.lat);
      this.maxLon = Math.max(this.maxLon, seg.p1.lon, seg.p2.lon);
      
      const gridX1 = Math.floor(seg.p1.lon / this.GRID_SIZE);
      const gridY1 = Math.floor(seg.p1.lat / this.GRID_SIZE);
      const gridX2 = Math.floor(seg.p2.lon / this.GRID_SIZE);
      const gridY2 = Math.floor(seg.p2.lat / this.GRID_SIZE);
      
      // Simple addition to all cells the segment endpoints touch
      // For more accuracy we should rasterize the segment, but endpoints are often enough
      // if segments are small. To be safe, we just bound the segment box.
      const minX = Math.min(gridX1, gridX2);
      const maxX = Math.max(gridX1, gridX2);
      const minY = Math.min(gridY1, gridY2);
      const maxY = Math.max(gridY1, gridY2);
      
      for (let x = minX; x <= maxX; x++) {
        for (let y = minY; y <= maxY; y++) {
          const key = `${x}_${y}`;
          if (!this.grid.has(key)) {
            this.grid.set(key, []);
          }
          this.grid.get(key)!.push(seg);
        }
      }
    }
  }

  /**
   * Projects a point (lat, lon) onto the segment (p1 -> p2).
   * Returns the closest point on the segment, and distance in meters.
   */
  private projectPointToSegment(lat: number, lon: number, seg: RoadSegment): { pt: Point2D, distSqDeg: number, distMeters: number } {
    // Equirectangular projection for local planar geometry
    const latRad = lat * Math.PI / 180;
    const cosLat = Math.cos(latRad);
    
    // Scale lon to be roughly in meters equivalent relative to lat
    // We compute in degrees, just scaling the X axis
    const px = lon * cosLat;
    const py = lat;
    const ax = seg.p1.lon * cosLat;
    const ay = seg.p1.lat;
    const bx = seg.p2.lon * cosLat;
    const by = seg.p2.lat;

    const dx = bx - ax;
    const dy = by - ay;
    const lengthSq = dx * dx + dy * dy;

    let t = 0;
    if (lengthSq > 0) {
      t = ((px - ax) * dx + (py - ay) * dy) / lengthSq;
      t = Math.max(0, Math.min(1, t));
    }

    const projLonScaled = ax + t * dx;
    const projLat = ay + t * dy;
    const projLon = projLonScaled / cosLat;
    
    const distMeters = OsmConverter.haversine(lat, lon, projLat, projLon);
    return { pt: { lat: projLat, lon: projLon }, distSqDeg: 0 /* unused */, distMeters };
  }

  /**
   * Normalizes angle difference to -PI to PI
   */
  private angleDiff(a: number, b: number): number {
    let diff = a - b;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    return diff;
  }

  /**
   * Snaps a position to the nearest road segment considering heading.
   * @param lat 
   * @param lon 
   * @param headingRad Vehicle heading in radians (North=0, East=PI/2)
   * @param speed Vehicle speed in m/s
   * @returns 
   */
  public snap(lat: number, lon: number, headingRad: number, speed: number): SnapResult {
    if (this.segments.length === 0) {
      return { lat, lon, segmentId: null, confidence: 0 };
    }

    // Lookup neighboring cells
    const gridX = Math.floor(lon / this.GRID_SIZE);
    const gridY = Math.floor(lat / this.GRID_SIZE);
    
    const candidates = new Set<RoadSegment>();
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const key = `${gridX + dx}_${gridY + dy}`;
        const segs = this.grid.get(key);
        if (segs) {
          segs.forEach(s => candidates.add(s));
        }
      }
    }
    
    // If no candidates in local grid, search all (fallback)
    const searchSet = candidates.size > 0 ? candidates : this.segments;

    let bestSnap: SnapResult = { lat, lon, segmentId: null, confidence: 0 };
    let bestScore = Infinity; // Lower is better (cost)

    // Configuration
    const MAX_SNAP_DIST = 20.0; // meters

    for (const seg of searchSet) {
      const proj = this.projectPointToSegment(lat, lon, seg);
      if (proj.distMeters > MAX_SNAP_DIST) {
        continue;
      }

      // Check heading consistency if moving
      let headingPenalty = 0;
      if (speed > 1.0) {
        const diff = Math.abs(this.angleDiff(headingRad, seg.headingRad));
        
        let validHeading = false;
        // Check forward direction
        if (diff < Math.PI / 4) {
          validHeading = true;
          headingPenalty = diff * 10; // 10 meters penalty per radian
        }
        
        // If not oneway, check reverse direction
        if (!validHeading && !seg.oneway) {
          const revDiff = Math.abs(this.angleDiff(headingRad, seg.headingRad + Math.PI));
          if (revDiff < Math.PI / 4) {
            validHeading = true;
            headingPenalty = revDiff * 10;
          }
        }
        
        if (!validHeading) {
          continue; // Heading completely mismatched, discard segment
        }
      }

      // Compute total cost (distance + heading penalty)
      const cost = proj.distMeters + headingPenalty;

      if (cost < bestScore) {
        bestScore = cost;
        // Confidence drops based on distance (0 at max distance, 1 at 0 dist)
        const confidence = Math.max(0, 1 - (proj.distMeters / MAX_SNAP_DIST));
        bestSnap = {
          lat: proj.pt.lat,
          lon: proj.pt.lon,
          segmentId: seg.id,
          confidence
        };
      }
    }

    return bestSnap;
  }
}
