/**
 * OsmConverter.ts
 * ===============
 * Ported from IDR_PRO_2/src/mapmatch/OsmConverter.ts.
 * Converts Overpass API JSON into RoadSegment[] for use by RealTimeMapMatcher.
 */

export interface Point2D {
  lat: number;
  lon: number;
}

export interface RoadSegment {
  id: string;       // wayId_segmentIndex
  wayId: number;
  p1: Point2D;
  p2: Point2D;
  roadClass: string;
  oneway: boolean;
  headingRad: number;
  lengthMeters: number;
}

export class OsmConverter {
  /** Haversine distance in meters */
  public static haversine(lat1: number, lon1: number, lat2: number, lon2: number): number {
    const R = 6378137.0;
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
      Math.sin(dLon / 2) * Math.sin(dLon / 2);
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  }

  /** Initial bearing from p1→p2 in radians (North=0, East=PI/2) */
  public static computeHeadingRad(lat1: number, lon1: number, lat2: number, lon2: number): number {
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const lat1R = lat1 * Math.PI / 180;
    const lat2R = lat2 * Math.PI / 180;
    const y = Math.sin(dLon) * Math.cos(lat2R);
    const x = Math.cos(lat1R) * Math.sin(lat2R) - Math.sin(lat1R) * Math.cos(lat2R) * Math.cos(dLon);
    return Math.atan2(y, x);
  }

  /** Fetch road segments from Overpass API for a bounding box */
  public static async fetchAndConvert(
    minLat: number, minLon: number,
    maxLat: number, maxLon: number
  ): Promise<RoadSegment[]> {
    const query = `
      [out:json];
      (
        way["highway"](${minLat},${minLon},${maxLat},${maxLon});
      );
      out body;
      >;
      out skel qt;
    `;
    const response = await fetch('https://overpass-api.de/api/interpreter', {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/x-www-form-urlencoded',
        'User-Agent': 'ReckonX-Navigation',
      },
      body: `data=${encodeURIComponent(query)}`,
    });
    if (!response.ok) throw new Error(`Overpass API error: ${response.statusText}`);
    return OsmConverter.parseOverpassJson(await response.json());
  }

  /** Parse already-fetched Overpass JSON into RoadSegment[] */
  public static parseOverpassJson(data: any): RoadSegment[] {
    const nodes = new Map<number, Point2D>();
    for (const el of data.elements) {
      if (el.type === 'node') nodes.set(el.id, { lat: el.lat, lon: el.lon });
    }

    const segments: RoadSegment[] = [];
    for (const el of data.elements) {
      if (el.type !== 'way' || !el.tags?.highway) continue;
      const hw = el.tags.highway as string;
      // Exclude non-vehicle ways
      if (['footway', 'steps', 'pedestrian', 'path', 'cycleway'].includes(hw)) continue;

      const oneway = el.tags.oneway === 'yes';
      for (let i = 0; i < el.nodes.length - 1; i++) {
        const n1 = nodes.get(el.nodes[i]);
        const n2 = nodes.get(el.nodes[i + 1]);
        if (!n1 || !n2) continue;
        segments.push({
          id: `${el.id}_${i}`,
          wayId: el.id,
          p1: n1,
          p2: n2,
          roadClass: hw,
          oneway,
          headingRad: OsmConverter.computeHeadingRad(n1.lat, n1.lon, n2.lat, n2.lon),
          lengthMeters: OsmConverter.haversine(n1.lat, n1.lon, n2.lat, n2.lon),
        });
      }
    }
    return segments;
  }
}
