"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.OsmConverter = void 0;
class OsmConverter {
    /**
     * Haversine distance in meters
     */
    static haversine(lat1, lon1, lat2, lon2) {
        const R = 6378137.0;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    }
    /**
     * Initial bearing from p1 to p2 in radians (East = 0, North = PI/2 in some conventions,
     * but usually North = 0, East = PI/2 in navigation. We'll use Math.atan2(E, N))
     */
    static computeHeadingRad(lat1, lon1, lat2, lon2) {
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const lat1R = lat1 * Math.PI / 180;
        const lat2R = lat2 * Math.PI / 180;
        const y = Math.sin(dLon) * Math.cos(lat2R);
        const x = Math.cos(lat1R) * Math.sin(lat2R) - Math.sin(lat1R) * Math.cos(lat2R) * Math.cos(dLon);
        // atan2(y, x) -> y is East, x is North -> North=0, East=PI/2
        return Math.atan2(y, x);
    }
    /**
     * Downloads and converts OSM data using Overpass API
     */
    static async fetchAndConvert(minLat, minLon, maxLat, maxLon) {
        const query = `
      [out:json];
      (
        way["highway"](${minLat},${minLon},${maxLat},${maxLon});
      );
      out body;
      >;
      out skel qt;
    `;
        const url = 'https://overpass-api.de/api/interpreter';
        console.log("Fetching from Overpass API...");
        const response = await fetch(url, {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Content-Type': 'application/x-www-form-urlencoded',
                'User-Agent': 'IDR-PoC (test-script)'
            },
            body: `data=${encodeURIComponent(query)}`
        });
        if (!response.ok) {
            throw new Error(`Overpass API error: ${response.statusText}`);
        }
        const data = await response.json();
        return this.parseOverpassJson(data);
    }
    static parseOverpassJson(data) {
        const nodes = new Map();
        for (const el of data.elements) {
            if (el.type === 'node') {
                nodes.set(el.id, { lat: el.lat, lon: el.lon });
            }
        }
        const segments = [];
        for (const el of data.elements) {
            if (el.type === 'way' && el.tags && el.tags.highway) {
                // Exclude pedestrian, footway, steps
                const hw = el.tags.highway;
                if (hw === 'footway' || hw === 'steps' || hw === 'pedestrian' || hw === 'path') {
                    continue;
                }
                const oneway = el.tags.oneway === 'yes';
                const roadClass = hw;
                const wayNodes = el.nodes;
                for (let i = 0; i < wayNodes.length - 1; i++) {
                    const n1 = nodes.get(wayNodes[i]);
                    const n2 = nodes.get(wayNodes[i + 1]);
                    if (n1 && n2) {
                        const lengthMeters = this.haversine(n1.lat, n1.lon, n2.lat, n2.lon);
                        const headingRad = this.computeHeadingRad(n1.lat, n1.lon, n2.lat, n2.lon);
                        segments.push({
                            id: `${el.id}_${i}`,
                            wayId: el.id,
                            p1: n1,
                            p2: n2,
                            roadClass,
                            oneway,
                            headingRad,
                            lengthMeters
                        });
                    }
                }
            }
        }
        return segments;
    }
}
exports.OsmConverter = OsmConverter;
