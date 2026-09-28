/**
 * RoadGraphCacheService.ts
 * =========================
 * IndexedDB-backed spatial cache for offline road network vector graphs.
 * Mirrors the structure and conventions of TileCacheService.ts.
 */

import { haversineDistance } from './ekf/OutputStabilizer';

export interface RoadNode {
  id: string;
  lat: number;
  lng: number;
}

export interface RoadEdge {
  id: string;
  fromNodeId: string;
  toNodeId: string;
  geometry: [number, number][]; // [lat, lng][] array
  lengthMeters: number;
  roadType: string;
  oneway?: boolean;
}

export interface RoadGraphCell {
  cellKey: string;
  nodes: RoadNode[];
  edges: RoadEdge[];
  timestamp: number;
}

export interface CellFetchResult {
  cell: RoadGraphCell | null;
  isStale: boolean;
}

const DB_NAME = 'IDR_Road_Graph_DB';
const DB_VERSION = 1;
const STORE_NAME = 'cells';

// Fixed 0.025° (~2.7km) lat/lng spatial grid resolution
export const GRID_CELL_SIZE = 0.025;
export const CELL_TTL_MS = 14 * 24 * 60 * 60 * 1000; // 14 Days TTL

// Minimum interval between Overpass API calls to respect rate limits
const MIN_FETCH_INTERVAL_MS = 10000; // 10s
let lastOverpassFetchTime = 0;
const inFlightFetches = new Set<string>();

let dbInstance: IDBDatabase | null = null;

async function getDB(): Promise<IDBDatabase> {
  if (dbInstance) return dbInstance;
  if (typeof indexedDB === 'undefined') {
    throw new Error('IndexedDB is not supported in this environment.');
  }

  return new Promise((resolve, reject) => {
    const request = indexedDB.open(DB_NAME, DB_VERSION);

    request.onupgradeneeded = (event) => {
      const db = (event.target as IDBOpenDBRequest).result;
      if (!db.objectStoreNames.contains(STORE_NAME)) {
        db.createObjectStore(STORE_NAME, { keyPath: 'cellKey' });
      }
    };

    request.onsuccess = () => {
      dbInstance = request.result;
      resolve(dbInstance);
    };

    request.onerror = () => {
      reject(new Error('Failed to open IndexedDB road graph store.'));
    };
  });
}

/**
 * Computes the unique cell key for a given lat/lng coordinate.
 * Formula: `${Math.floor(lat / 0.025)}_${Math.floor(lng / 0.025)}`
 */
export function getCellKey(lat: number, lng: number): string {
  const latIdx = Math.floor(lat / GRID_CELL_SIZE);
  const lngIdx = Math.floor(lng / GRID_CELL_SIZE);
  return `${latIdx}_${lngIdx}`;
}

/**
 * Returns the geographic bounding box (south, west, north, east) of a cell key.
 */
export function getCellBbox(cellKey: string): { south: number; west: number; north: number; east: number } {
  const parts = cellKey.split('_');
  const latIdx = parseInt(parts[0], 10);
  const lngIdx = parseInt(parts[1], 10);

  const south = latIdx * GRID_CELL_SIZE;
  const north = (latIdx + 1) * GRID_CELL_SIZE;
  const west = lngIdx * GRID_CELL_SIZE;
  const east = (lngIdx + 1) * GRID_CELL_SIZE;

  return { south, west, north, east };
}

/**
 * Returns the 8 adjacent neighboring cell keys surrounding the specified cell.
 */
export function getNeighborCellKeys(cellKey: string): string[] {
  const parts = cellKey.split('_');
  const latIdx = parseInt(parts[0], 10);
  const lngIdx = parseInt(parts[1], 10);

  const neighbors: string[] = [];
  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++) {
      if (dx === 0 && dy === 0) continue;
      neighbors.push(`${latIdx + dx}_${lngIdx + dy}`);
    }
  }
  return neighbors;
}

/**
 * Calculates polyline length in meters from an array of [lat, lng] coordinates.
 */
export function calculatePolylineLength(coords: [number, number][]): number {
  if (coords.length < 2) return 0;
  let totalMeters = 0;
  for (let i = 0; i < coords.length - 1; i++) {
    totalMeters += haversineDistance(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1]);
  }
  return totalMeters;
}

export const RoadGraphCacheService = {
  getCellKey,
  cellKey: getCellKey,
  getCellBbox,
  bboxForCell: getCellBbox,
  getNeighborCellKeys,

  /**
   * Saves a road graph cell into IndexedDB.
   */
  async saveCell(cell: RoadGraphCell): Promise<void> {
    try {
      const db = await getDB();
      const tx = db.transaction(STORE_NAME, 'readwrite');
      const store = tx.objectStore(STORE_NAME);
      store.put(cell);
    } catch (err) {
      console.warn('[RoadGraphCacheService] saveCell error:', err);
    }
  },

  /**
   * Retrieves a road graph cell from IndexedDB.
   * Enforces 14-day TTL. If stale and online, returns cell: null (to allow refresh).
   * If stale and offline, returns the stale cell with isStale: true.
   */
  async getCell(cellKey: string): Promise<CellFetchResult> {
    try {
      const db = await getDB();
      const rawCell = await new Promise<RoadGraphCell | null>((resolve) => {
        const tx = db.transaction(STORE_NAME, 'readonly');
        const store = tx.objectStore(STORE_NAME);
        const request = store.get(cellKey);

        request.onsuccess = () => {
          if (request.result) {
            resolve(request.result as RoadGraphCell);
          } else {
            resolve(null);
          }
        };

        request.onerror = () => resolve(null);
      });

      if (!rawCell) {
        return { cell: null, isStale: false };
      }

      const isExpired = Date.now() - rawCell.timestamp > CELL_TTL_MS;
      const isOnline = typeof navigator !== 'undefined' ? navigator.onLine : true;

      if (isExpired) {
        if (!isOnline) {
          return { cell: rawCell, isStale: true }; // offline fallback with stale data
        }
        return { cell: null, isStale: true }; // online miss to trigger refresh
      }

      return { cell: rawCell, isStale: false };
    } catch (err) {
      console.warn('[RoadGraphCacheService] getCell error:', err);
      return { cell: null, isStale: false };
    }
  },

  /**
   * Checks if a non-expired entry exists for the given cellKey.
   */
  async hasCell(cellKey: string): Promise<boolean> {
    const { cell, isStale } = await this.getCell(cellKey);
    return cell !== null && !isStale;
  },

  /**
   * Total number of cached road graph cells.
   */
  async getCacheCount(): Promise<number> {
    try {
      const db = await getDB();
      return new Promise((resolve) => {
        const tx = db.transaction(STORE_NAME, 'readonly');
        const store = tx.objectStore(STORE_NAME);
        const request = store.count();

        request.onsuccess = () => resolve(request.result || 0);
        request.onerror = () => resolve(0);
      });
    } catch {
      return 0;
    }
  },

  /**
   * Clears all cached road graph cells.
   */
  async clearCache(): Promise<void> {
    try {
      const db = await getDB();
      const tx = db.transaction(STORE_NAME, 'readwrite');
      const store = tx.objectStore(STORE_NAME);
      store.clear();
    } catch (err) {
      console.warn('[RoadGraphCacheService] clearCache error:', err);
    }
  },

  /**
   * Queries Overpass API for way["highway"] within the cell's bbox, parses into RoadGraphCell, and stores in IndexedDB.
   */
  async fetchAndSaveCell(cellKey: string): Promise<RoadGraphCell | null> {
    if (inFlightFetches.has(cellKey)) {
      return null;
    }

    const isOnline = typeof navigator !== 'undefined' ? navigator.onLine : true;
    if (!isOnline) {
      return null;
    }

    const now = Date.now();
    if (now - lastOverpassFetchTime < MIN_FETCH_INTERVAL_MS) {
      return null; // Rate limit guard
    }

    inFlightFetches.add(cellKey);
    lastOverpassFetchTime = now;

    const { south, west, north, east } = getCellBbox(cellKey);

    // Minimal Overpass QL query: bounding box only with geometry
    const overpassQuery = `[out:json][timeout:15];(way["highway"]["highway"!~"proposed|construction|abandoned|platform|raceway"](${south.toFixed(5)},${west.toFixed(5)},${north.toFixed(5)},${east.toFixed(5)}););out body geom;`;

    try {
      const response = await fetch('https://overpass-api.de/api/interpreter', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
          'User-Agent': 'ReckonX/1.0 (https://reckonx.app)',
        },
        body: `data=${encodeURIComponent(overpassQuery)}`,
      });

      if (!response.ok) {
        throw new Error(`Overpass API responded with HTTP ${response.status}`);
      }

      const data = await response.json();
      const elements: any[] = data.elements || [];

      const nodeUsage = new Map<string, number>();
      const ways: any[] = [];

      for (const el of elements) {
        if (el.type === 'way' && el.geometry && el.geometry.length >= 2) {
          ways.push(el);
          if (el.nodes) {
            for (const nid of el.nodes) {
              const idStr = String(nid);
              nodeUsage.set(idStr, (nodeUsage.get(idStr) || 0) + 1);
            }
          }
        }
      }

      const nodesMap = new Map<string, RoadNode>();
      const edges: RoadEdge[] = [];

      for (const way of ways) {
        const geom: [number, number][] = way.geometry.map((g: any) => [g.lat, g.lon]);
        const nodeIds: string[] = way.nodes && way.nodes.length === geom.length
          ? way.nodes.map((n: any) => String(n))
          : geom.map((_, idx) => `${way.id}_n${idx}`);
        const roadType = way.tags?.highway || 'road';
        const oneway = way.tags?.oneway === 'yes' || way.tags?.oneway === '1';

        let segStartIndex = 0;

        for (let i = 0; i < nodeIds.length; i++) {
          const nid = nodeIds[i];
          const isEndpoint = i === 0 || i === nodeIds.length - 1;
          const isJunction = (nodeUsage.get(nid) || 0) >= 2;

          if ((isEndpoint || isJunction) && i > segStartIndex) {
            const fromId = nodeIds[segStartIndex];
            const toId = nodeIds[i];

            const subGeom = geom.slice(segStartIndex, i + 1);
            const lengthMeters = calculatePolylineLength(subGeom);

            nodesMap.set(fromId, { id: fromId, lat: subGeom[0][0], lng: subGeom[0][1] });
            nodesMap.set(toId, { id: toId, lat: subGeom[subGeom.length - 1][0], lng: subGeom[subGeom.length - 1][1] });

            const edgeId = `${way.id}_${segStartIndex}_${i}`;
            const forwardEdge: RoadEdge = {
              id: edgeId,
              fromNodeId: fromId,
              toNodeId: toId,
              geometry: subGeom,
              lengthMeters,
              roadType,
              oneway,
            };
            edges.push(forwardEdge);

            if (!oneway) {
              const revGeom = [...subGeom].reverse() as [number, number][];
              const reverseEdge: RoadEdge = {
                id: `${edgeId}_rev`,
                fromNodeId: toId,
                toNodeId: fromId,
                geometry: revGeom,
                lengthMeters,
                roadType,
                oneway: false,
              };
              edges.push(reverseEdge);
            }

            segStartIndex = i;
          }
        }
      }

      const cell: RoadGraphCell = {
        cellKey,
        nodes: Array.from(nodesMap.values()),
        edges,
        timestamp: Date.now(),
      };

      await this.saveCell(cell);
      return cell;
    } catch (err) {
      console.warn(`[RoadGraphCacheService] Overpass fetch failed for cell ${cellKey}:`, err);
      return null;
    } finally {
      inFlightFetches.delete(cellKey);
    }
  },

  /**
   * Pre-fetches the current cell and its 8 neighboring cells in background if missing or expired.
   */
  async prefetchRegion(lat: number, lng: number): Promise<void> {
    const centerKey = getCellKey(lat, lng);
    const targetKeys = [centerKey, ...getNeighborCellKeys(centerKey)];

    for (const key of targetKeys) {
      const isCached = await this.hasCell(key);
      if (!isCached) {
        const fetched = await this.fetchAndSaveCell(key);
        if (fetched) {
          // If a cell was fetched, wait for next cycle due to rate limiter
          break;
        }
      }
    }
  },
};
