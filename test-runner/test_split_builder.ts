import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';
import { haversineDistance } from '../src/services/ekf/OutputStabilizer';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

async function testSplitBuilder() {
  const south = 17.6300;
  const west = 75.8700;
  const north = 17.6500;
  const east = 75.9100;

  const overpassQuery = `[out:json][timeout:25];(way["highway"]["highway"!~"proposed|construction|abandoned|platform|raceway"](${south.toFixed(5)},${west.toFixed(5)},${north.toFixed(5)},${east.toFixed(5)}););out body geom;`;

  console.log('Fetching Overpass data...');
  const res = await fetch('https://overpass-api.de/api/interpreter', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/x-www-form-urlencoded',
      'User-Agent': 'ReckonX/1.0 (https://reckonx.app)',
    },
    body: `data=${encodeURIComponent(overpassQuery)}`,
  });

  const rawJson = await res.json();
  const elements = rawJson.elements || [];
  console.log(`Received ${elements.length} elements from Overpass.`);

  // 1. Count node occurrences across all ways
  const nodeUsage = new Map<string, number>();
  const ways: any[] = [];

  for (const el of elements) {
    if (el.type === 'way' && el.geometry && el.geometry.length >= 2 && el.nodes) {
      ways.push(el);
      for (const nid of el.nodes) {
        const idStr = String(nid);
        nodeUsage.set(idStr, (nodeUsage.get(idStr) || 0) + 1);
      }
    }
  }

  // 2. Build properly split edges between junctions
  const nodesMap = new Map<string, { id: string; lat: number; lng: number }>();
  const edges: any[] = [];

  for (const way of ways) {
    const geom: [number, number][] = way.geometry.map((g: any) => [g.lat, g.lon]);
    const nodeIds: string[] = way.nodes.map((n: any) => String(n));
    const roadType = way.tags?.highway || 'road';
    const oneway = way.tags?.oneway === 'yes' || way.tags?.oneway === '1';

    let segStartIndex = 0;

    for (let i = 0; i < nodeIds.length; i++) {
      const nid = nodeIds[i];
      const isEndpoint = (i === 0 || i === nodeIds.length - 1);
      const isJunction = (nodeUsage.get(nid) || 0) >= 2;

      // If this is a junction/endpoint and not the start of current sub-segment
      if ((isEndpoint || isJunction) && i > segStartIndex) {
        const fromId = nodeIds[segStartIndex];
        const toId = nodeIds[i];

        const subGeom = geom.slice(segStartIndex, i + 1);
        let lengthMeters = 0;
        for (let k = 0; k < subGeom.length - 1; k++) {
          lengthMeters += haversineDistance(subGeom[k][0], subGeom[k][1], subGeom[k + 1][0], subGeom[k + 1][1]);
        }

        nodesMap.set(fromId, { id: fromId, lat: subGeom[0][0], lng: subGeom[0][1] });
        nodesMap.set(toId, { id: toId, lat: subGeom[subGeom.length - 1][0], lng: subGeom[subGeom.length - 1][1] });

        const edgeId = `${way.id}_${segStartIndex}_${i}`;
        edges.push({
          id: edgeId,
          fromNodeId: fromId,
          toNodeId: toId,
          geometry: subGeom,
          lengthMeters,
          roadType,
          oneway,
        });

        if (!oneway) {
          edges.push({
            id: `${edgeId}_rev`,
            fromNodeId: toId,
            toNodeId: fromId,
            geometry: [...subGeom].reverse(),
            lengthMeters,
            roadType,
            oneway: false,
          });
        }

        segStartIndex = i;
      }
    }
  }

  console.log(`Properly Split Graph: ${nodesMap.size} Junction Nodes, ${edges.length} Directed Edges`);

  // Analyze connectivity
  const neighbors = new Map<string, Set<string>>();
  for (const n of nodesMap.keys()) neighbors.set(n, new Set());
  for (const e of edges) {
    neighbors.get(e.fromNodeId)?.add(e.toNodeId);
    neighbors.get(e.toNodeId)?.add(e.fromNodeId);
  }

  const visited = new Set<string>();
  const components: string[][] = [];
  for (const nodeId of nodesMap.keys()) {
    if (!visited.has(nodeId)) {
      const comp: string[] = [];
      const queue = [nodeId];
      visited.add(nodeId);
      while (queue.length > 0) {
        const u = queue.shift()!;
        comp.push(u);
        for (const v of neighbors.get(u) || []) {
          if (!visited.has(v)) {
            visited.add(v);
            queue.push(v);
          }
        }
      }
      components.push(comp);
    }
  }

  components.sort((a, b) => b.length - a.length);
  const largestCompSize = components[0]?.length || 0;
  const largestCompPct = ((largestCompSize / nodesMap.size) * 100).toFixed(2);

  console.log(`Number of Connected Components: ${components.length}`);
  console.log(`Largest Component Size: ${largestCompSize} nodes (${largestCompPct}% of all nodes!)`);
  console.log(`Top 5 component sizes:`, components.slice(0, 5).map((c) => c.length));

  // Save the updated fixture
  const fixturePath = path.resolve(__dirname, 'fixtures', 'solapur_osm_graph.json');
  const cell = {
    cellKey: '705_3035',
    nodes: Array.from(nodesMap.values()),
    edges,
    timestamp: Date.now(),
  };
  fs.writeFileSync(fixturePath, JSON.stringify(cell, null, 2), 'utf-8');
  console.log(`Saved updated connected graph to ${fixturePath}`);
}

testSplitBuilder().catch(console.error);
