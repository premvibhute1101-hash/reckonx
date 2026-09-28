import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const fixturePath = path.resolve(__dirname, 'fixtures', 'solapur_osm_graph.json');
const cellData = JSON.parse(fs.readFileSync(fixturePath, 'utf-8'));

// Build adjacency to analyze graph topology
const nodes = new Map<string, any>();
const inDegree = new Map<string, number>();
const outDegree = new Map<string, number>();
const neighbors = new Map<string, Set<string>>();

for (const node of cellData.nodes) {
  nodes.set(node.id, node);
  inDegree.set(node.id, 0);
  outDegree.set(node.id, 0);
  neighbors.set(node.id, new Set());
}

for (const edge of cellData.edges) {
  outDegree.set(edge.fromNodeId, (outDegree.get(edge.fromNodeId) || 0) + 1);
  inDegree.set(edge.toNodeId, (inDegree.get(edge.toNodeId) || 0) + 1);

  if (!neighbors.has(edge.fromNodeId)) neighbors.set(edge.fromNodeId, new Set());
  if (!neighbors.has(edge.toNodeId)) neighbors.set(edge.toNodeId, new Set());

  neighbors.get(edge.fromNodeId)!.add(edge.toNodeId);
  neighbors.get(edge.toNodeId)!.add(edge.fromNodeId); // undirected connectivity for components
}

// Find connected components (undirected)
const visited = new Set<string>();
const components: string[][] = [];

for (const nodeId of nodes.keys()) {
  if (!visited.has(nodeId)) {
    const comp: string[] = [];
    const queue = [nodeId];
    visited.add(nodeId);

    while (queue.length > 0) {
      const u = queue.shift()!;
      comp.push(u);

      const nbrs = neighbors.get(u);
      if (nbrs) {
        for (const v of nbrs) {
          if (!visited.has(v)) {
            visited.add(v);
            queue.push(v);
          }
        }
      }
    }
    components.push(comp);
  }
}

components.sort((a, b) => b.length - a.length);

const totalNodes = nodes.size;
const largestCompSize = components[0]?.length || 0;
const largestCompPct = ((largestCompSize / totalNodes) * 100).toFixed(2);

// Degree 1 nodes
let deg1Count = 0;
for (const [nodeId, nbrs] of neighbors.entries()) {
  if (nbrs.size === 1) {
    deg1Count++;
  }
}

console.log('=== DIAGNOSIS REPORT ===');
console.log(`Total Nodes: ${totalNodes}`);
console.log(`Total Directed Edges: ${cellData.edges.length}`);
console.log(`Number of Connected Components: ${components.length}`);
console.log(`Largest Component Size: ${largestCompSize} nodes (${largestCompPct}% of all nodes)`);
console.log(`Degree 1 (Dead End) Nodes: ${deg1Count} (${((deg1Count / totalNodes) * 100).toFixed(1)}% of all nodes)`);
console.log(`Single-node components: ${components.filter((c) => c.length === 1).length}`);
console.log(`Top 5 component sizes:`, components.slice(0, 5).map((c) => c.length));
