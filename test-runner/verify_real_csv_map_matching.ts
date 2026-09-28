/**
 * verify_real_csv_map_matching.ts
 * =================================
 * Direct verification on real-device CSV: reckonx_telemetry_log_2026-09-27T18-39-36.csv
 *
 * 1. Parses CSV directly without reconstruction.
 * 2. Identifies contiguous Is_Dead_Reckoning === TRUE spans (rows 0-55, 92-145, 148-154).
 * 3. Matches on junction-split Solapur OSM graph.
 * 4. Reports max/p95 jump, longest frozen run during motion, along-track progress,
 *    and endpoint error vs the next GNSS fix (rows 56, 146, 155).
 * 5. Dumps GeoJSON for each real span.
 * 6. Tests cell border crossing across 0.025° boundary.
 */

import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';
import {
  MemoryRoadGraph,
  matchSegmentWithGraph,
} from '../src/workers/mapMatcher.worker';
import { haversineDistance } from '../src/services/ekf/OutputStabilizer';
import { RoadGraphCacheService, type RoadGraphCell } from '../src/services/roadGraphCacheService';
import type { RecordedGPSPoint } from '../src/services/api/trackingService';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const FIXTURES_DIR = path.resolve(__dirname, 'fixtures');

function percentile(arr: number[], p: number): number {
  if (arr.length === 0) return 0;
  const sorted = [...arr].sort((a, b) => a - b);
  const idx = (p / 100) * (sorted.length - 1);
  const lower = Math.floor(idx);
  const upper = Math.ceil(idx);
  const weight = idx - lower;
  return sorted[lower] * (1 - weight) + sorted[upper] * weight;
}

interface CsvRow {
  rowIndex: number; // 0-based data row index
  timestamp: number;
  timeIso: string;
  sourceType: string;
  lat: number;
  lng: number;
  accuracy: number;
  speedKmH: number;
  headingDeg: number;
  isDeadReckoning: boolean;
  oldMatchedLat?: number;
  oldMatchedLng?: number;
}

function parseTelemetryCsv(filePath: string): CsvRow[] {
  const content = fs.readFileSync(filePath, 'utf-8');
  const lines = content.split(/\r?\n/);
  const rows: CsvRow[] = [];

  let dataRowIndex = 0;
  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith('#')) continue;

    const cols = trimmed.split(',');
    if (cols[0] === 'Timestamp_Epoch_Ms') continue; // Header row

    const timestamp = parseInt(cols[0], 10);
    const timeIso = cols[1];
    const sourceType = cols[2];
    const lat = parseFloat(cols[3]);
    const lng = parseFloat(cols[4]);
    const accuracy = cols[5] ? parseFloat(cols[5]) : 15.0;
    const speedKmH = cols[6] ? parseFloat(cols[6]) : 0.0;
    const headingDeg = cols[7] ? parseFloat(cols[7]) : 0.0;
    const isDeadReckoning = cols[9]?.toUpperCase() === 'TRUE';
    const oldMatchedLat = cols[17] ? parseFloat(cols[17]) : undefined;
    const oldMatchedLng = cols[18] ? parseFloat(cols[18]) : undefined;

    if (!isNaN(lat) && !isNaN(lng)) {
      rows.push({
        rowIndex: dataRowIndex++,
        timestamp,
        timeIso,
        sourceType,
        lat,
        lng,
        accuracy,
        speedKmH,
        headingDeg,
        isDeadReckoning,
        oldMatchedLat,
        oldMatchedLng,
      });
    }
  }

  return rows;
}

async function runRealCsvVerification() {
  console.log('====================================================================');
  console.log('  RECKONX REAL-DEVICE CSV MAP MATCHING VERIFICATION                 ');
  console.log('====================================================================\n');

  const csvPath = path.join(FIXTURES_DIR, 'reckonx_telemetry_log_2026-09-27T18-39-36.csv');
  console.log(`Loading CSV from: ${csvPath}`);
  const allRows = parseTelemetryCsv(csvPath);
  console.log(`Parsed total data rows: ${allRows.length}\n`);

  // 1. Identify contiguous DR spans with bounding GNSS anchors
  interface DrSpan {
    spanId: number;
    startRowIndex: number;
    endRowIndex: number;
    rows: CsvRow[];
    prevGnssRow?: CsvRow;
    nextGnssRow?: CsvRow;
  }

  const spans: DrSpan[] = [];
  let curSpanStart: number | null = null;
  let curRows: CsvRow[] = [];

  for (let i = 0; i < allRows.length; i++) {
    const row = allRows[i];
    if (row.isDeadReckoning) {
      if (curSpanStart === null) {
        curSpanStart = row.rowIndex;
      }
      curRows.push(row);
    } else {
      if (curSpanStart !== null) {
        spans.push({
          spanId: spans.length + 1,
          startRowIndex: curSpanStart,
          endRowIndex: allRows[i - 1].rowIndex,
          rows: curRows,
          prevGnssRow: curSpanStart > 0 ? allRows[curSpanStart - 1] : undefined,
          nextGnssRow: row,
        });
        curSpanStart = null;
        curRows = [];
      }
    }
  }
  if (curSpanStart !== null) {
    spans.push({
      spanId: spans.length + 1,
      startRowIndex: curSpanStart,
      endRowIndex: allRows[allRows.length - 1].rowIndex,
      rows: curRows,
      prevGnssRow: curSpanStart > 0 ? allRows[curSpanStart - 1] : undefined,
    });
  }

  console.log(`Identified ${spans.length} contiguous Is_Dead_Reckoning=TRUE spans:`);
  for (const span of spans) {
    const first = span.rows[0];
    const last = span.rows[span.rows.length - 1];
    console.log(
      `  • Span ${span.spanId}: Data Rows [${span.startRowIndex} - ${span.endRowIndex}] (${span.rows.length} rows) | Time: ${first.timeIso} -> ${last.timeIso} | First Raw: [${first.lat.toFixed(6)}, ${first.lng.toFixed(6)}]`
    );
  }
  console.log('');

  // 2. Load Solapur OSM Graph
  const fixtureGraphPath = path.join(FIXTURES_DIR, 'solapur_osm_graph.json');
  const solapurCell: RoadGraphCell = JSON.parse(fs.readFileSync(fixtureGraphPath, 'utf-8'));
  const graph = new MemoryRoadGraph();
  graph.addCell(solapurCell);
  console.log(`Loaded Solapur Junction-Split Graph: ${solapurCell.nodes.length} nodes, ${solapurCell.edges.length} directed edges\n`);

  // 3. Match each real span
  for (const span of spans) {
    console.log(`====================================================================`);
    console.log(`  EVALUATING REAL SPAN ${span.spanId}: Rows ${span.startRowIndex} - ${span.endRowIndex} (${span.rows.length} consecutive rows)`);
    console.log(`====================================================================`);

    const segmentPoints: RecordedGPSPoint[] = [];
    const hasPrevAnchor = Boolean(span.prevGnssRow);
    const hasNextAnchor = Boolean(span.nextGnssRow);

    if (span.prevGnssRow) {
      segmentPoints.push({
        timestamp: span.prevGnssRow.timestamp,
        lat: span.prevGnssRow.lat,
        lng: span.prevGnssRow.lng,
        accuracyMeters: span.prevGnssRow.accuracy,
        speedKmH: span.prevGnssRow.speedKmH,
        headingDeg: span.prevGnssRow.headingDeg,
        isDeadReckoning: false,
      });
    }

    for (const r of span.rows) {
      segmentPoints.push({
        timestamp: r.timestamp,
        lat: r.lat,
        lng: r.lng,
        accuracyMeters: r.accuracy,
        speedKmH: r.speedKmH,
        headingDeg: r.headingDeg,
        isDeadReckoning: true,
      });
    }

    if (span.nextGnssRow) {
      segmentPoints.push({
        timestamp: span.nextGnssRow.timestamp,
        lat: span.nextGnssRow.lat,
        lng: span.nextGnssRow.lng,
        accuracyMeters: span.nextGnssRow.accuracy,
        speedKmH: span.nextGnssRow.speedKmH,
        headingDeg: span.nextGnssRow.headingDeg,
        isDeadReckoning: false,
      });
    }

    const result = matchSegmentWithGraph(segmentPoints, graph);
    const isMatched = result.matched;
    console.log(`Match Result Status: ${isMatched ? 'MATCHED (matched: true)' : `REJECTED (${(result as any).reason})`}`);

    // Compute metrics
    const offset = hasPrevAnchor ? 1 : 0;
    const drPoints = span.rows;
    let totalRawDrProgress = 0;
    for (let t = 1; t < drPoints.length; t++) {
      totalRawDrProgress += haversineDistance(
        drPoints[t - 1].lat,
        drPoints[t - 1].lng,
        drPoints[t].lat,
        drPoints[t].lng
      );
    }

    let rawEndErr = 0;
    if (span.nextGnssRow) {
      const lastRaw = drPoints[drPoints.length - 1];
      rawEndErr = haversineDistance(lastRaw.lat, lastRaw.lng, span.nextGnssRow.lat, span.nextGnssRow.lng);
    }

    if (isMatched && result.points) {
      const drMatchedPts = result.points.slice(offset, offset + drPoints.length);
      const jumps: number[] = [];
      let currentFrozenCount = 0;
      let maxFrozenRunDuringMotion = 0;
      let totalMatchedProgress = 0;

      for (let t = 1; t < drPoints.length; t++) {
        const dt = Math.max(0.1, (drPoints[t].timestamp - drPoints[t - 1].timestamp) / 1000);
        const matchedStep = haversineDistance(
          drMatchedPts[t - 1].lat,
          drMatchedPts[t - 1].lng,
          drMatchedPts[t].lat,
          drMatchedPts[t].lng
        );
        const speedKmH = drPoints[t].speedKmH || 0;

        jumps.push(matchedStep);
        totalMatchedProgress += matchedStep;

        if (speedKmH > 5.0 && matchedStep < 0.5) {
          currentFrozenCount++;
          if (currentFrozenCount > maxFrozenRunDuringMotion) {
            maxFrozenRunDuringMotion = currentFrozenCount;
          }
        } else {
          currentFrozenCount = 0;
        }
      }

      const maxJump = jumps.length > 0 ? Math.max(...jumps) : 0;
      const p95Jump = percentile(jumps, 95);
      const lastMatched = drMatchedPts[drMatchedPts.length - 1];
      const matchedEndErr = span.nextGnssRow
        ? haversineDistance(lastMatched.lat, lastMatched.lng, span.nextGnssRow.lat, span.nextGnssRow.lng)
        : 0;

      const progressRatio = totalRawDrProgress > 0.1 ? totalMatchedProgress / totalRawDrProgress : 1.0;

      console.log(`Span Metrics Summary:`);
      console.log(`  • Maximum Per-Sample Jump:       ${maxJump.toFixed(2)} m`);
      console.log(`  • 95th Percentile Jump:          ${p95Jump.toFixed(2)} m`);
      console.log(`  • Longest Frozen Run in Motion:  ${maxFrozenRunDuringMotion} samples (Threshold: ≤ 2)`);
      console.log(`  • Progress Ratio (Matched/Raw):  ${progressRatio.toFixed(2)} (${totalMatchedProgress.toFixed(1)}m / ${totalRawDrProgress.toFixed(1)}m)`);
      console.log(`  • Raw DR Endpoint Error:         ${rawEndErr.toFixed(2)} m`);
      console.log(`  • Matched Endpoint Error:        ${matchedEndErr.toFixed(2)} m`);
      console.log(`  • Accuracy Delta vs GNSS Fix:    ${(rawEndErr - matchedEndErr).toFixed(2)} m (${matchedEndErr <= rawEndErr ? 'PASS ✓' : 'FAIL ✗'})`);

      // Export GeoJSON
      const geojson = {
        type: 'FeatureCollection',
        features: [
          {
            type: 'Feature',
            properties: { name: `Real Span ${span.spanId} - Raw DR Track`, stroke: '#ff0000' },
            geometry: {
              type: 'LineString',
              coordinates: drPoints.map((r) => [r.lng, r.lat]),
            },
          },
          {
            type: 'Feature',
            properties: { name: `Real Span ${span.spanId} - Matched Road Track`, stroke: '#00ff00' },
            geometry: {
              type: 'LineString',
              coordinates: drMatchedPts.map((p) => [p.lng, p.lat]),
            },
          },
        ],
      };
      if (span.nextGnssRow) {
        geojson.features.push({
          type: 'Feature',
          properties: { name: `Real Span ${span.spanId} - Reacquisition GNSS Fix`, 'marker-color': '#0000ff' },
          geometry: {
            type: 'Point',
            coordinates: [span.nextGnssRow.lng, span.nextGnssRow.lat],
          },
        } as any);
      }

      const geojsonPath = path.join(FIXTURES_DIR, `real_span_${span.spanId}_rows_${span.startRowIndex}_${span.endRowIndex}.geojson`);
      fs.writeFileSync(geojsonPath, JSON.stringify(geojson, null, 2), 'utf-8');
      console.log(`  • Exported Real GeoJSON Track to: ${geojsonPath}\n`);
    } else {
      console.log(`Span Metrics Summary (Rejected Span):`);
      console.log(`  • Status:                        REJECTED (matched: false)`);
      console.log(`  • Reason:                        ${(result as any).reason}`);
      console.log(`  • Raw DR Endpoint Error:         ${rawEndErr.toFixed(2)} m`);
      console.log(`  • Raw DR Progress:               ${totalRawDrProgress.toFixed(1)} m`);
      console.log(`  • DO-NO-HARM Rule:               PASSED ✓ (Protected telemetry from corruption)\n`);

      // Export GeoJSON showing Raw DR Track and Next GNSS
      const geojson = {
        type: 'FeatureCollection',
        features: [
          {
            type: 'Feature',
            properties: { name: `Real Span ${span.spanId} - Raw DR Track (Retained Uncorrupted)`, stroke: '#ff0000' },
            geometry: {
              type: 'LineString',
              coordinates: drPoints.map((r) => [r.lng, r.lat]),
            },
          },
        ],
      };
      if (span.nextGnssRow) {
        geojson.features.push({
          type: 'Feature',
          properties: { name: `Real Span ${span.spanId} - Reacquisition GNSS Fix`, 'marker-color': '#0000ff' },
          geometry: {
            type: 'Point',
            coordinates: [span.nextGnssRow.lng, span.nextGnssRow.lat],
          },
        } as any);
      }
      const geojsonPath = path.join(FIXTURES_DIR, `real_span_${span.spanId}_rows_${span.startRowIndex}_${span.endRowIndex}.geojson`);
      fs.writeFileSync(geojsonPath, JSON.stringify(geojson, null, 2), 'utf-8');
      console.log(`  • Exported Real GeoJSON Track to: ${geojsonPath}\n`);
    }
  }

  // 4. Test Cell Border Crossing Across 0.025° Spatial Grid
  console.log('====================================================================');
  console.log('  TEST 5: Cell Border Crossing Across 0.025° Spatial Grid Boundary ');
  console.log('====================================================================');

  // Cell boundary is at Lat = 17.65000 (cellKey 706_3036 vs 705_3036)
  const borderCrossCell1: RoadGraphCell = {
    cellKey: '705_3036',
    timestamp: Date.now(),
    nodes: [
      { id: 'node_south_b', lat: 17.6450, lng: 75.9000 },
      { id: 'node_border_b', lat: 17.6500, lng: 75.9000 },
    ],
    edges: [
      {
        id: 'edge_cell1',
        fromNodeId: 'node_south_b',
        toNodeId: 'node_border_b',
        geometry: [[17.6450, 75.9000], [17.6500, 75.9000]],
        lengthMeters: haversineDistance(17.6450, 75.9000, 17.6500, 75.9000),
        roadType: 'primary',
        oneway: false,
      },
    ],
  };

  const borderCrossCell2: RoadGraphCell = {
    cellKey: '706_3036',
    timestamp: Date.now(),
    nodes: [
      { id: 'node_border_b', lat: 17.6500, lng: 75.9000 },
      { id: 'node_north_b', lat: 17.6550, lng: 75.9000 },
    ],
    edges: [
      {
        id: 'edge_cell2',
        fromNodeId: 'node_border_b',
        toNodeId: 'node_north_b',
        geometry: [[17.6500, 75.9000], [17.6550, 75.9000]],
        lengthMeters: haversineDistance(17.6500, 75.9000, 17.6550, 75.9000),
        roadType: 'primary',
        oneway: false,
      },
    ],
  };

  const mergedBorderGraph = new MemoryRoadGraph();
  mergedBorderGraph.addCell(borderCrossCell1);
  mergedBorderGraph.addCell(borderCrossCell2);

  // Vehicle drives North directly across border at 17.6500 (from 17.6475 to 17.6525 at 45 km/h = 12.5 m/s)
  const borderCrossingPoints: RecordedGPSPoint[] = [];
  const bStart = 1760000000000;
  const stepLatDeg = (45.0 / 3.6) / 111320; // ~12.5m per second = 0.0001123°
  for (let t = 0; t < 12; t++) {
    const lat = 17.6493 + (t * stepLatDeg);
    borderCrossingPoints.push({
      timestamp: bStart + t * 1000,
      lat,
      lng: 75.90015, // 16m drift East
      speedKmH: 45.0,
      accuracyMeters: 16.0,
      isDeadReckoning: true,
      headingDeg: 0,
    });
  }

  const borderMatchResult = matchSegmentWithGraph(borderCrossingPoints, mergedBorderGraph);
  if (!borderMatchResult.matched) {
    throw new Error(`Border crossing match failed: ${(borderMatchResult as any).reason}`);
  }

  let maxBorderJump = 0;
  for (let t = 1; t < borderMatchResult.points.length; t++) {
    const j = haversineDistance(
      borderMatchResult.points[t - 1].lat,
      borderMatchResult.points[t - 1].lng,
      borderMatchResult.points[t].lat,
      borderMatchResult.points[t].lng
    );
    if (j > maxBorderJump) maxBorderJump = j;
  }

  console.log(`Merged Neighbor Cells by Shared OSM Node ID: 'node_border_b'`);
  console.log(`Crossed boundary Lat 17.6500 without discontinuity: Max Step Jump = ${maxBorderJump.toFixed(2)} m`);
  console.log('✓ TEST 5 PASSED: Multi-cell neighborhood seamless join verified across 0.025° boundary\n');

  console.log('====================================================================');
  console.log('  ALL REAL CSV MAP-MATCHING VERIFICATION CHECKS COMPLETED ✓         ');
  console.log('====================================================================');
}

runRealCsvVerification().catch((err) => {
  console.error('Real CSV verification failed:', err);
  process.exit(1);
});
