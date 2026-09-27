/**
 * test_api_integration.ts
 * ========================
 * Level 2 Integration Tests for Fastify Backend REST API & Routes:
 * - Health Check (/health, /api/health)
 * - Session Ingestion & Query Lifecycle (/api/sessions)
 * - Batch Telemetry Sync (/api/telemetry/batch)
 * - Route Calculation (/api/routes/calculate)
 * 
 * Uses Fastify in-memory injection (fastify.inject) for zero network binding.
 */

import Fastify from 'fastify';
import cors from '@fastify/cors';
import { telemetryRoutes } from './routes/telemetry.js';
import { sessionRoutes } from './routes/sessions.js';
import { routeProxyRoutes } from './routes/routes.js';

interface TestResult {
  name: string;
  passed: boolean;
  error?: string;
  durationMs: number;
}

const results: TestResult[] = [];

function assert(condition: boolean, message: string) {
  if (!condition) throw new Error(message);
}

async function runTest(name: string, fn: () => void | Promise<void>) {
  const start = Date.now();
  try {
    await fn();
    results.push({ name, passed: true, durationMs: Date.now() - start });
    console.log(`  [PASS] ${name}`);
  } catch (err: any) {
    results.push({ name, passed: false, error: err.message, durationMs: Date.now() - start });
    console.error(`  [FAIL] ${name}: ${err.message}`);
  }
}

export async function runBackendIntegrationTests(): Promise<TestResult[]> {
  console.log("\n================================================================================");
  console.log("  BACKEND FASTIFY REST API INTEGRATION TESTS (In-Memory HTTP Injection)");
  console.log("================================================================================");

  // Build isolated Fastify instance
  const app = Fastify({ logger: false });
  await app.register(cors, { origin: '*' });

  app.get('/health', async () => ({ status: 'ok', service: 'reckonx-backend', timestamp: Date.now() }));
  app.get('/api/health', async () => ({ status: 'ok', service: 'reckonx-backend', timestamp: Date.now() }));

  await app.register(telemetryRoutes);
  await app.register(sessionRoutes);
  await app.register(routeProxyRoutes);
  await app.ready();

  const testSessionId = `test-sess-${Date.now()}`;

  // 1. Health Endpoint Test
  await runTest("Backend API: GET /api/health returns 200 OK", async () => {
    const res = await app.inject({ method: 'GET', url: '/api/health' });
    assert(res.statusCode === 200, `Expected 200, got ${res.statusCode}`);
    const body = JSON.parse(res.body);
    assert(body.status === 'ok', "Status should be 'ok'");
  });

  // 2. Session Ingestion & Retrieval Tests
  await runTest("Backend API: POST /api/sessions creates/updates a navigation session", async () => {
    const res = await app.inject({
      method: 'POST',
      url: '/api/sessions',
      payload: {
        sessionId: testSessionId,
        startTime: Date.now(),
        originAddress: 'Gateway of India, Mumbai',
        destAddress: 'Nariman Point, Mumbai',
        startCoords: [18.9220, 72.8347],
        destCoords: [18.9250, 72.8250],
        totalDistanceKm: 2.5,
        durationSeconds: 300,
        gnssPointsCount: 25,
        drPointsCount: 5,
        points: []
      }
    });
    assert(res.statusCode === 200, `Expected 200, got ${res.statusCode}: ${res.body}`);
    const body = JSON.parse(res.body);
    assert(body.success === true, "Expected success: true");
    assert(body.sessionId === testSessionId, "Expected matching sessionId");
  });

  await runTest("Backend API: GET /api/sessions lists stored sessions", async () => {
    const res = await app.inject({ method: 'GET', url: '/api/sessions' });
    assert(res.statusCode === 200, `Expected 200, got ${res.statusCode}`);
    const body = JSON.parse(res.body);
    assert(Array.isArray(body.sessions), "Expected array of sessions");
    const found = body.sessions.some((s: any) => s.id === testSessionId);
    assert(found, `Expected session ${testSessionId} to be in listed sessions`);
  });

  await runTest("Backend API: GET /api/sessions/:id retrieves session details", async () => {
    const res = await app.inject({ method: 'GET', url: `/api/sessions/${testSessionId}` });
    assert(res.statusCode === 200, `Expected 200, got ${res.statusCode}`);
    const body = JSON.parse(res.body);
    assert(body.id === testSessionId, "Expected retrieved session ID to match");
    assert(body.originAddress === 'Gateway of India, Mumbai', "Expected originAddress to match");
  });

  // 3. Telemetry Ingestion Test
  await runTest("Backend API: POST /api/telemetry persists sensor point batches", async () => {
    const res = await app.inject({
      method: 'POST',
      url: '/api/telemetry',
      payload: {
        sessionId: testSessionId,
        points: [
          {
            lat: 18.9220,
            lng: 72.8347,
            timestamp: Date.now(),
            speedKmH: 35.0,
            headingDeg: 90.0,
            accuracyMeters: 5.0,
            isDeadReckoning: false,
          },
          {
            lat: 18.9225,
            lng: 72.8350,
            timestamp: Date.now() + 100,
            speedKmH: 35.5,
            headingDeg: 90.0,
            accuracyMeters: 8.0,
            isDeadReckoning: true,
          }
        ]
      }
    });
    assert(res.statusCode === 200, `Expected 200, got ${res.statusCode}: ${res.body}`);
    const body = JSON.parse(res.body);
    assert(body.success === true, "Expected success: true");
    assert(body.pointsSaved === 2, `Expected 2 points saved, got ${body.pointsSaved}`);
  });

  // 4. Input Validation & Error Handling Tests
  await runTest("Backend API: POST /api/sessions rejects invalid payload with 400", async () => {
    const res = await app.inject({
      method: 'POST',
      url: '/api/sessions',
      payload: {
        // missing sessionId and coords
        startTime: Date.now()
      }
    });
    assert(res.statusCode === 400, `Expected 400 Bad Request, got ${res.statusCode}`);
  });

  await runTest("Backend API: POST /api/telemetry rejects empty points with 400", async () => {
    const res = await app.inject({
      method: 'POST',
      url: '/api/telemetry',
      payload: {
        sessionId: testSessionId,
        points: []
      }
    });
    assert(res.statusCode === 400, `Expected 400 Bad Request, got ${res.statusCode}`);
  });

  await runTest("Backend API: GET /api/sessions/:id returns 404 for non-existent session", async () => {
    const res = await app.inject({ method: 'GET', url: '/api/sessions/non-existent-99999' });
    assert(res.statusCode === 404, `Expected 404, got ${res.statusCode}`);
  });

  await app.close();

  console.log("--------------------------------------------------------------------------------");
  const passed = results.filter(r => r.passed).length;
  console.log(`Backend Integration Tests: ${passed}/${results.length} Passed`);
  
  if (passed !== results.length) {
    process.exit(1);
  }
  return results;
}

runBackendIntegrationTests();
