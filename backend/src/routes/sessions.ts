import { FastifyInstance } from 'fastify';
import { db } from '../db.js';
import { TelemetryPointBody } from './telemetry.js';

export interface SessionPayloadBody {
  sessionId: string;
  startTime: number;
  endTime?: number;
  originAddress: string;
  destAddress: string;
  startCoords: [number, number]; // [lat, lng]
  destCoords: [number, number];  // [lat, lng]
  points: TelemetryPointBody[];
  totalDistanceKm: number;
  durationSeconds: number;
  gnssPointsCount: number;
  drPointsCount: number;
}

export async function sessionRoutes(fastify: FastifyInstance) {
  // POST /api/sessions - Save full trip session
  fastify.post('/api/sessions', async (request, reply) => {
    const payload = request.body as SessionPayloadBody;

    if (!payload.sessionId || !payload.startCoords || !payload.destCoords) {
      return reply.status(400).send({ error: 'Missing required session fields' });
    }

    try {
      const session = await db.session.upsert({
        where: { id: payload.sessionId },
        update: {
          endTime: payload.endTime ?? null,
          totalDistanceKm: payload.totalDistanceKm,
          durationSeconds: payload.durationSeconds,
          gnssPointsCount: payload.gnssPointsCount,
          drPointsCount: payload.drPointsCount,
        },
        create: {
          id: payload.sessionId,
          startTime: payload.startTime,
          endTime: payload.endTime ?? null,
          originAddress: payload.originAddress,
          destAddress: payload.destAddress,
          startLat: payload.startCoords[0],
          startLng: payload.startCoords[1],
          destLat: payload.destCoords[0],
          destLng: payload.destCoords[1],
          totalDistanceKm: payload.totalDistanceKm,
          durationSeconds: payload.durationSeconds,
          gnssPointsCount: payload.gnssPointsCount,
          drPointsCount: payload.drPointsCount,
        },
      });

      if (Array.isArray(payload.points) && payload.points.length > 0) {
        // Save points linked to session
        const records = payload.points.map((p) => ({
          sessionId: session.id,
          timestamp: p.timestamp,
          lat: p.lat,
          lng: p.lng,
          accuracyMeters: p.accuracyMeters ?? null,
          speedKmH: p.speedKmH ?? null,
          headingDeg: p.headingDeg ?? null,
          altitudeMeters: p.altitudeMeters ?? null,
          isDeadReckoning: Boolean(p.isDeadReckoning),
          rawInsVelX: p.rawInsVelX ?? null,
          rawInsVelY: p.rawInsVelY ?? null,
          aiCorrectedVelX: p.aiCorrectedVelX ?? null,
          aiCorrectedVelY: p.aiCorrectedVelY ?? null,
          aiConfidence: p.aiConfidence ?? null,
          gpsGroundTruthLat: p.gpsGroundTruthLat ?? null,
          gpsGroundTruthLng: p.gpsGroundTruthLng ?? null,
        }));

        await db.telemetryPoint.createMany({
          data: records,
        });
      }

      return reply.send({ success: true, sessionId: session.id });
    } catch (err: any) {
      fastify.log.error(err, 'Failed to persist session');
      return reply.status(500).send({ error: 'Failed to persist session', details: err.message });
    }
  });

  // GET /api/sessions/:id - Retrieve session by ID
  fastify.get('/api/sessions/:id', async (request, reply) => {
    const { id } = request.params as { id: string };

    try {
      const session = await db.session.findUnique({
        where: { id },
        include: {
          points: {
            orderBy: { timestamp: 'asc' },
          },
        },
      });

      if (!session) {
        return reply.status(404).send({ error: `Session ${id} not found` });
      }

      return reply.send(session);
    } catch (err: any) {
      fastify.log.error(err, `Failed to retrieve session ${id}`);
      return reply.status(500).send({ error: 'Failed to retrieve session', details: err.message });
    }
  });

  // GET /api/sessions - List recent sessions
  fastify.get('/api/sessions', async (request, reply) => {
    try {
      const sessions = await db.session.findMany({
        orderBy: { createdAt: 'desc' },
        take: 20,
      });
      return reply.send({ sessions });
    } catch (err: any) {
      fastify.log.error(err, 'Failed to list sessions');
      return reply.status(500).send({ error: 'Failed to list sessions', details: err.message });
    }
  });
}
