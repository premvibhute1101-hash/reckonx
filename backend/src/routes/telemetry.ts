import { FastifyInstance } from 'fastify';
import { db } from '../db.js';

export interface TelemetryPointBody {
  timestamp: number;
  lat: number;
  lng: number;
  accuracyMeters?: number;
  speedKmH?: number;
  headingDeg?: number;
  altitudeMeters?: number | null;
  isDeadReckoning?: boolean;
  rawInsVelX?: number;
  rawInsVelY?: number;
  aiCorrectedVelX?: number | null;
  aiCorrectedVelY?: number | null;
  aiConfidence?: number | null;
  gpsGroundTruthLat?: number;
  gpsGroundTruthLng?: number;
}

export async function telemetryRoutes(fastify: FastifyInstance) {
  fastify.post('/api/telemetry', async (request, reply) => {
    const { sessionId, points } = request.body as {
      sessionId?: string;
      points: TelemetryPointBody[];
    };

    if (!Array.isArray(points) || points.length === 0) {
      return reply.status(400).send({ error: 'points array is required and cannot be empty' });
    }

    try {
      // Safely check if foreign key Session ID exists before linking
      let validSessionId: string | null = null;
      if (sessionId) {
        const sessionExists = await db.session.findUnique({
          where: { id: sessionId },
          select: { id: true },
        });
        if (sessionExists) {
          validSessionId = sessionId;
        }
      }

      const records = points.map((p) => ({
        sessionId: validSessionId,
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

      return reply.send({ success: true, pointsSaved: records.length });
    } catch (err: any) {
      fastify.log.error(err, 'Failed to save telemetry points');
      return reply.status(500).send({ error: 'Failed to persist telemetry points', details: err.message });
    }
  });
}
