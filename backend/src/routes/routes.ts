import { FastifyInstance } from 'fastify';
import { db } from '../db.js';

export async function routeProxyRoutes(fastify: FastifyInstance) {
  fastify.get('/api/routes', async (request, reply) => {
    const { profile = 'car', coordinates } = request.query as {
      profile?: string;
      coordinates?: string;
    };

    if (!coordinates) {
      return reply.status(400).send({ error: 'Query parameter coordinates (lng,lat;lng,lat) is required' });
    }

    const validProfiles = ['car', 'bike', 'foot'];
    const normalizedProfile = validProfiles.includes(profile) ? profile : 'car';

    try {
      // 1. Check cache first
      const cached = await db.routeCache.findUnique({
        where: {
          profile_coordinates: {
            profile: normalizedProfile,
            coordinates,
          },
        },
      });

      if (cached) {
        fastify.log.info(`Route cache hit for ${normalizedProfile} ${coordinates}`);
        return reply.send(JSON.parse(cached.routeData));
      }

      // 2. Fetch from OSRM public backend
      const osrmProfileMap: Record<string, string> = {
        car: 'driving',
        bike: 'bike',
        foot: 'foot',
      };
      const osrmProfile = osrmProfileMap[normalizedProfile] || 'driving';
      const osrmUrl = `https://router.project-osrm.org/route/v1/${osrmProfile}/${coordinates}?overview=full&geometries=geojson&steps=true`;

      fastify.log.info(`Proxying route request to OSRM: ${osrmUrl}`);
      const response = await fetch(osrmUrl);

      if (!response.ok) {
        return reply.status(response.status).send({ error: 'OSRM routing upstream error' });
      }

      const routeDataJson = await response.json();

      // 3. Save to database cache asynchronously
      await db.routeCache.upsert({
        where: {
          profile_coordinates: {
            profile: normalizedProfile,
            coordinates,
          },
        },
        update: {
          routeData: JSON.stringify(routeDataJson),
        },
        create: {
          profile: normalizedProfile,
          coordinates,
          routeData: JSON.stringify(routeDataJson),
        },
      });

      return reply.send(routeDataJson);
    } catch (err: any) {
      fastify.log.error(err, 'Failed to proxy route request');
      return reply.status(500).send({ error: 'Route proxy failed', details: err.message });
    }
  });
}
