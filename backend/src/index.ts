import Fastify from 'fastify';
import cors from '@fastify/cors';
import { db } from './db.js';
import { telemetryRoutes } from './routes/telemetry.js';
import { sessionRoutes } from './routes/sessions.js';
import { routeProxyRoutes } from './routes/routes.js';

const PORT = Number(process.env.PORT) || 3001;
const HOST = process.env.HOST || '0.0.0.0';

const fastify = Fastify({
  logger: true,
});

async function main() {
  // CORS configuration
  await fastify.register(cors, {
    origin: '*',
    methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  });

  // Health check endpoint
  fastify.get('/health', async () => {
    return { status: 'ok', service: 'reckonx-backend', timestamp: Date.now() };
  });

  fastify.get('/api/health', async () => {
    return { status: 'ok', service: 'reckonx-backend', timestamp: Date.now() };
  });

  // Register domain routes
  await fastify.register(telemetryRoutes);
  await fastify.register(sessionRoutes);
  await fastify.register(routeProxyRoutes);

  try {
    await fastify.listen({ port: PORT, host: HOST });
    console.log(`ReckonX Backend running at http://localhost:${PORT}`);
  } catch (err) {
    fastify.log.error(err);
    process.exit(1);
  }
}

// Graceful shutdown
process.on('SIGINT', async () => {
  await db.$disconnect();
  await fastify.close();
  process.exit(0);
});

process.on('SIGTERM', async () => {
  await db.$disconnect();
  await fastify.close();
  process.exit(0);
});

main();
