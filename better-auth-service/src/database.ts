import { Pool } from 'pg';
import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

/**
 * PostgreSQL connection pool for Better Auth
 * Uses Kysely-compatible pg Pool instance
 */
export const db = new Pool({
  connectionString: process.env.DATABASE_URL,
  max: 10, // Maximum number of clients in the pool
  idleTimeoutMillis: 30000, // How long a client is allowed to remain idle before being closed
  connectionTimeoutMillis: 2000, // How long to wait for a connection
});

// Handle pool errors
db.on('error', (err) => {
  console.error('Unexpected database pool error:', err);
  process.exit(-1);
});

// Graceful shutdown
process.on('SIGTERM', async () => {
  console.log('SIGTERM received, closing database pool...');
  await db.end();
  process.exit(0);
});

process.on('SIGINT', async () => {
  console.log('SIGINT received, closing database pool...');
  await db.end();
  process.exit(0);
});

export default db;
