import { db } from './database';

/**
 * Test database connection
 * Run with: npm run test:db
 */
async function testConnection() {
  try {
    console.log('Testing database connection...');

    // Test basic query
    const result = await db.query('SELECT NOW() as current_time, version() as pg_version');

    console.log('✓ Database connection successful!');
    console.log('PostgreSQL Version:', result.rows[0].pg_version);
    console.log('Current Time:', result.rows[0].current_time);

    // Close the pool
    await db.end();
    process.exit(0);
  } catch (error) {
    console.error('✗ Database connection failed:', error);
    await db.end();
    process.exit(1);
  }
}

testConnection();
