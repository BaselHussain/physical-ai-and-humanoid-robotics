import asyncio
from pathlib import Path
from dotenv import load_dotenv
from sqlalchemy import text

# Load .env
env_path = Path(__file__).resolve().parent / ".env"
load_dotenv(dotenv_path=env_path)

from src.auth.config import engine

async def verify():
    async with engine.connect() as conn:
        result = await conn.execute(text(
            "SELECT column_name, data_type FROM information_schema.columns "
            "WHERE table_name='users' ORDER BY ordinal_position"
        ))
        print("Users table columns:")
        for row in result:
            print(f"  - {row[0]}: {row[1]}")

if __name__ == "__main__":
    asyncio.run(verify())
