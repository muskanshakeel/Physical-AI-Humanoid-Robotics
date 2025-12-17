import asyncio
import os
import sys
import inspect
from sqlalchemy.ext.asyncio import create_async_engine

# Add the backend/src directory to the Python path
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
src_dir = os.path.join(current_dir, 'src')
sys.path.insert(0, src_dir)

from src.config.settings import settings
from models.user import User
from models.session import Session
from models.query import Query
from models.response import Response
from models.book_content import BookContent

# Get database URL from settings
DATABASE_URL = settings.database_url

async def create_tables():
    # Create async engine
    engine = create_async_engine(DATABASE_URL)

    async with engine.begin() as conn:
        # Create all tables
        await conn.run_sync(User.metadata.create_all)
        await conn.run_sync(Session.metadata.create_all)
        await conn.run_sync(Query.metadata.create_all)
        await conn.run_sync(Response.metadata.create_all)
        await conn.run_sync(BookContent.metadata.create_all)

    await engine.dispose()
    print("Tables created successfully!")

if __name__ == "__main__":
    asyncio.run(create_tables())