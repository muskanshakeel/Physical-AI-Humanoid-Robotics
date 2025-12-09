# rag-chatbot/backend/app/services/neon_db_service.py

import os
from dotenv import load_dotenv
import psycopg2

load_dotenv()

# TODO: Replace with actual connection details or environment variables
NEON_DB_URL = os.getenv("NEON_DB_URL", "postgresql://user:password@host:port/database")

def get_db_connection():
    """Establishes and returns a connection to the Neon Serverless Postgres database."""
    try:
        conn = psycopg2.connect(NEON_DB_URL)
        return conn
    except Exception as e:
        print(f"Error connecting to Neon DB: {e}")
        return None

def close_db_connection(conn):
    """Closes the database connection."""
    if conn:
        conn.close()

if __name__ == "__main__":
    # Example usage:
    conn = get_db_connection()
    if conn:
        print("Successfully connected to Neon DB (placeholder).")
        # You can add a simple query here to test the connection
        # with conn.cursor() as cur:
        #     cur.execute("SELECT version();")
        #     db_version = cur.fetchone()
        #     print(f"PostgreSQL database version: {db_version}")
        close_db_connection(conn)
        print("Neon DB connection closed.")
