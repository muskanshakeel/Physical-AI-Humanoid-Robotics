from pydantic_settings import BaseSettings
from typing import Optional
from pydantic import Field

class Settings(BaseSettings):
    """
    Application settings loaded from environment variables
    """
    # Database settings
    database_url: str = Field(default="postgresql+asyncpg://user:password@localhost/dbname", alias="DATABASE_URL")

    # Qdrant settings
    qdrant_url: str = Field(default="", alias="QDRANT_URL")
    qdrant_api_key: str = Field(default="", alias="QDRANT_API_KEY")

    # Authentication settings
    better_auth_secret: str = Field(default="your-default-secret-key-change-in-production", alias="BETTER_AUTH_SECRET")
    access_token_expire_minutes: int = Field(default=30, alias="ACCESS_TOKEN_EXPIRE_MINUTES")

    # AI Model settings
    ai_model_provider: str = Field(default="qwen", alias="AI_MODEL_PROVIDER")  # qwen or openai
    qwen_api_key: str = Field(default="", alias="QWEN_API_KEY")
    openai_api_key: str = Field(default="", alias="OPENAI_API_KEY")

    # Application settings
    app_env: str = Field(default="development", alias="APP_ENV")
    debug: str = Field(default="False", alias="DEBUG")  # Keep as string, convert when needed

    # Performance settings
    response_timeout_seconds: int = Field(default=30, alias="RESPONSE_TIMEOUT_SECONDS")
    max_query_length: int = Field(default=1000, alias="MAX_QUERY_LENGTH")
    vector_top_k: int = Field(default=5, alias="VECTOR_TOP_K")
    vector_threshold: float = Field(default=0.7, alias="VECTOR_THRESHOLD")

    # Rate limiting
    rate_limit_requests: int = Field(default=100, alias="RATE_LIMIT_REQUESTS")
    rate_limit_window_seconds: int = Field(default=3600, alias="RATE_LIMIT_WINDOW_SECONDS")  # 1 hour

    class Config:
        env_file = ".env"
        case_sensitive = True
        populate_by_name = True  # Allow both alias and field name

    @property
    def debug_bool(self) -> bool:
        """
        Convert the debug string to a boolean value
        """
        return self.debug.lower() in ('true', '1', 'yes', 'on')

# Create a global settings instance
settings = Settings()

def get_settings() -> Settings:
    """
    Get the application settings
    """
    return settings