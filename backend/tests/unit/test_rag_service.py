import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from backend.src.models.query import Query
from backend.src.models.response import Response


@pytest.fixture
def rag_service():
    with patch('backend.src.services.rag_service.VectorIndexService'), \
         patch('backend.src.services.rag_service.OpenAIService'), \
         patch('backend.src.services.rag_service.QwenAIService'):

        from backend.src.services.rag_service import RAGService

        # Create a mock database session and other dependencies
        mock_db = AsyncMock()
        mock_vector_service = MagicMock()
        mock_ai_service = MagicMock()
        return RAGService(db=mock_db, vector_service=mock_vector_service, ai_service=mock_ai_service)


@pytest.mark.asyncio
async def test_process_query_success():
    with patch('backend.src.services.rag_service.VectorIndexService'), \
         patch('backend.src.services.rag_service.OpenAIService'), \
         patch('backend.src.services.rag_service.QwenAIService'):

        from backend.src.services.rag_service import RAGService

        # Create a mock database session and other dependencies
        mock_db = AsyncMock()
        mock_vector_service = MagicMock()
        mock_ai_service = MagicMock()
        rag_service = RAGService(db=mock_db, vector_service=mock_vector_service, ai_service=mock_ai_service)

        query_data = {
            'query': 'What is artificial intelligence?',
            'user_id': 1,
            'session_id': 'session123'
        }

        # Mock vector search results
        mock_vector_results = [
            {'content': 'Artificial intelligence is a branch of computer science...', 'source': 'chapter1.txt'},
            {'content': 'AI systems can perform tasks that would typically require human intelligence...', 'source': 'chapter2.txt'}
        ]

        # Mock AI response
        mock_ai_response = 'Artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.'

        rag_service.vector_service.search.return_value = mock_vector_results
        rag_service.ai_service.generate_response.return_value = mock_ai_response

        with patch('backend.src.services.rag_service.datetime') as mock_datetime:
            mock_datetime.utcnow.return_value = '2023-01-01T00:00:00Z'

            result = await rag_service.process_query(query_data['query'], query_data['user_id'], query_data['session_id'])

            # Verify the result
            assert result['content'] == mock_ai_response
            assert 'sources' in result
            assert len(result['sources']) == 2
            assert rag_service.db.add.called
            assert rag_service.db.commit.called


@pytest.mark.asyncio
async def test_process_query_with_context():
    with patch('backend.src.services.rag_service.VectorIndexService'), \
         patch('backend.src.services.rag_service.OpenAIService'), \
         patch('backend.src.services.rag_service.QwenAIService'):

        from backend.src.services.rag_service import RAGService

        # Create a mock database session and other dependencies
        mock_db = AsyncMock()
        mock_vector_service = MagicMock()
        mock_ai_service = MagicMock()
        rag_service = RAGService(db=mock_db, vector_service=mock_vector_service, ai_service=mock_ai_service)

        query_data = {
            'query': 'Explain neural networks',
            'user_id': 1,
            'session_id': 'session123',
            'context': 'machine learning concepts'
        }

        mock_vector_results = [
            {'content': 'Neural networks are computing systems inspired by the human brain...', 'source': 'chapter5.txt'}
        ]
        mock_ai_response = 'Neural networks are computing systems inspired by the human brain, designed to recognize patterns.'

        rag_service.vector_service.search.return_value = mock_vector_results
        rag_service.ai_service.generate_response.return_value = mock_ai_response

        with patch('backend.src.services.rag_service.datetime') as mock_datetime:
            mock_datetime.utcnow.return_value = '2023-01-01T00:00:00Z'

            result = await rag_service.process_query(
                query_data['query'],
                query_data['user_id'],
                query_data['session_id'],
                context=query_data['context']
            )

            # Verify the result
            assert result['content'] == mock_ai_response
            assert 'sources' in result
            assert result['sources'][0] == 'chapter5.txt'


@pytest.mark.asyncio
async def test_process_query_no_results():
    with patch('backend.src.services.rag_service.VectorIndexService'), \
         patch('backend.src.services.rag_service.OpenAIService'), \
         patch('backend.src.services.rag_service.QwenAIService'):

        from backend.src.services.rag_service import RAGService

        # Create a mock database session and other dependencies
        mock_db = AsyncMock()
        mock_vector_service = MagicMock()
        mock_ai_service = MagicMock()
        rag_service = RAGService(db=mock_db, vector_service=mock_vector_service, ai_service=mock_ai_service)

        query_data = {
            'query': 'What is quantum computing?',
            'user_id': 1,
            'session_id': 'session123'
        }

        # Mock empty vector search results
        mock_vector_results = []

        rag_service.vector_service.search.return_value = mock_vector_results
        rag_service.ai_service.generate_response.return_value = "I couldn't find relevant information about quantum computing in the book content."

        with patch('backend.src.services.rag_service.datetime') as mock_datetime:
            mock_datetime.utcnow.return_value = '2023-01-01T00:00:00Z'

            result = await rag_service.process_query(query_data['query'], query_data['user_id'], query_data['session_id'])

            # Verify the result
            assert "couldn't find relevant information" in result['content']
            assert result['sources'] == []


@pytest.mark.asyncio
async def test_get_query_history():
    with patch('backend.src.services.rag_service.VectorIndexService'), \
         patch('backend.src.services.rag_service.OpenAIService'), \
         patch('backend.src.services.rag_service.QwenAIService'):

        from backend.src.services.rag_service import RAGService

        # Create a mock database session and other dependencies
        mock_db = AsyncMock()
        mock_vector_service = MagicMock()
        mock_ai_service = MagicMock()
        rag_service = RAGService(db=mock_db, vector_service=mock_vector_service, ai_service=mock_ai_service)

        user_id = 1
        mock_queries = [
            Query(
                id=1,
                user_id=user_id,
                content='What is AI?',
                timestamp='2023-01-01T10:00:00Z'
            ),
            Query(
                id=2,
                user_id=user_id,
                content='Explain neural networks',
                timestamp='2023-01-01T10:05:00Z'
            )
        ]

        mock_db_result = MagicMock()
        mock_db_result.scalars.return_value.all.return_value = mock_queries
        rag_service.db.execute.return_value = mock_db_result

        result = await rag_service.get_query_history(user_id)

        # Verify the result
        assert len(result) == 2
        assert result[0]['content'] == 'What is AI?'
        assert result[1]['content'] == 'Explain neural networks'


@pytest.mark.asyncio
async def test_get_response_for_query():
    with patch('backend.src.services.rag_service.VectorIndexService'), \
         patch('backend.src.services.rag_service.OpenAIService'), \
         patch('backend.src.services.rag_service.QwenAIService'):

        from backend.src.services.rag_service import RAGService

        # Create a mock database session and other dependencies
        mock_db = AsyncMock()
        mock_vector_service = MagicMock()
        mock_ai_service = MagicMock()
        rag_service = RAGService(db=mock_db, vector_service=mock_vector_service, ai_service=mock_ai_service)

        query_id = 1
        mock_response = Response(
            id=1,
            query_id=query_id,
            content='Response content here',
            sources=['source1.txt', 'source2.txt'],
            confidence_score=0.95,
            timestamp='2023-01-01T10:00:00Z',
            response_time_ms=1200
        )

        mock_db_result = MagicMock()
        mock_db_result.scalars.return_value.first.return_value = mock_response
        rag_service.db.execute.return_value = mock_db_result

        result = await rag_service.get_response_for_query(query_id)

        # Verify the result
        assert result['content'] == 'Response content here'
        assert result['sources'] == ['source1.txt', 'source2.txt']
        assert result['confidence_score'] == 0.95