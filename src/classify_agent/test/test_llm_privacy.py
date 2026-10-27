"""Cloud parsing is opt-in and provider errors must not reveal credentials."""
import sys
from types import SimpleNamespace

import rclpy

from classify_agent.agent_node import LanguageAgent
from classify_agent.llm import LLMClient


def test_default_agent_does_not_call_cloud_with_inherited_key(monkeypatch):
    monkeypatch.setenv('DEEPSEEK_API_KEY', 'unit-test-inherited-value')
    calls = []
    monkeypatch.setattr(LLMClient, 'parse', lambda *args: calls.append(args))
    rclpy.init(args=[])
    node = LanguageAgent()
    try:
        assert node.get_parameter('use_llm').value is False
        node.handle_text('无法识别的请求')
        assert not calls
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_empty_key_returns_without_loading_sdk(monkeypatch):
    monkeypatch.setitem(sys.modules, 'openai', None)
    client = LLMClient('', 'https://example.invalid', 'example-model')
    assert not client.enabled
    assert client.parse('任意输入') is None


def test_provider_error_redacts_submitted_key(monkeypatch):
    sentinel = 'unit-test-private-value'

    def fail(**kwargs):
        raise RuntimeError('Provider rejected credential: ' + sentinel)

    fake = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=fail)))
    monkeypatch.setitem(sys.modules, 'openai', SimpleNamespace(OpenAI=lambda **kwargs: fake))
    client = LLMClient(sentinel, 'https://example.invalid', 'example-model')
    response = client.parse('任意输入')
    assert sentinel not in response['_error']
    assert '[REDACTED]' in response['_error']
