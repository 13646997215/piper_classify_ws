"""A stopped scene publisher must not keep reporting old object counts."""
from unittest.mock import Mock

from classify_agent.agent_node import LanguageAgent


def test_expired_scene_is_unavailable(monkeypatch):
    node = Mock()
    node.last_scene = object()
    node._scene_wall = 0.0
    monkeypatch.setattr('classify_agent.agent_node.time.monotonic', lambda: 100.0)
    monkeypatch.setattr('classify_agent.agent_node.time.sleep', lambda _: None)
    assert LanguageAgent._scene_snapshot(node) is None
    node._call_trigger.assert_called_once()


def test_fresh_scene_does_not_call_service(monkeypatch):
    node = Mock()
    node.last_scene = object()
    node._scene_wall = 99.5
    monkeypatch.setattr('classify_agent.agent_node.time.monotonic', lambda: 100.0)
    assert LanguageAgent._scene_snapshot(node) is node.last_scene
    node._call_trigger.assert_not_called()
