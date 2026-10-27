"""Small OpenAI-compatible client used only for intent parsing.

Works with DeepSeek, OpenAI-compatible local servers (Ollama/vLLM) and any
endpoint exposing /v1/chat/completions.  The returned JSON is always validated
by parser.validate_spec before use.
"""
import json
from typing import Optional

from classify_agent.parser import SYSTEM_PROMPT


class LLMClient:
    def __init__(self, api_key: str, base_url: str, model: str,
                 timeout: float = 20.0) -> None:
        self.api_key = api_key
        self.base_url = base_url
        self.model = model
        self.timeout = timeout

    @property
    def enabled(self) -> bool:
        return bool(self.api_key)

    def parse(self, text: str) -> Optional[dict]:
        """Return a JSON object, or None when the LLM is disabled/failed."""
        if not self.enabled:
            return None
        try:
            from openai import OpenAI
        except ImportError:
            return {"_error": "python openai package is not installed"}
        try:
            client = OpenAI(api_key=self.api_key, base_url=self.base_url,
                            timeout=self.timeout)
            request = {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": text},
                ],
                "temperature": 0.1,
            }
            try:
                response = client.chat.completions.create(
                    response_format={"type": "json_object"}, **request)
            except Exception:
                # Some local OpenAI-compatible servers do not implement
                # response_format; retry once without it.
                response = client.chat.completions.create(**request)
            content = (response.choices[0].message.content or "").strip()
            if content.startswith("```"):
                content = content.strip("`")
                if content.startswith("json"):
                    content = content[4:]
            return json.loads(content)
        except Exception as error:  # network, auth, malformed JSON, ...
            # Some providers echo the submitted key in authentication errors.
            return {"_error": str(error).replace(self.api_key, '[REDACTED]')}
