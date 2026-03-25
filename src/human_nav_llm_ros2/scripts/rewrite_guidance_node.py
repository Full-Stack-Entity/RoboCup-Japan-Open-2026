#!/usr/bin/env python3
# Copyright 2026 Human Navigation LLM bridge — Subtask B (local Ollama only)
"""ROS 2 service: RewriteGuidance -> Ollama OpenAI-compatible API at 127.0.0.1 only."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any, Dict, List, Tuple

import rclpy
from rclpy.node import Node

from human_nav_llm_ros2.srv import RewriteGuidance


def _normalize_host(host: str) -> str:
    h = host.strip().lower()
    if h in ("localhost", "::1"):
        return "127.0.0.1"
    return h


def _build_system_prompt(max_output_chars: int) -> str:
    return f"""You rewrite short English instructions for a volunteer in a VR Human Navigation task.
Rules:
- Output ONLY English. One or two short sentences.
- Use simple, concrete words. Prefer landmark + position (e.g. on/near/in the [furniture]) as in the draft.
- Do NOT add objects, colors, or places that are not clearly implied by the draft or the JSON context.
- Prefer large, stable, non-interactive landmarks (table/cabinet/shelf/refrigerator) for "on/in/near".
- Do NOT use rare or literary vocabulary.
- Maximum length: about {max_output_chars} characters (stay well under this if possible).
- No quotes, no numbering, no preamble — only the instruction text.

Stage templates (strict):
- If phase is "pick", use exactly:
  "Please grab the <target> on <main location>, near <landmark>."
  Use "on" (never "near") for the main location. Prefer context_json.pick_on and context_json.pick_near.
  If no landmark is known, omit ", near <landmark>".
- If phase is "place", use exactly:
  "Please place it on <destination>, near <landmark>, where the robot points."
  Use "on" (never "near") for the main destination. Prefer context_json.place_on and context_json.place_near.
  Keep destination and landmark as separate noun phrases.

Grammar constraints:
- Main position MUST use "on": grab X on A, place it on A. Never use "near" for the main position.
- Secondary landmark uses ", near B" (comma before near). Never merge two items into one phrase.
- If the draft contains ", near ", you MUST keep the comma exactly where it is; never write "on the X near the Y" without a comma before "near".
- Use the on-slot and near-slot as two separate noun phrases from context_json; do not concatenate them.
- BAD: "near the cafe set mota table" (cafe set and mota table are two different items).
- BAD: "on the desk lamp, near the cabinet" when on=desk and near=lamp.
- GOOD: "on the mota table, near the cafe set" — one item per slot."""


def _ollama_chat(
    base_url: str,
    model: str,
    system_prompt: str,
    user_content: str,
    timeout_sec: float,
) -> Tuple[bool, str]:
    """POST /v1/chat/completions. Returns (ok, text_or_error)."""
    url = base_url.rstrip("/") + "/v1/chat/completions"
    body: Dict[str, Any] = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_content},
        ],
        "stream": False,
    }
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
        parsed = json.loads(raw)
        choices = parsed.get("choices") or []
        if not choices:
            return False, ""
        msg = choices[0].get("message") or {}
        content = (msg.get("content") or "").strip()
        if not content:
            return False, ""
        return True, content
    except urllib.error.HTTPError as e:
        try:
            detail = e.read().decode("utf-8", errors="replace")
        except Exception:
            detail = str(e)
        return False, f"HTTPError {e.code}: {detail[:200]}"
    except (urllib.error.URLError, json.JSONDecodeError, OSError, TimeoutError) as e:
        return False, str(e)[:200]
    except Exception as e:
        return False, str(e)[:200]


class RewriteGuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("rewrite_guidance_node")

        self.declare_parameter("ollama_host", "127.0.0.1")
        self.declare_parameter("ollama_port", 11434)
        self.declare_parameter("model", "llama3.2:3b")
        self.declare_parameter("request_timeout_sec", 30.0)
        self.declare_parameter("max_output_chars", 280)

        host_raw = self.get_parameter("ollama_host").get_parameter_value().string_value
        host = _normalize_host(host_raw)
        if host != "127.0.0.1":
            self.get_logger().error(
                f"ollama_host must be loopback (127.0.0.1 or localhost), got '{host_raw}'."
            )
            raise RuntimeError("Invalid ollama_host — only 127.0.0.1/localhost allowed")

        port = self.get_parameter("ollama_port").get_parameter_value().integer_value
        self._base_url = f"http://127.0.0.1:{port}"
        self._model = self.get_parameter("model").get_parameter_value().string_value
        self._timeout = self.get_parameter("request_timeout_sec").get_parameter_value().double_value
        self._max_out = self.get_parameter("max_output_chars").get_parameter_value().integer_value

        self._srv = self.create_service(RewriteGuidance, "rewrite_guidance", self._handle)
        self.get_logger().info(
            f"rewrite_guidance service ready -> {self._base_url} model={self._model} "
            f"timeout={self._timeout}s max_out={self._max_out}"
        )

    def _handle(self, request: RewriteGuidance.Request, response: RewriteGuidance.Response):
        response.success = False
        response.rewritten = ""

        try:
            draft = (request.draft or "").strip()
            if not draft:
                self.get_logger().warn("empty draft")
                return response

            phase = request.phase or ""
            ctx = request.context_json or "{}"

            max_chars = max(32, min(self._max_out, 400))
            system_prompt = _build_system_prompt(max_chars)

            user_content = (
                f"Phase: {phase}\n"
                f"Context JSON (do not invent objects not present here):\n{ctx}\n\n"
                f"Draft instruction:\n{draft}\n\n"
                "Rewrite using the stage template rules above. "
                "If information is missing, keep draft meaning and produce a grammatical sentence. "
                "Output only the new instruction."
            )

            ok, text = _ollama_chat(
                self._base_url,
                self._model,
                system_prompt,
                user_content,
                self._timeout,
            )
            if not ok or not text:
                self.get_logger().warn(f"Ollama call failed or empty: {text!r}")
                return response

            if len(text) > max_chars:
                text = text[:max_chars].rstrip()

            response.rewritten = text
            response.success = True
            self.get_logger().debug(f"rewritten ({len(text)} chars): {text[:120]}...")
        except Exception as e:
            self.get_logger().error(f"rewrite_guidance handler error: {e}")
        return response


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = RewriteGuidanceNode()
    except RuntimeError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
