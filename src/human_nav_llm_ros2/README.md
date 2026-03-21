# human_nav_llm_ros2

- **接口**：`human_nav_llm_ros2/srv/RewriteGuidance`
- **节点**：`rewrite_guidance_node`（仅连接本机 `127.0.0.1` Ollama OpenAI 兼容 API）

## 运行

1. 本机启动 Ollama，并 `ollama pull` 与参数 `model` 一致的模型。
sudo systemctl start ollama 
2. source install/setup.bash
3. ros2 launch human_nav_ros2 sample_launch.py enable_llm_rewrite:=true 

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `ollama_host` | `127.0.0.1` | 仅允许 loopback（`localhost` 会规范为 127.0.0.1） |
| `ollama_port` | `11434` | Ollama HTTP 端口 |
| `model` | `llama3.2:3b` | 模型名 |
| `request_timeout_sec` | `30.0` | HTTP 超时（秒） |
| `max_output_chars` | `280` | 改写结果最大字符数（硬上限仍须满足 competition ≤400） |

示例：`ros2 run human_nav_llm_ros2 rewrite_guidance_node --ros-args -p model:=qwen2.5:3b`

## 测试

```bash
ros2 service call /rewrite_guidance human_nav_llm_ros2/srv/RewriteGuidance \
  "{draft: 'Get the cup near the table.', phase: 'pick', context_json: '{}'}"
```

Ollama 未启动时应返回 `success: false`。

Launch 一键启动（含桥接 + 可选 LLM）：见 `human_nav_ros2` 的 `sample_launch.py` / `sample_with_rviz_launch.py`，参数 `enable_llm_rewrite:=true`。

赛前检查单（可提交仓库）：`human_nav_ros2/HUMAN_NAV_LLM_CHECKLIST.md`（安装后在 `share/human_nav_ros2/`）。

详见仓库 `docs/SUBTASK2_LLM_方案分析.md`（若 `docs/` 被 gitignore，请以包内 `HUMAN_NAV_LLM_CHECKLIST.md` 为准）。
