# Human Navigation + 本机 LLM — 赛前检查单（子任务 D）

与 `docs/SUBTASK2_LLM_方案分析.md` 一致；本文件在仓库内，便于版本管理（`docs/` 若被 gitignore 仍可查阅本路径）。

---

## 1. 环境与软件冻结

| 检查项 | 记录 |
|--------|------|
| 比赛规定软件冻结前 **N** 小时（见当年 S-OPL 规则） | N = ______ |
| 冻结日期/时间 | |
| 本仓库 commit / tag | |
| Ubuntu / ROS 2 发行版 | |
| `colcon build` 通过（至少 `human_nav_llm_ros2`、`human_nav_ros2`） | ☐ |

---

## 2. Ollama（仅本机 127.0.0.1）

| 检查项 | 记录 |
|--------|------|
| `ollama --version` | |
| 赛场用模型名（与 launch `llm_ollama_model` 一致） | |
| `ollama list` 含上述模型 | ☐ |
| `curl -s http://127.0.0.1:11434/api/tags` 成功 | ☐ |
| 无外网机器：模型已预先 `pull` / 缓存已备份 | ☐ |

---

## 3. Launch 与参数

**不含 RViz（标准联机）**

```bash
source install/setup.bash
# 关 LLM（默认）
ros2 launch human_nav_ros2 sample_launch.py

# 开 LLM（需先或同时保证 Ollama 已运行）
ros2 launch human_nav_ros2 sample_launch.py enable_llm_rewrite:=true
```

**含 RViz**

```bash
ros2 launch human_nav_ros2 sample_with_rviz_launch.py enable_llm_rewrite:=true
```

常用覆盖参数：

| Launch 参数 | 含义 |
|-------------|------|
| `enable_llm_rewrite` | `true` / `false` |
| `llm_ollama_model` | 如 `llama3.2:3b`、`qwen2.5:3b` |
| `llm_http_timeout_sec` | Python 节点调 Ollama 的 HTTP 超时 |
| `llm_client_timeout_sec` | C++ `spin_until_future_complete` 超时 |
| `llm_service_name` | 默认 `/rewrite_guidance` |

| 检查项 | ☐ |
|--------|---|
| `enable_llm_rewrite:=false` 能完整跑通（Unity + Quest） | ☐ |
| `enable_llm_rewrite:=true` 至少 1 次完整 session | ☐ |
| 临场策略：默认关 LLM 或带一键关 launch | ☐ |

---

## 4. 功能与规则向检查

| 检查项 | ☐ |
|--------|---|
| `/human_navigation/message/guidance_message` 单条长度 ≤ 400（抽查日志或 echo） | ☐ |
| 关 Ollama 或关 `rewrite_guidance_node` 时仍能完成任务（降级为原文骨架） | ☐ |
| 指示条数与延迟可接受（对照 Japan Open S-OPL 15 条罚分规则） | ☐ |

---

## 5. 单独终端启动（不通过 launch 启 LLM 时）

若只启动桥接 + `human_navigation_sample`，可另开终端：

```bash
source install/setup.bash
ros2 run human_nav_llm_ros2 rewrite_guidance_node
ros2 run human_nav_ros2 human_navigation_sample --ros-args -p use_llm_rewrite:=true
```

---

## 6. 签字 / 日期

- 执行人：________________  
- 日期：________________  
