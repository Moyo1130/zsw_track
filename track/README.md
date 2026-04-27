# Track 项目说明

这个目录是一套面向机器狗目标跟随的本地调试工程，核心目标是把视觉检测结果转换成机器狗 UDP 连续运动指令。

当前主链路是：

`YOLO HTTP / YOLO WebSocket / 本地 JSON 场景` -> `FrameInput` -> `P4TrackerController` -> `TwistCommand` -> `track_live.py` 轴映射 -> `totalController.py` UDP 下发

## 核心入口

如果你只关心“现在实际跑哪一个脚本”，优先看这几个文件：

- `track_live.py`
  现在的主入口。接 YOLO WebSocket、YOLO HTTP 或本地 scenarios，把控制结果直接下发到机器狗。
- `p4_tracker/controller.py`
  控制器核心。负责把检测框变成 `linear_x / angular_z`。
- `totalController.py`
  UDP 底层驱动。真正负责给机器狗发控制包。
- `TRACK_LIVE_运行教程.md`
  现场运行教程，适合直接照着执行。

## 常见使用方式

### 1. 真实运行

```powershell
python track_live.py --yolo-ws ws://<狗IP>:8001/ws/detection --turn-axis-min 0.30 --turn-axis-max 0.32 --action-s 0.10 --cooldown-s 0.10 --start-norm 0.30 --stop-norm 0.15 --seen-n 4 --desired-fill 0.85 --forward-axis-max 0.35 --forward-gain 2.5 --far-start-boost 0.15
```

说明：

- `--ip` 现在可以省略；脚本会优先从 `--yolo-ws` 或 `--yolo-url` 里推断机器狗 IP。
- 如果 YOLO 服务主机和机器狗控制目标不是同一台，再额外显式写 `--ip <机器狗IP>`。

### 2. 只看映射，不连狗

```powershell
python track_live.py --check-mapping
```

### 3. dry-run 看日志

```powershell
python track_live.py --dry-run --yolo-ws ws://<狗IP>:8001/ws/detection
```

### 4. 单帧验证某个 JSON 会触发什么动作

```powershell
python track_json_one.py samples/frames/turn_left.json --dry-run
```

### 5. 不接 YOLO，直接用合成场景仿真

```powershell
python simulate_tracking.py --scene all
```

### 6. 只做底层 UDP 烟测

```powershell
python udp_motion_demo.py --ip <机器狗IP>
```

## 文件说明

下面按文件和目录说明作用。

### 根目录 Python 脚本

| 文件 | 作用 | 什么时候看它 |
|---|---|---|
| `track_live.py` | 主运行脚本。支持 `--scenarios`、`--yolo-url`、`--yolo-ws` 三种输入源；负责把控制器输出映射成机器狗前进/转向轴，并通过 UDP 连续下发。 | 真实联机运行、调参、排查“为什么现在这样走”时首先看它。 |
| `track_json_one.py` | 单帧验证工具。读取一个 JSON 帧，算出一次动作，可 `--dry-run`，也可真机执行一次短动作。 | 想验证“这一帧到底会左转/右转/前进吗”时看它。 |
| `simulate_tracking.py` | 合成 bbox 场景仿真器。不依赖 YOLO，可打印控制器输出，也可把合成结果逐帧发给真机。 | 调控制器逻辑、验证远近变化、看左右跟踪趋势时很有用。 |
| `udp_motion_demo.py` | UDP 教学/烟测脚本。演示前进、后退、左转、右转以及前进+转向的底层组合。 | 确认“狗能不能被 UDP 控起来”时先跑它。 |
| `main.py` | 最早的控制器回放入口。读取 `samples/scenarios.json`，打印每帧决策和等价 ROS `/cmd_vel`。 | 只想看控制器原始输出，不关心 UDP 细节时可用。 |
| `totalController.py` | 机器狗 UDP 控制底层实现。包含命令字、心跳、模式切换、持续运动、语音指令等。 | 需要查底层协议、死区、线程发送逻辑、模式切换顺序时看它。 |

### 控制器包

| 文件 | 作用 | 说明 |
|---|---|---|
| `p4_tracker/controller.py` | 控制器核心实现。定义 `BBox`、`Detection`、`FrameInput`、`TwistCommand`、`P4TrackerController`。 | 真正决定目标居中时怎么转、目标变远时怎么前进、丢失目标时怎么缓冲/搜索。 |
| `p4_tracker/__init__.py` | 包导出层。把 `controller.py` 里的核心类型重新导出。 | 作用很轻，主要是方便 `from p4_tracker import ...`。 |

### 文档与配置

| 文件 | 作用 | 说明 |
|---|---|---|
| `README.md` | 这个总说明文档。 | 新人先从这里看全局。 |
| `TRACK_LIVE_运行教程.md` | 现场运行教程。 | 写的是“怎么跑”，不是“代码怎么组织”。 |
| `YOLO客户端开发文档.md` | YOLO WebSocket 协议说明。 | 说明 token、payload 结构、ping/pong 和字段格式。 |
| `requirements-yolo.txt` | YOLO WebSocket 相关最小依赖。 | 目前主要是 `websockets` 和 `python-dotenv`。 |
| `.env` | 本地环境变量文件。 | 主要用于放 `API_TOKEN`；已被 `.gitignore` 忽略。 |
| `.gitignore` | Git 忽略规则。 | 当前主要忽略 `.env`。 |
| `test.txt` | 临时命令记录。 | 目前只是几条手写命令，不是正式文档。 |

### 样例数据目录

| 文件 | 作用 |
|---|---|
| `samples/scenarios.json` | 通用逐帧回放样例，给 `main.py` 和 `track_live.py --scenarios` 使用。 |
| `samples/lock_recovery_scenarios.json` | 偏重目标锁定/丢失/恢复的样例。 |
| `samples/edge_case_scenarios.json` | 边界场景样例，方便测异常输入或极端状态。 |
| `samples/frames/forward.json` | 单帧“应该以前进为主”的样例。 |
| `samples/frames/turn_left.json` | 单帧左转样例。 |
| `samples/frames/turn_right.json` | 单帧右转样例。 |
| `samples/frames/stop_near.json` | 单帧近距离停止样例。 |

### 前端可视化目录

| 文件 | 作用 | 说明 |
|---|---|---|
| `web/index.html` | 前端页面骨架。 | 提供 WS 地址、token、参数输入和画布区域。 |
| `web/app.js` | 前端逻辑。 | 连接 YOLO WebSocket、画 bbox、在浏览器里计算并显示控制器指令。 |
| `web/style.css` | 页面样式。 | 仅负责展示。 |
| `web/README.md` | 前端使用说明。 | 说明怎么开本地静态服务器、怎么连 WebSocket。 |

### 自动生成或非核心目录

| 路径 | 说明 |
|---|---|
| `__pycache__/` | Python 自动生成的字节码缓存，不是源码。 |
| `p4_tracker/__pycache__/` | 同上。 |
| `tests/__pycache__/` | 当前目录里只有缓存，没有对应测试源码。说明这个工作区曾经跑过测试，但源码文件不在当前目录快照内。 |

## 关键文件之间的关系

### 1. 控制器层

- `p4_tracker/controller.py` 负责算控制量。
- 输入是一帧检测结果。
- 输出是 `TwistCommand(linear_x, angular_z, ...)`。

### 2. 映射层

- `track_live.py` 负责把 `TwistCommand` 转成机器狗真正能吃的 `forward_speed / turn_speed`。
- 这里处理了：
- 前进轴死区抬升
- 转向轴死区抬升
- 只允许前进，不做后退
- 转向脉冲门控
- 远距离时提高 `start_norm`

### 3. 下发层

- `totalController.py` 负责 UDP 发包。
- `udp_motion_demo.py` 里的 `SoftExitController` 和 `safe_stop()` 则负责更适合现场调试的安全关闭方式。

## 建议阅读顺序

如果你刚接手这个目录，建议按这个顺序看：

1. `README.md`
2. `TRACK_LIVE_运行教程.md`
3. `track_live.py`
4. `p4_tracker/controller.py`
5. `udp_motion_demo.py`
6. `totalController.py`
7. `simulate_tracking.py`
8. `web/README.md`

## 现场调试建议

1. 先跑 `python track_live.py --check-mapping`。
2. 再跑 `python track_live.py --dry-run ...` 看 `fill / linear_x / angular_z / fwd / turn`。
3. 再做 `udp_motion_demo.py` 或 `track_live.py --smoke-udp` 真机烟测。
4. 最后再上完整 `track_live.py --yolo-ws ...` 闭环。

## 依赖

最常用的是：

```powershell
pip install websockets python-dotenv
```

如果只跑纯本地 JSON / scenarios / 仿真，不接 YOLO WebSocket，很多场景下不一定需要这两个依赖。

## 备注

- 当前项目更偏“调试工作台”，不是一个完整打包发布的 Python 包。
- 文档、脚本、样例数据都放在同一个目录里，方便现场直接改直接跑。
- 若后续要继续整理，通常第一步会是把 `README.md`、`TRACK_LIVE_运行教程.md`、`web/README.md` 的交叉引用再收紧一些。
