# YOLO客户端开发文档

> 版本：v2.0
> 更新日期：2026-03-16
> 面向：机器狗侧客户端开发者

---

## 快速上手

```
连接地址：ws://<机器狗IP>:8001/ws/detection?token=<TOKEN>
```

连接成功后，服务端**持续推送**当前摄像头画面的 person 检测结果（约 20fps）。  
客户端只需做两件事：**处理推送的检测数据** + **响应心跳 ping**。

---

## 一、鉴权

Token 由服务端通过 `.env` 文件配置，不在代码或命令行中明文出现。  
客户端同样应将 Token 存储在本地配置文件中。

**`.env` 配置示例：**

```
API_TOKEN=aetherismoyo88
```

将 Token 附在 WebSocket 连接 URL 的查询参数中：

```
ws://192.168.1.68:8001/ws/detection?token=<TOKEN>
```

- Token 错误时，服务端在握手阶段直接拒绝连接，关闭码 `4001`
- 连接建立后无需再发送任何认证消息

**Python 示例（从本地 .env 读取 token）：**

```python
from dotenv import load_dotenv
import os

load_dotenv()                        # 加载 .env 文件
TOKEN = os.getenv("API_TOKEN", "")

uri = f"ws://192.168.1.68:8001/ws/detection?token={TOKEN}"
```

---

## 二、接收检测数据

连接建立后服务端以约 **20fps** 推送 JSON 消息。  
消息分两类，通过 `type` 字段区分：

| `type` 字段 | 含义 |
|---|---|
| 不存在 | 检测数据（正常处理） |
| `"ping"` | 心跳探测（需回复 pong，见第三章） |

### 检测数据格式

**有人时：**

```json
{
  "code": 0,
  "msg": "success",
  "data": {
    "timestamp": 1710000123.456,
    "image_width": 1920,
    "image_height": 1080,
    "detected": true,
    "person_count": 2,
    "persons": [
      {
        "track_id": 1,
        "bbox": { "x1": 120, "y1": 80, "x2": 310, "y2": 450 },
        "confidence": 0.91
      },
      {
        "track_id": 2,
        "bbox": { "x1": 680, "y1": 200, "x2": 850, "y2": 520 },
        "confidence": 0.76
      }
    ]
  }
}
```

**无人时：**

```json
{
  "code": 0,
  "msg": "success",
  "data": {
    "timestamp": 1710000123.456,
    "image_width": 1920,
    "image_height": 1080,
    "detected": false,
    "person_count": 0,
    "persons": []
  }
}
```

### 字段说明

| 字段 | 类型 | 说明 |
|---|---|---|
| `code` | int | 0 = 正常；非 0 见错误码说明 |
| `data.timestamp` | float | 帧采集时的 Unix 时间戳（秒） |
| `data.image_width` | int | 摄像头图像宽度（像素） |
| `data.image_height` | int | 摄像头图像高度（像素） |
| `data.detected` | bool | 当前帧是否有人 |
| `data.person_count` | int | 当前帧检测到的人数 |
| `data.persons` | array | 人员列表，无人时为 `[]` |
| `persons[].track_id` | int | 跨帧稳定 ID，同一个人在连续帧中 ID 不变 |
| `persons[].bbox` | object | 边界框像素坐标：左上角 `(x1, y1)`，右下角 `(x2, y2)` |
| `persons[].confidence` | float | 置信度，范围 0~1 |

---

## 三、心跳

服务端每隔 **10 秒**发送一条 ping 消息：

```json
{ "type": "ping", "ts": 1710000123.456 }
```

客户端须在收到 ping 后 **5 秒内**回复 pong，否则服务端主动断开连接（关闭码 `1001`）：

```json
{ "type": "pong", "ts": 1710000123.456 }
```

> `ts` 建议原样回传，也可填写客户端当前时间。

**处理示例：**

```python
async for raw in ws:
    msg = json.loads(raw)
    if msg.get("type") == "ping":
        await ws.send(json.dumps({"type": "pong", "ts": msg["ts"]}))
    else:
        handle_detection(msg)
```

---

## 四、断线重连

网络波动或服务重启后客户端应自动重连，建议使用**指数退避**策略：

| 参数 | 值 |
|---|---|
| 首次重连等待 | 1 秒 |
| 最大重连间隔 | 16 秒 |
| 退避规律 | 1s → 2s → 4s → 8s → 16s → 16s → ... |
| 是否无限重连 | 是 |

**完整示例（含鉴权 + 心跳 + 断线重连）：**

```python
import asyncio, json, os, time
import websockets
from dotenv import load_dotenv

load_dotenv()
TOKEN = os.getenv("API_TOKEN", "")
HOST  = "192.168.1.68"
PORT  = 8001
URI   = f"ws://{HOST}:{PORT}/ws/detection?token={TOKEN}"


def handle_detection(msg: dict):
    """在此处理检测结果，触发机器狗业务逻辑。"""
    if msg["code"] != 0:
        print(f"服务端错误: code={msg['code']} msg={msg['msg']}")
        return
    data = msg["data"]
    if data["detected"]:
        print(f"[{time.strftime('%H:%M:%S')}] 发现 {data['person_count']} 人")
        for p in data["persons"]:
            b = p["bbox"]
            print(f"  ID={p['track_id']}  conf={p['confidence']:.2f}"
                  f"  ({b['x1']},{b['y1']})-({b['x2']},{b['y2']})")


async def run():
    delay = 1.0
    while True:
        try:
            async with websockets.connect(URI) as ws:
                delay = 1.0                  # 连接成功，重置退避
                async for raw in ws:
                    msg = json.loads(raw)
                    if msg.get("type") == "ping":
                        await ws.send(json.dumps({"type": "pong", "ts": msg["ts"]}))
                    else:
                        handle_detection(msg)
        except Exception as e:
            print(f"连接断开，{delay}s 后重连: {e}")
            await asyncio.sleep(delay)
            delay = min(delay * 2, 16.0)


asyncio.run(run())
```

---

## 五、错误码

`code` 为非 0 时表示服务端异常，`data` 字段为 `null`。

| code | WebSocket 关闭码 | 含义 | 处理建议 |
|---|---|---|---|
| `0` | — | 正常 | 处理 `data` |
| `1001` | 1001 | 心跳超时，服务端主动断开 | 触发重连 |
| `1001` | 1001 | 服务初始化中（等待首帧） | 等待后重连 |
| `1002` | — | 检测结果过期（摄像头超 3s 无新帧） | 记录告警，等待恢复 |
| `1003` | — | 服务内部异常 | 记录日志，等待恢复 |
| `1004` | — | 视频源无法打开 | 通知运维检查摄像头 |
| `4001` | 4001 | Token 鉴权失败 | 检查 token 配置 |

---

## 六、服务健康检查

如需确认服务存活状态（不需要 token）：

```
GET http://192.168.1.68:8001/health
```

```json
{
  "status": "ok",
  "source_ok": true,
  "has_result": true,
  "result_age_sec": 0.04
}
```

`result_age_sec` 为检测结果距上次更新的秒数，正常应 < 0.5。
