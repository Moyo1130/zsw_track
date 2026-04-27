# track_live 运行教程

这份教程用于直接跑 `track_live.py`，实现机器狗对远处人员的左右跟踪和前进跟随。

当前确认使用的参数目标是：

- 左右跟踪保持较稳，不要小抖动就大转向
- 目标远离时前进
- 不做后退
- 目标较远且只略偏左/右时，转向不要过激

## 1) 最终命令

在项目根目录 `D:\Desktop\track` 下执行。

推荐写法：把机器狗 IP 只写在 `--yolo-ws` 里，`--ip` 省略，让脚本自动取同一个主机。

```powershell
python track_live.py --yolo-ws ws://<狗IP>:8001/ws/detection --turn-axis-min 0.30 --turn-axis-max 0.32 --action-s 0.10 --cooldown-s 0.10 --start-norm 0.30 --stop-norm 0.15 --seen-n 4 --desired-fill 0.85 --forward-axis-max 0.35 --forward-gain 2.5 --far-start-boost 0.15
```

如果想写成多行，PowerShell 可用反引号续行：

```powershell
python track_live.py `
  --yolo-ws ws://<狗IP>:8001/ws/detection `
  --turn-axis-min 0.30 `
  --turn-axis-max 0.32 `
  --action-s 0.10 `
  --cooldown-s 0.10 `
  --start-norm 0.30 `
  --stop-norm 0.15 `
  --seen-n 4 `
  --desired-fill 0.85 `
  --forward-axis-max 0.35 `
  --forward-gain 2.5 `
  --far-start-boost 0.15
```

如果你想手动指定机器狗控制 IP，也可以显式加上：

```powershell
python track_live.py --yolo-ws ws://<YOLO主机IP>:8001/ws/detection --ip <机器狗IP> --turn-axis-min 0.30 --turn-axis-max 0.32 --action-s 0.10 --cooldown-s 0.10 --start-norm 0.30 --stop-norm 0.15 --seen-n 4 --desired-fill 0.85 --forward-axis-max 0.35 --forward-gain 2.5 --far-start-boost 0.15
```

适用场景：

- `--yolo-ws` 和机器狗控制 UDP 在同一台设备上：省略 `--ip`
- YOLO 服务和机器狗控制目标不是同一台设备：显式写 `--ip`

## 2) 运行前准备

1. 打开 PowerShell。
2. 进入项目目录：

```powershell
cd D:\Desktop\track
```

1. 确认 Python 可用：

```powershell
python --version
```

1. 安装运行依赖：

```powershell
pip install websockets python-dotenv
```

1. 确认本次机器狗 IP。

如果机器狗 IP 经常变化，现场只需要改 `--yolo-ws` 里的 `<狗IP>`；默认不必再单独改 `--ip`。

1. 准备 `API_TOKEN`。

如果 `--yolo-ws` URL 里没有直接带 `token=...`，脚本会从根目录 `.env` 读取 `API_TOKEN`。

根目录 `.env` 示例：

```dotenv
API_TOKEN=你的token
```

## 3) 建议的实际运行顺序

建议不要一上来就直接打真机，按下面顺序更稳。

### 第一步：先检查映射

```powershell
python track_live.py --check-mapping
```

用途：

- 检查 `linear_x / angular_z` 到 `forward / turn` 的映射是否正常
- 确认前进轴和转向轴都能跨过机器狗固件死区

### 第二步：先 dry-run 看日志

```powershell
python track_live.py --dry-run --yolo-ws ws://<狗IP>:8001/ws/detection --turn-axis-min 0.30 --turn-axis-max 0.32 --action-s 0.10 --cooldown-s 0.10 --start-norm 0.30 --stop-norm 0.15 --seen-n 4 --desired-fill 0.85 --forward-axis-max 0.35 --forward-gain 2.5 --far-start-boost 0.15
```

重点看终端里这些字段：

- `fill=...`
- `linear_x=...`
- `angular_z=...`
- `-> fwd=... turn=...`

你可以这样理解：

- `fill` 越小，目标越远
- `linear_x > 0` 代表控制器认为应该前进
- `fwd > 0` 代表最终真的会给机器狗发前进轴
- `turn != 0` 代表当前会触发一次转向脉冲

### 第三步：真机运行

确认周围空旷、机器狗可安全行走后，再执行最终命令。

```powershell
python track_live.py --yolo-ws ws://<狗IP>:8001/ws/detection --turn-axis-min 0.30 --turn-axis-max 0.32 --action-s 0.10 --cooldown-s 0.10 --start-norm 0.30 --stop-norm 0.15 --seen-n 4 --desired-fill 0.85 --forward-axis-max 0.35 --forward-gain 2.5 --far-start-boost 0.15
```

结束时按 `Ctrl+C`。

脚本退出时会自动走 `safe_stop`，停止连续运动并关闭持续运动通道。

## 4) 这组参数的含义

### 跟随距离

- `--desired-fill 0.85`

含义：当目标框高度占画面高度低于 `0.85` 时，允许前进。

效果：

- 更愿意追近目标
- 目标稍远就会跟上去
- 但仍然不会后退

### 前进力度

- `--forward-gain 2.5`
- `--forward-axis-max 0.35`

含义：

- 把控制器输出的 `linear_x` 放大后映射到前进轴
- 并把前进上限限制在 `0.35`

效果：

- 决定开始前进后，狗会比默认更积极一些
- 但仍然限制在比较可控的速度上限内

### 转向力度

- `--turn-axis-min 0.30`
- `--turn-axis-max 0.32`

含义：

- 一旦触发转向，保证转向轴能跨过死区
- 同时把最大转向幅度压在较小范围内，避免猛转

### 转向触发条件

- `--start-norm 0.30`
- `--stop-norm 0.15`
- `--seen-n 4`

含义：

- 偏差达到 `0.30` 才考虑触发转向
- 偏差回到 `0.15` 以内就认为基本对正
- 连续满足 4 帧才真正发一次转向动作

效果：

- 能过滤掉 YOLO 抖动
- 不会轻微偏一点就马上左右乱摆

### 转向动作节奏

- `--action-s 0.10`
- `--cooldown-s 0.10`

含义：

- 每次转向动作持续 `0.10s`
- 动作结束后冷却 `0.10s`

效果：

- 转向是短脉冲
- 前进仍然可以持续
- 整体比“持续打大转向”更稳

### 远距离抑制过度转向

- `--far-start-boost 0.15`

含义：

- 目标越远，实际使用的 `start_norm` 会自动变大
- 目标越近，又会自动回到你设定的 `0.30`

效果：

- 人在远处时，稍微偏左/右不会立刻触发大转向
- 更像“先稳住前进，再逐步修正方向”

## 5) 实际运动逻辑

如果目标在远处左侧，当前脚本的行为通常是：

1. 先根据远近决定是否前进
2. 目标足够偏左并连续满足阈值后，打一小段左转脉冲
3. 转向冷却期间继续前进
4. 如果仍偏左，再打一小段左转

所以它更接近：

- 以前进为底
- 间歇叠加转向修正

不是“先原地转完再走”。

## 6) 常见现象与调参

### 现象 A：走得很远才跟上去

优先调大：

- `--desired-fill`

例如：

- `0.85 -> 0.90`

### 现象 B：已经决定跟了，但前进还是偏慢

优先调大：

- `--forward-axis-max`
- `--forward-gain`

例如：

- `--forward-axis-max 0.38`
- `--forward-gain 2.8`

### 现象 C：远处稍微偏一点就转得明显

优先调大：

- `--far-start-boost`
- 或 `--start-norm`

例如：

- `--far-start-boost 0.20`
- 或 `--start-norm 0.33`

### 现象 D：转向太慢，不容易把目标拉回中心

优先调：

- `--turn-axis-max`
- 或减小 `--seen-n`

例如：

- `--turn-axis-max 0.34`
- `--seen-n 3`

## 7) 安全建议

1. 先 `--dry-run`，再上真机。
2. 先在空旷区域测试，尤其不要正前方有人贴得太近时直接运行。
3. 第一次真机建议旁边有人随时准备接管。
4. 如果发现动作异常，直接按 `Ctrl+C`。

