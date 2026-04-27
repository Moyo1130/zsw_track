# WS Box 实时可视化（前端）

这个页面用于连接机器狗 YOLO WebSocket，实时绘制 `persons[].bbox` 在画面中的位置，并在前端用“居中跟随”算法计算并显示对应指令（`linear_x / angular_z`）。

## 1) 启动方式

浏览器直接打开 `file://` 往往会因为安全限制导致 WebSocket 连接异常。建议用一个本地静态服务器打开 `web/` 目录。

### 方式 A：Python（推荐）

在项目根目录执行：

```powershell
cd D:\Desktop\track
python -m http.server 5173
```

然后浏览器打开：

- `http://127.0.0.1:5173/web/`

### 方式 B：Node（可选）

```powershell
npx serve -l 5173 .
```

然后打开：

- `http://127.0.0.1:5173/web/`

## 2) 使用

1. 在页面上填写 WS 地址，例如：
   - `ws://10.61.248.46:8001/ws/detection`
2. 如需 token，把 token 填在输入框里，页面会自动在 URL 后追加 `?token=...`（若 URL 已带 `token=` 则不重复追加）。
3. 点击“连接”。

## 3) 说明

- **bbox 可视化**：灰色框为全部 persons，绿色框为当前选中的目标。
- **指令展示**：页面会显示计算得到的 `linear_x / angular_z`，以及 `offset / fill_ratio` 等中间量。
- **参数调节**：右侧可实时调 `k_ang / ang_max / desired_fill / k_lin / lin_max / smooth_alpha`，用于现场找手感。

