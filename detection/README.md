# 路线 A：实现与复现说明

本目录实现了 [原始题目](ASSIGNMENT.md) 要求的检测流程。Python 3.12 + ONNX Runtime CPU + OpenCV，
不使用 Ultralytics 等现成检测流水线，不训练模型，不读取视频文件名决定检测结果。

## 安装和运行

以下命令均在**仓库根目录**执行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r detection/requirements.txt

.venv/bin/python detection/detect.py \
  --model detection/assets/0526.onnx \
  --video detection/assets/small.mp4 \
  --calibration detection/assets/calibration.yaml \
  --output detection/results/small.jsonl \
  --annotated-video detection/results/small_annotated.mp4 \
  --preview-dir detection/results/small_previews

.venv/bin/python detection/detect.py \
  --model detection/assets/0526.onnx \
  --video detection/assets/large.mp4 \
  --calibration detection/assets/calibration.yaml \
  --output detection/results/large.jsonl \
  --annotated-video detection/results/large_annotated.mp4 \
  --preview-dir detection/results/large_previews
```

如果在 `detection/` 目录中运行，可改为 `../.venv/bin/python detect.py --model assets/0526.onnx ...`。
虚拟环境仅安装本项目依赖；不修改系统 Python。若当前终端通过 `PYTHONPATH` 注入了 ROS 等其他 Python 包，
可以在安装、验证命令前添加 `env -u PYTHONPATH`，避免其依赖影响项目环境。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--model`、`--video`、`--calibration`、`--output` | 必填 | 模型、输入视频、标定和输出 JSONL 路径 |
| `--confidence` | 0.5 | sigmoid(objectness) 阈值 |
| `--nms-iou` | 0.5 | 四角点包围矩形的 NMS IoU 阈值 |
| `--max-reprojection-error` | 5.0 | 原图坐标下允许的最大 RMS，单位 px |
| `--threads` | 2 | ONNX Runtime CPU 运算线程数 |
| `--max-frames` | 不限 | 仅用于快速检查，例如 `--max-frames 3`；正式运行应省略 |
| `--annotated-video` | 不生成 | 保存标注 MP4，不改变 JSONL 内容 |
| `--preview-dir` / `--preview-every` | 不保存 / 90 | 按帧间隔保存 PNG 抽样图 |

输出目录自动创建，同名输出会覆盖。日志写到 stderr；完成后的摘要写到 stdout。
MP4 使用 OpenCV 的 `mp4v` 编码。无需显示服务器，也无需单独安装 ffmpeg。
输入分辨率必须与标定文件一致，否则报错，避免静默使用错误内参。

## 算法说明

1. **预处理**：保持宽高比缩放到 640×640 范围，居中用 114 补边。
   本题 1440×1080 → 640×480，上下各补 80 像素。保存真实缩放尺寸与补边，支持奇数尺寸的舍入。
   BGR 转 RGB，除以 255，转换成连续的 `[1,3,640,640]` float16 张量；模型接口也会检查。
2. **解码**：输出 `[1,25200,22]` 中前 8 个值直接作为四角点坐标；第 8 列用稳定 sigmoid，
   颜色 `9:13`、图案 `13:22` 分别 argmax。置信度不乘类别分数，不过滤友方、neutral 或 purple。
3. **NMS**：由四角点 min/max 得到浮点轴对齐矩形，按 objectness 降序抑制 IoU > 0.5 的重复项。
   使用类别无关 NMS，避免同一块板因类别波动重复输出。0.5/0.5 沿用题目建议，未针对视频帧调参。
4. **原图角点**：减补边、除真实缩放比例。始终保留 UL、LL、LR、UR 顺序；不按图像 x/y 排序，
   也不裁剪图像边缘外的预测点，避免人为破坏投影几何。
5. **大小选择**：仅 `pattern="1"` 和 `"base_big"` 用 0.230×0.056 m；其余用 0.135×0.056 m。
   选择与文件名、颜色、帧号无关。
6. **PnP**：将四个原图像素点、原始 K 和畸变参数传给 `solvePnPGeneric(..., SOLVEPNP_IPPE)`。
   对各候选进行 LM 重投影优化，同时保留原候选。排除非有限数、非正深度和 RMS 超过 5 px 的解，
   选择剩余误差最小者。退化/不凸四边形或求解失败返回空位姿。
7. **输出**：每个物理角点做 `P_camera = R @ P_object + t`，并计算
   `sqrt(sum_i(||projected_i - observed_i||²) / 4)`。输出单位为米，+X 右、+Y 下、+Z 前。
   5 px 门限是保守的几何一致性检查，可调整；不是检测真值判定。

颜色和图案映射严格使用题目表格，而非上游 README 的颜色描述：

```text
color:   blue, red, neutral, purple
pattern: sentry, 1, 2, 3, 4, 5, outpost, base_small, base_big
```

## 输出与故障处理

`frame_index` 从 0 连续增长。对官方 30 FPS 视频，`timestamp_s = frame_index / 30`；
对其他固定帧率视频，使用视频声明的 FPS。每次运行时间从零开始。
没有检测时仍写一行 `detections: []`。PnP 失败保留颜色、图案、置信度和二维角点，
将 `corners_camera_m` 与 `reprojection_error_px` 同时写成 `null`。
JSON 序列化禁止 NaN/Infinity；非有限的模型输出行被忽略。
已知帧数的输入如果提前解码终止会报错，不会报告为完整成功。

标注图中的 0、1、2、3 对应 UL、LL、LR、UR。彩色实点为检测角点，绿色圆环为三维角点重投影，
`#编号` 对应左上角说明中的颜色、图案、objectness、平均深度和 RMS。

## 验证

```bash
.venv/bin/python -m unittest discover -s detection/tests -v
.venv/bin/python detection/validate.py \
  detection/results/small.jsonl detection/results/large.jsonl \
  --calibration detection/assets/calibration.yaml \
  > detection/results/validation.json
```

13 项确定性单元测试覆盖预处理通道和数据类型、奇数尺寸逆变换、全部类别、objectness/NMS、
无检测/非有限模型值、旋转后的角点顺序、带畸变的大小板三维恢复、米制物理边长、RMS 公式、
退化四边形、PnP 异常、相机后方解和不合法标定。已知位姿测试固定随机种子，包含 24 个不同姿态。

独立校验器逐行检查完整 schema、帧序、时间戳、数值有限性和空值配对；从导出的三维角点重新检查
边长/正交性/正深度，并直接重新投影到原图以核对 RMS，不依赖求解器返回的误差值。

| 指标 | small | large |
| --- | ---: | ---: |
| 完整输出帧数 | 360 | 360 |
| 有检测帧数 | 198 | 101 |
| 无检测帧数 | 162 | 259 |
| 检测数 | 396 | 151 |
| 有效位姿 / 空位姿 | 396 / 0 | 148 / 3 |
| RMS 中位数 | 0.118885 px | 0.274623 px |
| RMS P95 | 0.302033 px | 1.821927 px |
| RMS 最大值 | 0.539286 px | 3.503211 px |

全部 720 条记录通过验证。详细类别计数、深度范围见 [validation.json](results/validation.json)。
`large` 的第 123、137、138 帧各有一个检测超过 5 px 位姿门限；其部分预测角点超出图像边界。
保留这些检测的二维结果，按题目约定输出空位姿，不伪造三维点。

## 局限与来源

- 数据没有逐帧检测或三维真值。低重投影误差表明几何拟合一致，不能证明检测类别或实际距离正确。
- 固定预训练模型可能在低照度、遮挡、出画、运动模糊时漏检/误分类；大小判断也会受分类影响。
- 没有跨帧跟踪。平面双解即使按误差择优，仍可能在接近正视时产生姿态抖动；题目不要求时序消歧。
- 本次只使用每帧相机坐标系，不输出世界坐标，不连接瞄准/发射控制。

参考 [RobotDetectionModel 的 OpenVINO 实现](https://github.com/broalantaps/RobotDetectionModel/blob/babaebd6c8b3aeceda0ca924772b2d70d51801b4/OpenvinoInfer.cpp)
理解张量和后处理；按本题要求重新实现预处理、解码、NMS、PnP 和输出，未照搬其颜色过滤或 NMS 分数处理。
PnP 与坐标约定参考 [OpenCV 官方文档](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)。
输入模型/视频/标定保留自 [官方作业模板](https://github.com/HKUAstar/vision-training-assignment)。
