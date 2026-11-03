# HerKules 视觉组大作业：装甲板检测与位姿估计

[中文](README.md) · [English](README.en.md)

已实现**路线 A：检测**。使用官方 ONNX 模型和相机标定，在 CPU 上检测视频中的装甲板，
恢复四角点的相机坐标，并为每帧导出一条 JSONL。作业只需选择一条路线，`aiming/` 保留模板原样。

- [完整运行说明、算法和参数](detection/README.md)
- [原始检测题目](detection/ASSIGNMENT.md)
- [实现](detection/detect.py) · [测试](detection/tests/test_detection.py) · [输出校验器](detection/validate.py)
- [small.jsonl](detection/results/small.jsonl) · [large.jsonl](detection/results/large.jsonl) · [验证报告](detection/results/validation.json)

## 快速复现

在仓库根目录运行，推荐 Python 3.12（本次验证版本为 3.12.3）：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r detection/requirements.txt

for clip in small large; do
  .venv/bin/python detection/detect.py \
    --model detection/assets/0526.onnx \
    --video "detection/assets/$clip.mp4" \
    --calibration detection/assets/calibration.yaml \
    --output "detection/results/$clip.jsonl" \
    --annotated-video "detection/results/${clip}_annotated.mp4" \
    --preview-dir "detection/results/${clip}_previews"
done

.venv/bin/python -m unittest discover -s detection/tests -v
.venv/bin/python detection/validate.py \
  detection/results/small.jsonl detection/results/large.jsonl \
  --calibration detection/assets/calibration.yaml
```

输入模型、两个视频和标定文件已随模板保存在 `detection/assets/`，不需要训练、GPU 或 ROS。
程序使用 `CPUExecutionProvider`；即使 ONNX Runtime 在启动时提示无法发现 GPU，也不影响 CPU 推理。
MP4 由 OpenCV 编码，不需要单独安装 ffmpeg。标注视频默认不纳入 Git，可用上述命令重新生成。

## 验证结果

默认参数：objectness 阈值 0.5、NMS IoU 0.5、最大重投影 RMS 5 像素。

| 输入视频 | 输出帧数 | 检测数 | 有效位姿 | 位姿为 null | 重投影 RMS 中位数 / P95 |
| --- | ---: | ---: | ---: | ---: | --- |
| small.mp4 | 360 | 396 | 396 | 0 | 0.119 / 0.302 px |
| large.mp4 | 360 | 151 | 148 | 3 | 0.275 / 1.822 px |

13 项单元测试全部通过；两份 JSONL 的 720 帧全部通过独立检查，包含帧序、时间戳、类别、
有限数值、正深度、物理尺寸、矩形几何和重投影误差。没有检测的帧也保留记录。
这验证了实现与输出的一致性；数据不提供逐帧真值，不能据此声称检测准确率或真实三维精度。

![小装甲板检测示例](detection/results/small_previews/frame_0000.png)

## 来源与提交

- [任务页面](https://training.herkules.dev/learn/vision-assignment)
- [官方模板](https://github.com/HKUAstar/vision-training-assignment)，基于提交 `c9fa5e9`。
- [模型及参考实现](https://github.com/broalantaps/RobotDetectionModel)，具体说明见检测目录。

作业使用了 AI 辅助实现与验证，算法流程、参数和失败处理均在文档中说明。
正式提交时，按任务页面要求将本仓库发布到自己的 **Public GitHub 仓库**，再在培训网站填写该仓库 URL。
