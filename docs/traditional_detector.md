# 传统装甲板检测与空隙误配排除

`src/vision/traditional_detector.*` 接收处理后的 8 位 BGR 图像，输出灯条及装甲板候选。中央图案检查默认开启，适用于本地录像中带亮色数字的装甲板；其依据是实际图像内容，不是新增的赛务规则或已确认的所有靶板规格。

## 检测流程

1. 对每个像素取 BGR 通道最大值，按亮度阈值生成 mask，提取外轮廓。
2. 用最小外接旋转矩形构造灯条，检查长度、长宽比、倾角与轮廓填充率。填充率为连续轮廓面积除以旋转矩形面积，默认下限 `0.35`；该定义与按填充像素数计数不同，不能直接照搬参考项目的阈值。
3. 在轮廓内部填充 mask 后统计 BGR 均值，以蓝红通道差分类；白色边缘不会单独决定整根灯条的颜色。颜色差不足时仍归为 unknown 并排除。
4. 按 x 坐标排序，枚举同色灯条对，沿用中心距/灯长、两侧长度比和矩形方向误差检查。
5. 排除候选四边形内的第三根灯条。检查已通过灯条筛选的红蓝灯条的顶点、中点、底点，包括边界；按 ID 跳过候选本身。使用实际四边形，避免轴对齐包围框误排倾斜目标。
6. 默认进一步检查中央亮图案。相邻装甲板之间的内侧灯条也可能通过几何检查，而且中间没有第三根灯条，必须增加内容证据才能排除这种空隙候选。

中央图案检查将四点透视映射至 `64 × 56` 图像，灯条端点位于 y=16/40，向上下扩展以容纳高于灯条的数字；裁出中央 38 列以避开发光灯条。灰度化和 `3 × 3` 模糊后，在灯条高度对应的中央 24 行估计对比度和 Otsu 阈值，避免上下扩展区的亮背景淹没中央暗数字。该区域最大最小灰度差须至少为 `6`；Otsu 阈值随后应用于完整高度，保留数字上下笔画及边界检查依据。对比度门限只拒绝低幅变化，不作为固定二值化下限，以保留亮度不均的数字。

至少一块连通亮区域须满足：宽至少 2 像素、高至少为矫正灯长的一半，面积占裁图 2%–60%，不接触上下边界，横向质心在中央 25%–75%、纵向质心在高度 35%–65%，并跨过高度中线。这些条件排除细小亮点、满幅背景和仅出现在候选上方的纹理。

它只验证中央有合适的亮图案，不识别数字身份，不输出分类置信度。其他具有相似亮纹理的背景仍可能通过；无标记、暗色标记、严重遮挡、低对比度、模糊或极端透视也可能造成漏检。当亮背景与数字笔画连为同一块、触及矫正图边界时，目前仍会拒绝该候选；本轮录像中确有这类真实目标漏检，不能声称已经覆盖所有目标。

## 配置与观察

程序默认与根目录 `run.conf` 均开启检查：

```ini
detector.require_center_pattern=true
```

可用 `--require-center-pattern` / `--no-center-pattern` 临时覆盖；配置文件先应用，命令行后应用，同类命令行开关最后一个生效。关闭后仍保留颜色、填充率、几何及第三灯条检查，但失去中央内容证据，可能重新出现空隙误配。其余新增门限目前是 `TraditionalDetectorConfig` 的 C++ 参数，不是应用配置键。

在笔记本上观察已有录像：

```bash
bash run.sh video \
  --source calibration/nuc-recordings-2026-10-14/chessboard-02.avi \
  --dry-run --show-debug --start-paused
# 无中央亮标记时，显式关闭该检查进行对照：
bash run.sh video \
  --source calibration/nuc-recordings-2026-10-14/chessboard-02.avi \
  --dry-run --show-debug --start-paused --no-center-pattern
```

窗口依赖本机桌面和 HighGUI；空格暂停/继续，n 单步，q/Esc 退出。无桌面时使用 `--debug-dir`、`--debug-every`、CSV 和 summary，见 [应用说明](application.md)。同帧应对照原图确认框中是否确有目标；总候选数量不能当作精确率或召回率。没有真实内外参时，回放只用于检测诊断。

## 与解算层的接口

`ArmorObservation::points` 继续使用左上、右上、右下、左下，即 `TL, TR, BR, BL`。新增透视变换适配这一顺序；参考项目数字提取的输入顺序不同。输出灯条端点、角点、ROI 和后续 PnP 接口沿用现有定义，图案矫正图不作为 PnP 输入。

## 参考实现与许可

本轮基于以下本地源码审查和适配：

- [chenjunnn/rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim)，本地 `chenjun_rm_auto_aim-main/`：`armor_detector/src/detector.cpp` 的轮廓内颜色统计、`containLight`，以及 `src/number_classifier.cpp` 的 `extractNumbers` 透视矫正、上下扩展、中央裁切与 Otsu 流程。
- [FaterYU/rm_auto_aim](https://github.com/FaterYU/rm_auto_aim)，本地 `FaterYU_rm_auto_aim-main/`：`armor_detector/src/detector.cpp` 的填充率、轮廓内颜色统计及 `containLight`，以及数字提取流程。

本项目适配为 BGR、连续轮廓面积、实际候选四边形及现有角点顺序，并以连通域检查中央内容。两个参考项目还使用数字分类模型及负样本过滤；本轮未迁移 ROS2、模型推理、大小装甲板物理模型或参考通信格式。现有 `sp_vision_25-main` 检测几何复用关系见 [架构说明](architecture.md#传统检测层第四轮基础与第二十二轮改进)。

以下保留本轮参考适配部分的上游版权和 MIT 许可；其他既有模块按各自原有来源与许可管理。

```text
MIT License

Copyright (c) 2022 ChenJun
Copyright (C) 2024 Zheng Yu

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
