# 传统装甲板检测与空隙误配排除

`src/vision/traditional_detector.*` 接收处理后的 8 位 BGR 图像，输出灯条及装甲板候选。第二十四轮的应用默认启用 ONNX 内容分类，用数字/negative 证据排除空隙误配，并解决共享灯条冲突；同帧多个独立装甲可同时保留。库级 `NumberClassifierConfig` 默认关闭分类器，便于几何测试和显式基线对照。旧中央亮图案检查只在分类器关闭时使用。

## 检测流程

1. 对每个像素取 BGR 通道最大值，按亮度阈值生成 mask，提取外轮廓。
2. 用最小外接旋转矩形构造灯条，检查长度、长宽比、倾角与轮廓填充率。填充率为连续轮廓面积除以旋转矩形面积，默认下限 `0.35`；该定义与按填充像素数计数不同，不能直接照搬参考项目的阈值。
3. 在轮廓内部填充 mask 后统计 BGR 均值，以蓝红通道差分类；白色边缘不会单独决定整根灯条的颜色。颜色差不足时仍归为 unknown 并排除。
4. 按 x 坐标排序，枚举同色灯条对，沿用中心距/灯长、两侧长度比和矩形方向误差检查。
5. 排除候选四边形内的第三根灯条。检查已通过灯条筛选的红蓝灯条的顶点、中点、底点，包括边界；按 ID 跳过候选本身。使用实际四边形，避免轴对齐包围框误排倾斜目标。
6. 应用默认将每个几何候选交给内容分类器，记录类别、softmax 置信度和拒绝原因，过滤 `negative` 及低置信度候选。分类器开启时不先经过旧中央亮图案门限。
7. 对共享任一灯条的候选建立冲突连通组，选择总分类置信度最高、灯条不重复使用的配对集合。最优与次优得分差不超过 `conflict_margin` 时整组拒绝；平分时即使 margin 为0也拒绝。超过18个候选的冲突组直接按歧义拒绝，限制组合搜索规模。独立组互不影响。

相邻装甲板之间的内侧灯条也可能通过几何检查，而且中间没有第三根灯条，因此仍需内容证据。分类器拒绝记录保留在 `DetectionResult::rejected_armors`：`negative`、`low_confidence`、`shared_light` 或 `ambiguous_shared_light`；`armors` 仅包含最终接受项。

## 内容分类与旧图案检查

`src/vision/number_classifier.*` 复用参考项目的数字 ROI 和 ONNX softmax 流程，通过 OpenCV DNN 的 CPU 后端推理。四点映射至高28像素的矫正图，灯条端点位于 y=7/19，按几何比例选择32或54像素宽，再取中央20×28区域。为保持上游模型预处理约定，沿用其 `RGB2GRAY` 转换，然后进行 Otsu 二值化并缩放为 float32 的0..1输入。这里的宽度分支仅用于提取图像，不决定物理装甲板尺寸。

默认模型、标签和 MIT 许可位于 `models/armor_number/`。标签必须恰含一个 `negative`，网络输出数量须与标签数一致。类别只作内容证据，不作为物理装甲唯一身份、大小装甲规则或优先级。模型置信度也不是现场准确率。

分类器关闭且 `require_center_pattern=true` 时，沿用第二十二轮的中央亮图案检查：

中央图案检查将四点透视映射至 `64 × 56` 图像，灯条端点位于 y=16/40，向上下扩展以容纳高于灯条的数字；裁出中央 38 列以避开发光灯条。灰度化和 `3 × 3` 模糊后，在灯条高度对应的中央 24 行估计对比度和 Otsu 阈值，避免上下扩展区的亮背景淹没中央暗数字。该区域最大最小灰度差须至少为 `6`；Otsu 阈值随后应用于完整高度，保留数字上下笔画及边界检查依据。对比度门限只拒绝低幅变化，不作为固定二值化下限，以保留亮度不均的数字。

至少一块连通亮区域须满足：宽至少 2 像素、高至少为矫正灯长的一半，面积占裁图 2%–60%，不接触上下边界，横向质心在中央 25%–75%、纵向质心在高度 35%–65%，并跨过高度中线。这些条件排除细小亮点、满幅背景和仅出现在候选上方的纹理。

旧图案检查只验证中央有合适的亮图案，不输出类别或分类置信度。其他相似亮纹理仍可能通过；无标记、暗色标记、严重遮挡、低对比度、模糊或极端透视也可能造成漏检。亮背景与数字笔画连为一块并触及矫正图边界时仍会拒绝候选，历史录像调查中确有这类漏检。

## 配置与观察

应用默认使用以下分类设置；模型和标签默认取构建时指定的项目资源目录，部署到其他路径时显式配置：

```ini
detector.classifier.enabled=true
detector.classifier.model=models/armor_number/mlp.onnx
detector.classifier.labels=models/armor_number/label.txt
detector.classifier.min_confidence=0.7
detector.classifier.conflict_margin=0.05
detector.require_center_pattern=true
```

`--classifier` / `--no-classifier`、`--classifier-model`、`--classifier-labels` 和 `--classifier-confidence` 可临时覆盖。配置先应用、命令行后应用，同类开关最后一个生效。`--require-center-pattern` / `--no-center-pattern` 只控制分类器关闭后的旧检查；关闭两种内容检查时仍保留颜色、填充率、几何及第三灯条检查，但可能重新出现空隙误配，也不执行基于分类证据的共享灯条消歧。其余灯条/图案门限是 `TraditionalDetectorConfig` 的 C++ 参数。

在笔记本上观察已有录像：

```bash
bash run.sh video \
  --source calibration/nuc-recordings-2026-10-14/chessboard-02.avi \
  --dry-run --show-debug --start-paused
# 对照关闭所有内容检查的几何基线：
bash run.sh video \
  --source calibration/nuc-recordings-2026-10-14/chessboard-02.avi \
  --dry-run --show-debug --start-paused --no-classifier --no-center-pattern
```

窗口依赖本机桌面和 HighGUI；空格暂停/继续，n 单步，q/Esc 退出。无桌面时使用 `--debug-dir`、`--debug-every`、CSV 和 summary，见 [应用说明](application.md)。同帧应对照原图确认框中是否确有目标；总候选数量不能当作精确率或召回率。没有真实内外参时，回放只用于检测诊断。

## 第二十四轮离线结果与边界

对 `calibration/nuc-recordings-2026-10-15/chessboard-05.avi` 的指定五帧，使用应用同一 C++ 检测器和默认模型验证如下。帧号从0开始；每帧恰接受一个类别 `2` 的候选，原有两个空隙候选均被分类为 `negative` 并拒绝。

| 帧号 | 接受类别 | 接受置信度 | 拒绝空隙数 |
|---|---|---|---|
| 165 | 2 | 0.999960 | 2 |
| 174 | 2 | 0.999781 | 2 |
| 182 | 2 | 0.999873 | 2 |
| 192 | 2 | 0.999959 | 2 |
| 214 | 2 | 0.999949 | 2 |

完整395帧应用 dry-run 无处理错误，累计469个接受候选，75帧有多个候选；这些计数不能证明多目标正确率或全片精度。五帧检测通过也不代表后续 PnP 和旋转关联都接受：其中仅182帧形成当前旋转关联，其余四帧为 `observation_rejected`。该录像没有任何可靠旋转或有效旋转预测帧。

分类结果与标注图见 `build/armor_review_2026_10_15/integrated_classifier.csv`、`frame_*_integrated.png`；完整应用输出见 `round24_app/`、`round24_app.csv` 和 `round24_app_summary.txt`。这些 build 产物未纳入版本管理，清理构建目录前应另行保留。两份本地参考分类模型字节相同，不构成独立模型验证；现场字体、模糊、遮挡和光照适应性仍需人工标注数据评估。

## 与解算层的接口

`ArmorObservation::points` 继续使用左上、右上、右下、左下，即 `TL, TR, BR, BL`。分类和图案透视变换适配这一顺序；参考项目数字提取的输入顺序不同。输出灯条端点、角点、ROI 和后续 PnP 接口沿用现有定义，分类矫正图不作为 PnP 输入。PnP 使用的特征矩形尺寸须对应这些端点，规则板面140×125 mm仅是未确认几何时的诊断默认值。

## 参考实现与许可

相关轮次基于以下本地源码审查和适配：

- [chenjunnn/rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim)，本地 `chenjun_rm_auto_aim-main/`：`armor_detector/src/detector.cpp` 的轮廓内颜色统计、`containLight`，以及 `src/number_classifier.cpp` 的 `extractNumbers` 透视矫正、上下扩展、中央裁切与 Otsu 流程。
- [FaterYU/rm_auto_aim](https://github.com/FaterYU/rm_auto_aim)，本地 `FaterYU_rm_auto_aim-main/`：`armor_detector/src/detector.cpp` 的填充率、轮廓内颜色统计及 `containLight`，以及数字提取流程。

本项目保留 BGR 输入、连续轮廓面积、实际候选四边形和现有角点顺序。第二十二轮适配连通域图案检查；第二十四轮复用 `chenjun_rm_auto_aim-main` 的模型、标签、ROI和推理预处理，并补充共享灯条冲突集合选择。模型来源与校验值见 [资源说明](../models/armor_number/README.md)。未迁移 ROS2、参考大小装甲物理规则或通信格式。现有 `sp_vision_25-main` 检测几何复用关系见 [架构说明](architecture.md#传统检测层第四轮基础与第二十二轮改进)。

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
