# 项目交接文档

更新时间：2026-10-04
项目路径：`/home/wuming/Projects/shu_auto_aim_project`

## 1. 项目目标

本项目目标是完成上海大学 SRM 战队算法校内赛**自瞄赛道**上位机程序，形成：

```text
相机输入
→ 装甲板检测
→ PnP/坐标解算
→ 目标选择
→ 跟踪与预测
→ 弹道/延迟补偿
→ 安全决策
→ USB CDC 通信
```

正式规则和通信协议优先级：

1. 最新赛务通知；
2. `shu_rules/rules.md`；
3. `shu_rules/self_aim_protocol.md`；
4. 当前项目 `docs/`；
5. `sp_vision_25-main/`。

参考项目只能用于通用算法、工程思路和可兼容工具复用。其旧版 `SP` 帧头、CRC16、四元数通信结构和旧字段不能迁移到当前正式通信层。

## 2. 当前工作区重要说明

用户最新要求是：**只新增/修改 `AGENT_HANDOFF.md`，不要修改其他文件。**

本交接文件由当前 agent 创建；本轮没有修改任何其他项目文件。

工作区在本轮之前已有未提交变更，主要来自前面阶段开发。不要误以为这些变更都是本轮产生的，也不要擅自清理或回滚。

## 3. 已完成的软件工作

### 3.1 工程和协议

已完成：

- CMake/C++17 工程基础；
- 正式自瞄协议编解码；
- 小端字段处理；
- 上位机发送固定 18 字节帧：
  - body length 16；
  - ID1 yaw/pitch；
  - ID2 `fire_flag`；
- 下位机接收固定 30 字节帧：
  - body length 28；
  - ID1 云台 yaw/pitch/roll/mode/color；
  - ID2 `bullet_speed`；
- 流式解析：支持半帧、整帧、多帧、任意拆包；
- 非法长度、未知 ID、记录长度错误和顺序错误处理；
- 没有引入参考项目旧协议的 `SP`、CRC、时间戳、序列号。

核心文件：

```text
src/protocol/self_aim_protocol.hpp
src/protocol/self_aim_protocol.cpp
```

### 3.2 USB CDC/串口传输

已完成：

- `SelfAimTransport`；
- 后台接收线程；
- 后台周期发送线程；
- 设备不存在时重连；
- 设备断开处理；
- 最近一次下位机状态保存；
- 最近状态帧时间戳；
- transport 收发计数和通信错误计数；
- 退出前发送安全帧；
- 默认命令 `fire_flag=0`；
- Linux 伪终端测试。

核心文件：

```text
src/io/self_aim_transport.hpp
src/io/self_aim_transport.cpp
src/third_party/serial/
```

注意：当前 transport 生命周期仍是一次 `start()`/`stop()`，停止后同一实例不能重新启动。设备断开期间支持后台重连。

### 3.3 输入抽象

已完成：

- 单张图片输入；
- 视频文件输入；
- OpenCV camera index 输入；
- 单调时间戳；
- 帧序号；
- 文件源时间；
- EOF 处理；
- 图片/视频循环读取；
- reset；
- 离线 smoke test。

核心文件：

```text
src/io/frame_source.hpp
src/io/frame_source.cpp
```

当前未直接接入海康 MVS SDK。

### 3.4 传统视觉检测

已完成基础传统检测器：

```text
BGR 图像
→ 亮度阈值
→ 灯条轮廓
→ 灯条几何筛选
→ 红/蓝颜色分类
→ 灯条配对
→ 装甲板候选
```

主要类型：

```cpp
LightbarObservation
ArmorObservation
DetectionResult
TraditionalDetector
```

位置：

```text
src/vision/traditional_detector.hpp
src/vision/traditional_detector.cpp
```

`ArmorObservation` 和 `LightbarObservation` 都定义在 `traditional_detector.hpp`，不是独立的 `armor.hpp`。

`draw_debug()` 只返回绘制后的图像，不打开 GUI。

当前未实现/未接入：

- YOLO；
- 数字分类；
- 复杂目标类别识别；
- 参考项目完整 `Classifier`/`YOLO` 体系。

### 3.5 PnP 和坐标解算

已完成：

```cpp
ArmorPoseSolver
PoseSolution
PoseSolverConfig
```

位置：

```text
src/vision/pose_solver.hpp
src/vision/pose_solver.cpp
```

`ArmorPoseSolver` 对应参考项目中的 `Solver`，负责：

- 装甲板四点 PnP；
- 相机坐标位置；
- 云台坐标转换；
- 距离；
- yaw/pitch；
- 重投影误差；
- 有效性检查。

默认装甲板尺寸依据当前规则设置为 `140 mm × 125 mm`，不能默认视为现场确认的最终标定数据。

### 3.6 目标选择、跟踪、预测

已完成：

- 每个候选的 PnP 解算；
- 无效候选保留诊断信息；
- 有效候选筛选；
- 基于瞄准角的单帧选择；
- `detecting/tracking/lost` 状态；
- 连续确认和连续丢失计数；
- yaw 角度展开；
- 差分速度估计；
- 短时间预测；
- 预测位置关联；
- 简单遮挡恢复。

文件：

```text
src/vision/target_selector.hpp
src/vision/target_selector.cpp
src/vision/target_tracker.hpp
src/vision/target_tracker.cpp
```

这不是参考项目完整 EKF/MPC/RANSAC 预测系统，而是当前项目的可验证基础实现。

### 3.7 弹道和延迟补偿

已完成基础 `AimCompensator`：

```text
src/control/aim_compensator.hpp
src/control/aim_compensator.cpp
```

实现范围：

- bullet speed 合法性检查；
- 飞行时间；
- 恒重力、无空气阻力基础模型；
- pitch 重力补偿；
- 目标运动前视；
- 额外延迟补偿接口；
- 迭代求解目标运动和飞行时间耦合；
- 输出 `AimSolution`；
- 不直接生成协议帧或开火请求。

### 3.8 阶段六垂直组合

文件：

```text
src/control/auto_aim_pipeline.hpp
src/control/auto_aim_pipeline.cpp
```

组合：

```text
DetectionResult
→ TargetSelector
→ ArmorPoseSolver
→ TargetTracker
→ AimCompensator
→ AutoAimFrameResult
```

不负责相机、串口、安全状态机和 `fire_flag`。

### 3.9 安全控制和运行时主循环

安全状态机：

```text
src/control/safety_controller.hpp
src/control/safety_controller.cpp
```

状态：

```text
safe
 aiming
fire_ready
```

安全门包括：

- transport 是否运行；
- transport 是否连接；
- 下位机状态是否存在/过期；
- 当前帧是否存在/过期；
- 检测处理是否成功；
- 目标是否存在；
- 目标是否 tracking；
- 数学瞄准结果是否有效；
- bullet speed 是否有效；
- 视觉开火是否显式允许。

默认配置：

```cpp
allow_visual_fire = false
```

当前所有默认和异常路径保持：

```text
fire_flag=0
```

运行时主循环：

```text
src/control/auto_aim_runtime.hpp
src/control/auto_aim_runtime.cpp
```

负责读取一帧、取下位机状态、运行检测/算法管线、运行安全决策并写入 transport 命令。`RuntimeStepResult` 还记录单次 step 耗时。

### 3.10 应用入口、日志和统计

应用入口：

```text
src/app/main.cpp
```

已支持：

- 图片/视频/OpenCV camera 输入；
- `--dry-run`；
- `--config PATH`；
- 配置文件先加载，命令行覆盖；
- `--log PATH`；
- `--debug-dir PATH`；
- `--debug-every N`；
- `--summary PATH`；
- `--max-frames`；
- `--loop/--no-loop`；
- transport 参数；
- 内参输入；
- SIGINT/SIGTERM；
- 退出前安全帧。

参数解析：

```text
src/app/application_options.hpp
src/app/application_options.cpp
```

CSV 日志：

```text
src/app/runtime_csv_logger.hpp
src/app/runtime_csv_logger.cpp
```

运行统计：

```text
src/app/runtime_statistics.hpp
src/app/runtime_statistics.cpp
```

统计内容包括：

- step/帧/EOF/错误；
- 候选数量；
- 安全状态分布；
- 非零开火请求次数；
- 总/平均/最大耗时；
- transport 计数。

## 4. 测试和当前编译状态

最近一次完整验证命令：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/srm_auto_aim_app --help
git diff --check
```

最近一次结果：

```text
13/13 tests passed
100% tests passed
```

CTest 当前包含：

```text
srm_protocol_test
srm_transport_test
srm_frame_source_test
srm_traditional_detector_test
srm_pose_solver_test
srm_target_selector_test
srm_target_tracker_test
srm_aim_compensator_test
srm_auto_aim_pipeline_test
srm_safety_controller_test
srm_auto_aim_runtime_test
srm_application_support_test
srm_application_app_smoke_test
```

应用 smoke test 使用：

```text
tests/application_smoke.cmake
tests/data/application_smoke.ppm
```

自动检查 debug 图像、CSV、summary 和 `fire_flag=0`。

最近 smoke 输出示例：

```text
processed_frames=1
log_rows=1
dry_run=1
error_count=0
nonzero_fire_request_count=0
```

## 5. 已修改/新增文件清单

### 当前工作区已有修改文件

```text
AGENTS.md
CMakeLists.txt
docs/application.md
docs/architecture.md
docs/development_plan.md
docs/development_workflow.md
src/app/application_options.cpp
src/app/application_options.hpp
src/app/main.cpp
src/app/runtime_csv_logger.cpp
src/app/runtime_csv_logger.hpp
src/control/auto_aim_runtime.cpp
src/control/auto_aim_runtime.hpp
tests/application_support_test.cpp
```

### 当前工作区新增但尚未提交文件

```text
AGENT_HANDOFF.md
dev_log/2026-10-04-round-14.md
dev_log/2026-10-04-round-15.md
docs/deployment.md
src/app/runtime_statistics.cpp
src/app/runtime_statistics.hpp
tests/application_smoke.cmake
tests/data/application_smoke.ppm
```

注意：`AGENT_HANDOFF.md` 是本轮新增；其它新增文件来自前面阶段。

### 主要已有提交

```text
f24d5f7 第十二轮修改
b496b6d 第十一轮开发
f7505f4 第十轮开发，完成第七阶段
791b097 第九轮开发，完成第六阶段
```

工作区存在未提交修改，继续开发前应先查看：

```bash
git status --short
git diff --stat
git diff
```

## 6. 未解决问题和限制

### 6.1 现场硬件未验证

尚未验证：

- 真实 x86_64 NUC 环境；
- Ubuntu/OpenCV/MVS 依赖；
- 海康 CS016 实际型号、镜头和分辨率；
- MVS SDK 采集；
- `/dev/ttyACM*` 实际设备名和权限；
- USB CDC 真实收发；
- 下位机状态字段真实语义和范围；
- 云台 yaw/pitch 方向和零位；
- 实际弹速；
- 相机内外参；
- 机械偏置和延迟。

### 6.2 视觉能力仍是基础版

当前传统检测没有：

- YOLO；
- 数字分类；
- 完整目标身份识别；
- 复杂多目标管理；
- 参考项目完整 `Aimer`/`Shooter` 逻辑；
- 参考项目 EKF/MPC/RANSAC 等高级预测完整迁移。

需要用现场图片/视频确认检测效果后，再决定是否迁移参考项目 YOLO/Classifier 或增强跟踪器。不能在没有现场数据时擅自锁定路线。

### 6.3 配置能力限制

当前命令行和简单 `key=value` 配置主要覆盖：

- 输入；
- transport；
- 基础内参；
- 日志；
- debug；
- summary；
- dry-run。

尚未完成完整配置文件加载：

- 畸变参数；
- 相机到云台外参；
- 完整检测阈值；
- 跟踪参数；
- 弹道参数；
- 延迟参数。

### 6.4 发射安全限制

- 当前应用入口默认不产生非零 `fire_flag`；
- 尚未进行真实发射测试；
- 不得把离线 smoke test 当作命中率或现场效果证明；
- 真实发射前必须完成通信、相机、标定、静态瞄准、目标丢失和断链保护测试，并取得现场安全员许可。

### 6.5 生命周期和统计细节

- `SelfAimTransport` 当前停止后不能重新启动同一个实例；
- transport 各统计计数是后台线程并发读取，不保证同一事务时刻的一致快照；
- MVS 专用输入仍未实现；
- `fire_ready` 和视觉开火策略尚未针对现场下位机行为验证。

## 7. 现场测试统一入口

所有需要真实设备、实车、标定、装弹或发射的事项统一写在：

```text
questions.md
```

该文件包含：

- NUC 环境检查；
- CDC 设备检查；
- 相机检查；
- 标定要求；
- 装甲板现场图像采集；
- 弹速/弹道确认；
- 通信非发射测试；
- 云台非发射测试；
- 相机输入测试；
- 静态瞄准测试；
- 最终受控发射测试；
- 需要反馈的原始日志和参数。

现场操作不是每轮开发的默认前置条件。软件主体开发阶段应优先使用 CTest、伪终端、离线图片/视频和合成数据。

## 8. 下一步计划

### 推荐第一步：现场前离线审查

在接入真实硬件前，建议先：

1. 检查当前工作区状态；
2. 执行完整构建和 CTest；
3. 执行 `srm_auto_aim_app --help`；
4. 执行应用 smoke test；
5. 检查 `docs/deployment.md` 和 `questions.md`；
6. 确认默认运行配置仍保持 `fire_flag=0`。

### 推荐第二步：现场环境确认

按 `questions.md` 先确认：

- NUC 系统和依赖；
- 相机型号、MVS SDK、图像分辨率；
- CDC 设备路径和权限；
- 下位机状态帧实际行为。

### 推荐第三步：根据现场结果选择开发方向

- 如果 OpenCV 能稳定读取工业相机，优先做参数和标定接入；
- 如果 OpenCV 不能读取，开发 MVS `FrameSource` 适配器；
- 如果传统检测对真实图像效果不足，再迁移参考项目 YOLO/Classifier；
- 如果跟踪和预测不足，再引入更复杂滤波/预测；
- 如果静态瞄准存在系统偏差，再补充外参、零位和机械偏置配置；
- 未得到现场安全事实前，不开启非零 `fire_flag`。

### 推荐第四步：现场安全顺序

```text
环境检查
→ 相机检查
→ CDC 非发射通信
→ 无弹云台测试
→ 标定
→ 离线真实图像回放
→ 目标丢失/断链保护
→ 安全员许可下的受控发射
```

## 9. 继续开发时的工作规范

后续 coding agent 必须：

- 先阅读本文件、`AGENTS.md`、相关规则、协议和 `questions.md`；
- 先检查当前项目和 `sp_vision_25-main` 中是否已有可复用实现；
- 规则/正式协议优先于参考项目；
- 不使用参考项目旧通信协议；
- 不把未验证的现场参数写死；
- 默认保持 `fire_flag=0`；
- 每轮只完成一个可验证切片；
- 每次代码修改后运行 CMake、编译、CTest、`--help` 和必要的离线 smoke test；
- 真实硬件和发射测试必须按 `questions.md`，不能用推测代替；
- 每轮完成后更新 `dev_log/` 并汇报真实结果；
- 如果用户要求只修改某个文件，严格限制修改范围。

## 10. 关键命令速查

```bash
cd /home/wuming/Projects/shu_auto_aim_project

git status --short
git diff --stat

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure

./build/srm_auto_aim_app --help

git diff --check
```

离线运行示例：

```bash
./build/srm_auto_aim_app \
  --input image \
  --source tests/data/application_smoke.ppm \
  --dry-run \
  --intrinsics 800,800,32,24 \
  --debug-dir /tmp/srm_debug \
  --debug-every 1 \
  --log /tmp/srm_run.csv \
  --summary /tmp/srm_summary.txt
```

## 11. 交接结论

当前项目不是 `sp_vision_25-main` 的完整类级别移植，而是一个已经通过本地自动化验证的、遵守本次正式协议和安全边界的自瞄基础闭环。

当前状态可概括为：

```text
本地软件基础闭环完成
+ 自动化测试通过
+ 离线运行入口完成
+ 交付文档完成
- MVS/真实硬件未接入
- 现场标定未完成
- 现场视觉效果未验证
- 非零 fire_flag 未验证
```

下一个 coding agent 不应直接声称项目已经达到参考项目的完整自瞄能力。应先根据现场数据决定继续做硬件适配、参数配置、传统视觉增强，还是迁移 YOLO/数字分类和更复杂跟踪预测。
