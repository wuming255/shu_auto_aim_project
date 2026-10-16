# 运行入口、离线回放与运行报告

## 作用

`build/srm_auto_aim_app` 是当前项目的通用运行入口，用于把已有的 `FrameSource`、传统检测器、阶段六算法管线、阶段七安全决策和 CDC 传输层组合起来。

本入口支持 OpenCV 的图片、视频文件和 camera index 输入，以及可选的海康 MVS USB 相机输入。它们共用 `FrameSource` 和后续检测/运行报告接口；MVS 的真实相机采集仍需按 `questions.md` 验证。

日常使用可直接执行 `bash run.sh`，长期参数编辑根目录 `run.conf`；模式、可选参数和窗口操作见 [一条命令启动说明](launcher.md)。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
```

窗口功能使用可选的 OpenCV HighGUI；CMake 会打印 `SRM optional debug windows: enabled` 或 `unavailable`。没有 HighGUI 时仍可构建并执行非 GUI 回放，也可用 `-DSRM_ENABLE_GUI=OFF` 显式关闭窗口支持。

MVS 支持默认尝试启用，只在 Linux 构建找到 SDK 开发头文件时可用；可通过 `-DSRM_MVS_INCLUDE_DIR=/path/to/sdk/include` 指定，或使用 `-DSRM_ENABLE_MVS=OFF` 关闭。运行库只在 MVS 查询/采集时加载，图片、视频和 OpenCV camera 不需要它。头文件来源、4.7 Runtime 限制和运行环境设置见 [deployment.md](deployment.md#海康-mvs-构建与运行环境)。

可执行文件：

```text
build/srm_auto_aim_app
build/srm_calibrate_camera
```

## 输入模式与离线回放

默认输入模式为 OpenCV camera index 0。离线图片和视频示例：

```bash
build/srm_auto_aim_app \
  --input image \
  --source sample.png \
  --dry-run \
  --intrinsics 800,800,320,240 \
  --max-frames 1 \
  --debug-dir replay_debug \
  --summary replay_summary.txt \
  --log replay.csv

build/srm_auto_aim_app \
  --input video \
  --source sample.mp4 \
  --dry-run \
  --intrinsics 800,800,320,240 \
  --debug-dir replay_debug \
  --debug-every 5 \
  --summary replay_summary.txt \
  --log replay.csv
```

`--input image` 默认只处理一帧后进入 EOF；`--input video` 默认读到文件结束。需要重复回放时使用 `--loop`，使用 `--max-frames N` 限制总处理帧数。

常用选项：

- `--input image|video|camera|mvs`：输入类型，`camera` 仍指 OpenCV，`mvs` 指海康 USB SDK；
- `--source PATH`：图片或视频路径；
- `--camera-index N`：OpenCV camera index；
- `--device PATH`：USB CDC 设备，默认 `/dev/ttyACM0`；
- `--calibration PATH`：完整 OpenCV YAML，含 K、畸变、分辨率和方向；
- `--extrinsics PATH`：处理后相机到云台 R/t，要求同时提供完整标定；
- `--intrinsics fx,fy,cx,cy`：旧四参数方式，仅供 dry-run，与完整标定互斥；
- `--camera-rotation DEG`：仅实时相机顺时针旋转 0/90/180/270 度，默认 0；
- `--file-rotation DEG`：仅图片/视频追加旋转，默认 0；
- `--file-orientation DEG`：仅声明旧文件已有像素方向，不再旋转；新录像自动读取 sidecar；
- `--log PATH`：写入 CSV 运行日志；
- `--summary PATH`：写入 key=value 运行摘要；
- `--debug-dir PATH`：创建目录并保存检测、实际关联和旋转状态叠加图；为空时不保存；
- `--debug-every N`：每隔 N 个输入序号保存一张 debug 图，默认 `1`；
- `--classifier` / `--no-classifier`：开启或关闭 ONNX 内容分类，应用默认开启；启用时替代旧中央亮图案门限；
- `--classifier-model PATH` / `--classifier-labels PATH`：模型和标签路径，默认使用项目 `models/armor_number/`；
- `--classifier-confidence P`：非 negative 类的最低置信度，默认 `0.7`，范围 `(0.5,1]`；
- `--require-center-pattern` / `--no-center-pattern`：分类器关闭时启用或关闭旧中央亮图案检查，默认开启；
- `--rotation` / `--no-rotation`：独立四甲旋转模型，应用默认开启；关闭后使用原视线差分跟踪器；
- `--feature-size W,H`：检测灯条端点对应的实测矩形宽、高，单位 m；必须同时提供；
- `--rotation-tilt RAD|unknown`：实测装甲安装倾角，单位 rad，默认未知；
- `--static-camera` / `--no-static-camera`：明确采用固定相机假设，默认关闭，仅允许 dry-run 图片/视频；
- `--show-debug` / `--no-show-debug`：开启或关闭可选窗口，默认关闭，仅支持 dry-run；文件输入可回放交互，相机输入实时刷新；
- `--playback-fps FPS`：窗口播放帧率，`0` 自动选择，非零值范围为 `[0.1, 1000]`；
- `--start-paused` / `--no-start-paused`：视频是否在第一帧暂停，默认不暂停；
- `--record PATH` / `--no-record`：保存原始 BGR 视频或清除配置中的录像路径，默认关闭，仅用于 dry-run；
- `--record-fps FPS`：录像播放元数据帧率，0 使用输入名义帧率或回退 30；不限制采集速度；
- `--record-codec FOURCC`：录像编码，默认 MJPG，可选可用编码器如 FFV1；
- `--max-frames N`：处理 N 帧后退出，0 表示不限制；
- `--loop`：循环图片/视频文件；
- `--dry-run`：不启动串口线程，可用于离线回放或相机采集检查。此模式安全控制器会因 transport 未运行而拒绝有效瞄准命令；
- `--transmit-period-ms N`、`--io-timeout-ms N`、`--reconnect-ms N`：传输层时间参数；
- `--help`：显示完整帮助。

`--debug-dir` 会在启动时创建目录（包括必要的父目录）；图像文件命名为 `frame_000000.png`、`frame_000001.png` 等，编号使用 `Frame.sequence`。应用先绘制检测，再调用 `draw_tracking_debug()` 叠加当前关联和旋转状态；当检测回调成功且序号满足 `debug-every` 时才写出。

未提供标定时，dry-run 仍可用于输入和检测 smoke test，但会明确警告；上面的四参数示例不是现场标定。`--live` 必须提供完整 `--calibration` 和 `--extrinsics`，缺失时在启动 CDC 前拒绝。

应用默认对几何候选提取数字 ROI 并执行 OpenCV DNN CPU 推理，拒绝 `negative` 和低置信度结果，再解决共享灯条冲突；同帧多个不冲突装甲可同时输出。模型类别只提供内容证据，不是物理装甲的唯一身份，也不决定大小装甲或优先级。分类器关闭时才使用旧中央亮图案检查；只传 `--no-center-pattern` 不会关闭分类器。流程、参考设计和许可见 [传统检测说明](traditional_detector.md)。

## 四甲旋转诊断与实际关联

独立旋转模型使用本帧全部有效 PnP 观测和 IPPE 候选姿态，拟合四甲的中心、半径、连续相位与角速度。原 tracker 的视线角差分速度不作为旋转角速度。半径/相位残差、可观测法向跨度、更新数和采样间隔共同限制预测；拒绝的观测不会显示为当前关联。短失预测只允许继承此前可靠的拟合，长失、时间轴重置或参考系有效性变化会清除历史。新观测与上次有效观测的间隔满足 `max_angular_speed_rad_s × 间隔 >= pi/4` 时，四甲相位无法唯一展开，须重新收敛。

可靠预测要求显式特征尺寸、安装倾角和有效固定参考系。`140 × 125 mm` 是规则中的板面尺寸，默认只用于未确认几何的诊断，不能冒充灯条端点的实测宽高。缺少尺寸、倾角或参考系时，数值拟合即使初始化，也保持诊断状态且 `prediction=INVALID`；实际录像还可能因 PnP 法向、双解或拟合残差不合格而无法初始化。

实时相机只有配置 `rotation.feedback_frame_confirmed=true` 后才使用反馈姿态；这表示使用者已经确认 x前/y左/z上、yaw左转、pitch抬头、roll绕x右手和反馈角度单位。runtime 从最多512条完整状态帧历史中选取不晚于图像主机读取时间且年龄不超过50 ms的最近反馈，不插值、不使用未来反馈。时间基准是主机接收/读取时间，不是硬件曝光同步。文件回放不使用当前串口反馈；确认录像来自固定相机后才可显式采用 `--static-camera`。

应用默认旋转分支当前将瞄准结果标记为 `fixed_guard_not_configured`，因此尚不提供有效云台瞄准命令。固定守点、重定位、到点弹道及开火门控属于下一轮；`--no-rotation` 可对照原跟踪/瞄准基线，视觉开火仍默认关闭。

叠加图中，绿色是检测候选，紫色粗四边形是 `tracking.current_observation` 的本帧实际关联；没有本帧关联时不画历史框。文字显示 track/rotation 状态、诊断或可靠标志、半径、相位、角速度和预测有效性。仅在有效预测时绘制青色四板模型位置和关联板 `+0.10 s` 圆圈；暂失时标注 `PREDICTION ONLY`。模型投影标记不是可见目标检测框。

## 完整标定与输入方向

离线棋盘格工具的采集要求、实测板规格、图片/视频命令、逐视图误差和外参格式见 [标定说明](calibration.md)。`--calibration` 中的 K 与非零畸变直接交给现有 PnP，外参不再只依赖默认轴映射。

输入层在 BGR 转换后应用方向；检测、PnP、预览和原始 BGR 录像看到同一份处理后像素。相机方向与文件方向互不影响：相机需旋转时用 `--camera-rotation`，不会使后续视频回放自动重复旋转。新录像的 `<录像名>.metadata.yml` 记录实际存储尺寸和相对原生像素的方向，须随视频一起保留。旧文件无元数据时假设已有方向为 0；若已知像素本身已转过，使用 `--file-orientation` 声明。

每帧交付尺寸/方向必须与内参一致，外参也必须声明同一几何。不匹配直接报错，不自动缩放 K 或把旧方向的 R/t 当作新外参。几何元数据不能识别换镜头、同尺寸 ROI 或机械安装变化，这些仍需实际记录与重新验证。

## 海康 MVS USB 输入

设置 SDK 运行环境后，先枚举，再查询设备参数；两种查询都会提前退出，不创建 CDC transport、不启动采集：

```bash
build/srm_auto_aim_app --list-cameras \
  --mvs-library /opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3

build/srm_auto_aim_app --camera-info --mvs-serial ACTUAL_SERIAL \
  --mvs-library /opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3

build/srm_auto_aim_app --input mvs --mvs-serial ACTUAL_SERIAL \
  --mvs-library /opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3 \
  --dry-run --max-frames 120 --debug-dir replay_debug/mvs \
  --debug-every 20 --log mvs.csv --summary mvs_summary.txt
```

`ACTUAL_SERIAL` 必须替换为枚举结果。也可使用 `--mvs-index N`；序列号和非负索引互斥。没有选择参数时，只在枚举恰好一台 USB 相机时自动选择；不会在多设备时默认打开第一台。枚举输出包括 SDK 版本、实际加载请求路径、设备数量、索引、型号、序列号和 VID:PID。相机无需出现为 `/dev/video*`。

`--camera-info` 独占打开设备，只查询、不改参数、不启动取流；即使同时给出曝光等选项，也不应用这些设置。它报告宽高/ROI 偏移的当前值、范围和增量，曝光/增益/帧率的当前值与范围，像素格式等枚举的当前值和支持符号，以及节点读写状态；不支持的节点显示 `query_error`。

| 选项 | 默认与语义 |
|---|---|
| `--mvs-library PATH` | 空值时尝试系统库搜索路径及 `/opt/MVS/lib/64` 的标准库名和已知 4.7.0.3 文件；目前只接受 SDK `4.7.x` |
| `--camera-width N` / `--camera-height N` | `0` 保留当前尺寸；显式非零值验证 min/max/inc、写入并读回 |
| `--camera-fps FPS` | `0` 保留当前帧率控制；显式正值启用 `AcquisitionFrameRateEnable` 后设置并读回，不保证实际达到请求 FPS |
| `--exposure-us US\|default` | 默认保留曝光及自动控制；显式值关闭 `ExposureAuto`，设置微秒曝光并读回 |
| `--gain VALUE\|default` | 默认保留增益及自动控制；显式值关闭 `GainAuto`，按相机节点单位设置并读回，`0` 是显式值 |
| `--pixel-format NAME\|default` | 默认保留当前格式；显式值使用设备的 GenICam 符号并读回 |
| `--camera-timeout-ms N` | 默认 `1000`，接受 `[1,10000]`；一次 SDK 等帧的超时上限 |

曝光/增益/像素格式的 `default` 和宽高/FPS 的 `0` 会清除配置文件中的显式覆盖，保留打开设备时的当前状态，不是恢复出厂设置。启动采集会确保 `AcquisitionMode=Continuous`、`TriggerMode=Off`，其余未指定节点保留当前设置，包括白平衡和 ROI 偏移。参数不能写、越界、SDK 拒绝或读回不符时启动失败，不静默截断；本轮不回滚此前已经成功写入的节点。改变尺寸/ROI 后必须重新核对标定。

取流通过 `MV_CC_GetImageBuffer` 借用缓冲；BGR8 复制，其余格式调用 SDK 转为自有 BGR `cv::Mat`，成功和异常路径均归还缓冲。时间戳是主机取到帧时的 `steady_clock`，不是设备曝光时间；MVS 的 `source_time_seconds=0`。SDK 采用一个缓存节点，但这不等同于保证最新帧或已测得实时延迟。

超时、取流、元数据、转换或缓冲释放错误会进入现有读帧失败安全路径，打印具体 SDK 操作/错误码，写出 CSV 和 summary，随后以返回码 `1` 退出；实时失败不会记为文件 EOF。当前没有相机自动重连。可保存 `--debug-dir` 图像用于检查检测与颜色；dry-run MVS 支持 `--show-debug` 实时预览，但拒绝暂停或非零回放速度参数。

## 可选窗口调试

窗口均要求 `--dry-run`、可用的 OpenCV HighGUI 和桌面环境。缺少编译支持或 DISPLAY/WAYLAND_DISPLAY 时，在打开相机前给出清楚错误；窗口后端创建失败也会报告。相机预览示例：

```bash
bash run.sh mvs --show-debug --max-frames 0
# 同时保存可供重新检测的原始 BGR 视频：
bash run.sh mvs --show-debug --max-frames 0 --record replay_debug/camera-01.avi
```

MVS 和 OpenCV camera 每次成功处理帧后刷新同一套检测叠加图/亮度 mask 窗口。相机模式调用一次 `waitKey(10)` 给 GTK3 后端留出按键事件处理时间，修复快速刷新时 1 ms 等待无法可靠返回按键的问题；没有文件回放 deadline，不响应空格暂停或 n 单步，显式 `--start-paused` 或非零 `--playback-fps` 会报错。这项 GUI 等待会增加预览循环开销，实际耗时见 `gui_wait`。q/Q/Esc、关闭任一窗口或停止信号正常结束。GUI 后端仍有调度开销，且当前 SDK 等帧调用未返回前无法处理按键；响应延迟还受取帧超时和处理耗时影响，不能作为硬实时控制入口。

启用调试窗口时，也可在启动程序的前台终端按 q/Q/Esc，无需回车；暂停回放期间同样有效。终端临时关闭行缓冲和回显，保留 Ctrl+C 信号，并在正常或异常退出时恢复原设置；非终端标准输入不会被读取。终端方向键等转义序列以 Esc 开头，也会触发退出。窗口按键需先点击检测或 mask 窗口获得焦点。收到退出请求后先关闭窗口、恢复终端，再封口录像；终端显示 `finalizing recording` 时是在完整回读校验长录像，进程可能继续等待校验完成。

有桌面显示服务和 HighGUI 后端时，可逐帧查看视频的传统检测结果：

```bash
build/srm_auto_aim_app \
  --input video \
  --source demo.avi \
  --dry-run \
  --show-debug \
  --start-paused \
  --debug-dir replay_debug \
  --debug-every 5 \
  --summary replay_summary.txt \
  --log replay.csv
```

`SRM detection` 显示当前 BGR 帧、灯条、装甲板四角点和 ROI；上方信息栏显示帧序号、源时间、灯条/装甲板数量及播放状态。`SRM brightness mask` 显示该帧 `DetectionResult::brightness_mask`，采用现有的 BGR 通道最大值阈值结果。无装甲板时仍更新两个窗口；检测异常时显示当前原图、空二值图和 `detection unavailable`，不复用上一帧结果。

文件回放时，在任一窗口获得键盘焦点后：

- 空格：暂停或继续；暂停期间不读取下一帧，也不追加 CSV 或重复保存图像；
- `n`：前进一帧并保持暂停；播放中按 `n` 也会进入单步状态；
- `q`、Esc 或关闭任一窗口：正常退出并写出已处理帧的 summary；
- Ctrl+C / SIGTERM：播放和暂停期间都响应停止请求。

默认按 `FrameSource::nominal_fps()` 播放；源帧率不可用时回退为 30 FPS，可用 `--playback-fps 10` 放慢。播放等待计入检测和保存已花费的时间，处理慢时仍逐帧显示，不丢帧追赶；GUI 事件处理使实际帧率可能低于指定值。自动帧率的等待范围限制为 0.1–1000 FPS。播放时将剩余等待限制在 1–10 ms，已超时时仅请求 1 ms 事件处理，修复原先此分支仍固定请求 10 ms 的额外等待；暂停时请求 10 ms。实际耗时受桌面后端调度影响。

图片输入会在首帧自动暂停，方便观察；按 `n` 继续到下一帧或 EOF。视频读到 EOF 或达到 `--max-frames` 后关闭窗口；需要反复观察时使用 `--loop`，或先用 `--start-paused` 再单步。窗口显示每个成功读取的帧，`--debug-every` 只控制文件保存；两者共用检测、实际关联和旋转叠加，保存图不包含窗口额外的帧序号/播放信息栏。

窗口仅允许与 `--dry-run` 一起使用，因此不会在实际控制流程中等待窗口。文件回放保留暂停/单步，实时相机只泵送 GUI 事件。不启用窗口时不调用 HighGUI，也不加入播放等待；无桌面环境可继续使用 `--debug-dir`、录像、CSV 和 summary。若配置文件启用了窗口，可用 `--no-show-debug --no-start-paused --playback-fps 0` 覆盖；构建不支持窗口或缺少显示环境时，显式请求窗口会报告错误。

输入层分别提供主机读取时间 `Frame.timestamp` 与源运动时间 `Frame.motion_timestamp`。视频优先按源帧序号和名义 FPS 建立时间轴；无有效 FPS 时尝试视频位置时间，无法取得有限且递增的源时间时不提供运动时间。源首帧的 0 秒有效；暂停、单步和窗口播放速度不改变文件的源时间间隔。源运动时间与主机读取时间属于不同时间轴，不能直接相减。视频自动循环或显式 `reset()` 后，首帧的 `timeline_reset` 表示源时间轴重新开始；自动循环仍保持对外帧序号和主机读取时间递增。

`motion_time_source` 区分 `video_nominal_fps`、`video_position` 和 `host_arrival`。名义 FPS 不是实测曝光节奏；实时相机目前仍用主机取到帧的时间，也没有设备曝光时刻保证。输入时间轴的回归通过不代表后续算法或整机性能已经验收。`step_duration_us` 只记录运行步骤，不包含窗口显示、保存或播放等待。

## 原始 BGR 录像

参考 `sp_vision_25-main/tools/recorder.cpp` 使用 MJPG AVI 保存调用方传入的图像，同时另写 `t w x y z`；`src/standard_mpc.cpp:98–106` 在检测前调用 `record(img, q, t)`，传入的是相机原图。本项目沿用“原始图像用于复查/重新检测”的用途，保存 `Frame.bgr`，不保存检测叠加、mask 或窗口信息栏，也不写虚构的四元数/姿态文件。这里的原始帧指输入层已转换为 BGR 并完成配置旋转的图像，不是传感器 Bayer 原始数据。

```bash
# 不开窗口也可以录制；路径须尚不存在。
bash run.sh mvs --max-frames 300 --record replay_debug/raw-01.avi
# 需要逐像素保留 BGR 时使用本机可用的无损 FFV1：
bash run.sh mvs --record replay_debug/raw-lossless-01.avi --record-codec FFV1
# 之后用现有检测器重新回放：
bash run.sh video --source replay_debug/raw-01.avi --show-debug --max-frames 0
```

- 默认关闭，通过非空 `--record PATH` 或 `runtime.record` 启用；`--no-record`/空配置关闭。同步编码与校验仅用于 dry-run，`--live` 加录像会明确拒绝。
- 输出必须是 `.avi`，由 OpenCV FFmpeg 视频后端编码。默认 `MJPG` 是有损压缩；`FFV1` 可无损保存。必须有相应编码器/解码器，不存在时返回错误，不静默改格式或换后端。录像不依赖 HighGUI。
- 每个成功读取并进入应用处理分支的帧录一次，不受 `--debug-every` 影响；检测结果不可用时仍录本帧原图，文件暂停时不重复录像。尺寸必须为偶数且整段一致，拒绝静默裁掉奇数边或自动缩放。
- `--record-fps` 接受 0 或 `[0.1,1000]`，0 使用有效的输入名义 FPS，否则 30。它只是恒定帧率视频的播放元数据，不是采集限速，也不重采样/插帧。实际采集速度与元数据不同会导致回放时长不同；如需主机帧时间和检测结果，另保留现有 CSV，按成功帧顺序对应。
- 自动创建父目录，拒绝已有文件（含符号链接）；拒绝与 CSV/summary 路径相同。采集期间写同目录 `PATH.partial-XXXXXX.avi`，结束时 release 封口、fsync，并完整回读核对尺寸和帧数，随后通过无覆盖硬链接发布为 `PATH`、删除临时链接。录制期间另有同名文件出现时也不覆盖；文件系统不支持硬链接会明确失败并保留临时文件。
- 启动时还拒绝已有 `PATH.metadata.yml`；首个交付帧在写视频前独占创建 sidecar，记录旋转后的宽高/方向。回放自动读取它并验证尺寸，不受相机方向配置影响。失败可能留下 sidecar 和 partial 作为诊断；sidecar 存在不表示录像成功，下次使用新路径，保留原视频及元数据对应关系。
- OpenCV `VideoWriter::write` 返回 void，部分底层 I/O 失败不抛异常；本实现捕获可见写入错误，并在封口时校验静默短写/少帧。**成功写入检查包含退出时的完整解码，长视频的退出可能需要额外时间**；不是每帧落盘确认或硬件持久性保证。
- EOF、帧数上限、q/Esc、关闭窗口、SIGINT/SIGTERM 正常封口；MVS 取流失败也会封口已录到的有效帧，然后按采集失败返回 1。零帧不发布空视频。写入/封口/校验失败返回 1，不发布最终文件，诊断中给出保留的 partial 路径；异常终止或断电可能只留下不可播放的 partial 文件。
- 正常封口后 stderr 输出 `recorded_frames`、`record_path`。录像失败可能在 summary 写出前退出，已有旧摘要不能视为本次成功结果。

## 配置文件

命令行支持通过 `--config PATH` 读取简单的 UTF-8 文本配置。格式为一行一个 `key=value`，空行和以 `#` 开头的行会被忽略；不支持隐式未知键，拼写错误会直接报错。

示例：

```text
# offline replay configuration
input.type=video
input.source=sample.mp4
input.camera_index=0
input.loop=false
input.rotation_deg=0

transport.device=/dev/ttyACM0
transport.transmit_period_ms=10
transport.io_timeout_ms=20
transport.reconnect_ms=100

runtime.dry_run=true
runtime.log=replay.csv
runtime.debug_dir=replay_debug
runtime.summary=replay_summary.txt
runtime.debug_every=5
runtime.max_frames=0
runtime.show_debug=false
runtime.start_paused=false
runtime.playback_fps=0
runtime.record=
runtime.record_fps=0
runtime.record_codec=MJPG
detector.require_center_pattern=true
camera.intrinsics=800,800,320,240
camera.rotation_deg=0
```

当前支持的键：

- `input.type`
- `input.source`
- `input.camera_index`
- `input.width`
- `input.height`
- `input.fps`
- `input.loop`
- `input.rotation_deg`
- `input.orientation_deg`
- `transport.device`
- `transport.transmit_period_ms`
- `transport.io_timeout_ms`
- `transport.reconnect_ms`
- `runtime.dry_run`
- `runtime.log`
- `runtime.debug_dir`
- `runtime.summary`
- `runtime.debug_every`
- `runtime.max_frames`
- `runtime.show_debug`
- `runtime.start_paused`
- `runtime.playback_fps`
- `runtime.record`
- `runtime.record_fps`
- `runtime.record_codec`
- `detector.require_center_pattern`
- `detector.classifier.enabled`
- `detector.classifier.model`
- `detector.classifier.labels`
- `detector.classifier.min_confidence`
- `detector.classifier.conflict_margin`
- `pose.feature_width_m`
- `pose.feature_height_m`
- `rotation.enabled`
- `rotation.assume_static_camera`
- `rotation.feedback_frame_confirmed`
- `rotation.mounting_tilt_rad`
- `rotation.min_updates`
- `rotation.min_radius_m`
- `rotation.max_radius_m`
- `rotation.fit_window_seconds`
- `rotation.short_loss_seconds`
- `rotation.long_loss_seconds`
- `rotation.max_position_residual_m`
- `rotation.max_phase_residual_rad`
- `rotation.max_tilt_error_rad`
- `rotation.max_angular_speed_rad_s`
- `rotation.min_normal_span_rad`
- `camera.intrinsics`
- `camera.calibration`
- `camera.extrinsics`
- `camera.rotation_deg`
- `camera.mvs_serial`
- `camera.mvs_index`
- `camera.mvs_library`
- `camera.exposure_us`
- `camera.gain`
- `camera.pixel_format`
- `camera.timeout_ms`

保留设备当前参数的 MVS 配置示例：

```text
input.type=mvs
input.width=0
input.height=0
input.fps=0
camera.mvs_index=-1
camera.mvs_serial=ACTUAL_SERIAL
camera.mvs_library=/opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3
camera.exposure_us=default
camera.gain=default
camera.pixel_format=default
camera.timeout_ms=1000
runtime.dry_run=true
runtime.max_frames=120
runtime.show_debug=false
```

分类和旋转的应用默认值如下；特征尺寸默认省略，待实测后成对填写 `pose.feature_width_m`、`pose.feature_height_m`。模型路径按进程工作目录解析，构建时的默认路径指向本项目模型目录；移动源码/二进制部署时应显式配置资源路径。分类冲突 margin 在 `[0,1]` 内，近似等分的共享灯条冲突组会被拒绝。

```ini
detector.classifier.enabled=true
detector.classifier.min_confidence=0.7
detector.classifier.conflict_margin=0.05
rotation.enabled=true
rotation.assume_static_camera=false
rotation.feedback_frame_confirmed=false
rotation.mounting_tilt_rad=unknown
rotation.min_updates=8
rotation.min_radius_m=0.03
rotation.max_radius_m=1.0
rotation.fit_window_seconds=1.0
rotation.short_loss_seconds=0.15
rotation.long_loss_seconds=0.5
rotation.max_position_residual_m=0.08
rotation.max_phase_residual_rad=0.25
rotation.max_tilt_error_rad=0.20
rotation.max_angular_speed_rad_s=20
rotation.min_normal_span_rad=0.15
```

配置文件先于命令行应用；同名命令行参数覆盖配置文件值。例如：

```bash
build/srm_auto_aim_app \
  --config replay.conf \
  --intrinsics 810,810,320,240 \
  --no-loop \
  --debug-every 10
```

配置文件只负责运行参数，不包含协议字段、视觉开火许可或下位机固件配置；当前入口仍不会生成非零 `fire_flag`。

`camera.calibration` 和 `camera.extrinsics` 是 YAML 路径，空值表示未配置。`input.orientation_deg` 空值会报错，未确认旧文件方向时应省略该键；显式值必须为 0/90/180/270。用完整标定替换旧配置时，删除或注释 `camera.intrinsics`，不能仅追加 `--calibration` 而保留旧四参数。

log/summary 在打开前检查与输入文件、标定、外参及源/录像 sidecar 的规范化路径或已有文件别名冲突，以免截断输入。正常日志/摘要输出仍可能覆盖上次结果，建议使用独立批次目录。

## CSV 运行日志

`RuntimeCsvLogger` 输出以下字段，每个 `step()` 调用最多写一行：

```text
frame_sequence
frame_read
end_of_stream
frame_time_us
source_time_seconds
step_duration_us
transport_sent_frame_count
transport_received_frame_count
transport_communication_error_count
candidate_count
track_state
aim_valid
yaw_deg
pitch_deg
bullet_speed_m_s
safety_state
aim_command_valid
fire_allowed
fire_flag
safety_reasons
error
motion_time_us
motion_time_source
timeline_reset
track_status
current_detection_index
current_label
current_confidence
current_tl_x
current_tl_y
current_tr_x
current_tr_y
current_br_x
current_br_y
current_bl_x
current_bl_y
rotation_status
rotation_initialized
rotation_reliable
rotation_prediction_valid
rotation_stable_frame
rotation_pose_ambiguous
rotation_center_x_m
rotation_center_y_m
rotation_center_z_m
rotation_radius_m
rotation_phase_rad
rotation_omega_rad_s
rotation_position_rms_m
rotation_phase_rms_rad
rotation_observation_age_s
rotation_associated_plate
```

其中：

- `step_duration_us` 是本次运行步骤从读帧开始到安全命令写入 transport 的单调耗时；
- 三个 `transport_*` 字段是该行写入时的累计传输计数，可能受后台收发线程并发影响；
- `frame_time_us` 是 `steady_clock` 时间点相对于其时钟 epoch 的微秒值，只适合在同一进程/同一日志内比较延迟，不是 Unix 日历时间；
- `source_time_seconds` 是输入文件提供的源时间；图片和 MVS 输入为 0；
- `motion_time_us` 是本帧运动时间的微秒值；有效首帧可为 `0`，无法取得有效运动时间时留空。视频源时间与 `frame_time_us` 的主机时间属于不同时间轴，不能直接相减；
- `motion_time_source` 表示输入层的时间依据：`video_nominal_fps`、`video_position` 或 `host_arrival`。名义帧率不是实测曝光节奏；即使字段标为 `video_position`，仍需检查 `motion_time_us` 是否有值；
- `timeline_reset=1` 表示本帧开启新的源时间轴，例如视频循环回到起点或显式重置后的首帧；有帧且没有重置时为 `0`，没有帧时留空；
- `safety_reasons` 使用 `|` 分隔；包含逗号、引号或换行的字段按照 CSV 规则转义；
- `current_*` 只记录本帧实际关联；暂失或观测被拒时11列均留空，不写入历史观测。`current_detection_index` 是本帧检测下标，`current_label` 是模型内容类别，均不是永久目标编号；
- `rotation_initialized` 只表示有数值拟合；判断可靠性和是否允许模型投影必须同时看 `rotation_reliable`、`rotation_prediction_valid` 及参考系等门限；
- 未初始化时中心、半径、相位、角速度和两个残差共8列留空。`rotation_associated_plate` 是当前轨道内模4编号，重捕获后可能重新编号；
- 输入 EOF 也会产生一行 `end_of_stream=1`，这用于区分正常回放结束和处理失败；缺失的帧和管线字段留空，所有行保持与表头一致的52列。时间诊断保留原前24列，当前关联和旋转诊断追加28列；分析脚本应按列名读取。

CSV 记录用于离线诊断，不应直接作为下位机协议输入。

## key=value 运行摘要

指定 `--summary PATH` 后，程序在正常完成输入、达到 `--max-frames`、窗口退出、MVS 读帧失败或收到停止信号并完成安全停止后写出摘要。启动失败没有运行步骤，因此不生成运行摘要。摘要使用稳定的 `key=value` 行格式，至少包含：

```text
processed_frames=...
log_rows=...
dry_run=...
calibration_loaded=...
extrinsics_loaded=...
image_width=...
image_height=...
orientation_deg=...
step_count=...
frame_count=...
end_of_stream_count=...
error_count=...
candidate_count=...
safe_count=...
aiming_count=...
fire_ready_count=...
nonzero_fire_request_count=...
total_step_duration_us=...
average_step_duration_us=...
max_step_duration_us=...
transport_sent_frame_count=...
transport_received_frame_count=...
transport_communication_error_count=...
```

摘要用于比较离线回放、检查安全状态分布、检查处理耗时和观察 transport 线程；它不包含协议原始帧，也不能替代真实设备测试。`nonzero_fire_request_count` 在当前应用中应保持为 `0`。

`image_width`、`image_height`、`orientation_deg` 仅在有交付帧时出现，取首帧几何。加载状态为 1 只表示文件已成功解析/校验，不表示现场标定已验收。

## 分层耗时与卡顿定位

summary 对下列每个阶段输出 `<stage>_sample_count`、`total_<stage>_duration_us`、`average_<stage>_duration_us`、`max_<stage>_duration_us`。时钟为 `steady_clock`，单位微秒，平均值按该阶段自己的样本数计算，未执行的阶段样本数为 0。

| stage | 计时边界 |
|---|---|
| `acquisition` | `FrameSource::read`，包括解码/SDK 等帧、复制、方向处理和几何检查 |
| `detection` | 检测回调 |
| `pipeline` | 检测后的 PnP/选择/跟踪/弹道管线，不包含独立安全控制和 transport 写入 |
| `logging` | 每步 CSV logger 调用，含关闭日志时的空调用，不代表物理落盘确认 |
| `recording` | 每帧同步 `VideoRecorder::write`，不含首帧 sidecar 创建 |
| `debug_draw` | 应用绘制检测、当前关联和旋转诊断，不含窗口内部状态栏绘制 |
| `debug_save` | 同步调试 PNG 编码/保存 |
| `gui_display` | 窗口状态栏/mask 准备及 `imshow` 提交 |
| `gui_wait` | GUI 事件处理、文件节流或用户暂停等待 |
| `loop` | 一次应用循环，包含读帧至日志、录像、绘图、GUI 及首帧元数据等开销 |
| `recording_close` | 退出时封口、fsync、完整回读及发布，与循环单独记录 |

`acquisition` 对每步计数，可能包含最后一次 EOF；`detection`/`pipeline` 对读到帧的步骤计数，处理异常时未执行的阶段可能记 0。外层 `loop`、`step_duration` 和内部阶段相互包含，不能把这些平均值全部相加；`step_duration` 仍只包含 runtime 读帧至安全命令写入。计时小于 1 微秒会取整为 0。

先用同一离线输入和帧数、关闭窗口及保存输出取得基线，再分别启用录像、debug 存图，最后在桌面启用 GUI。读帧慢应结合源解码/SDK 等待检查；保存慢看对应编码阶段；`gui_wait` 很大先排除主动暂停和播放节流。录像退出变慢应看 `recording_close`，不要据 `step_duration` 推断总体吞吐。本轮离线计时没有验证真实桌面或相机性能，测量方法与现场待办见 [questions.md](../questions.md)。

## 安全约束

- 程序默认关闭视觉开火，当前命令入口没有启用视觉开火意图；默认和异常路径的 `fire_flag` 都为 0。
- `--dry-run` 不打开 CDC 设备；`--live` 在完整内外参校验后启动传输线程，按照正式 18 字节 CDC 命令发送，仍保持 `fire_flag=0`。
- 图片/视频 EOF、MVS 读帧失败、停止信号和正常退出前会尝试发送安全命令，然后停止传输线程。
- 日志、debug 图像、录像或摘要写入失败会使程序报告错误；传输对象通过析构路径停止后台线程，不把异常留成运行中的收发线程。窗口与同步录像均限定为 dry-run。
- 真实设备、云台和发射测试不由本入口自动获得许可，必须按照根目录 `questions.md` 执行。
