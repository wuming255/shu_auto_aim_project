# 运行入口、离线回放与运行报告

## 作用

`build/srm_auto_aim_app` 是当前项目的通用运行入口，用于把已有的 `FrameSource`、传统检测器、阶段六算法管线、阶段七安全决策和 CDC 传输层组合起来。

本入口支持 OpenCV 的图片、视频文件和 camera index 输入，以及可选的海康 MVS USB 相机输入。它们共用 `FrameSource` 和后续检测/运行报告接口；MVS 的真实相机采集仍需按 `questions.md` 验证。

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
- `--intrinsics fx,fy,cx,cy`：相机内参；
- `--log PATH`：写入 CSV 运行日志；
- `--summary PATH`：写入 key=value 运行摘要；
- `--debug-dir PATH`：创建目录并保存传统检测 debug 图像；为空时不保存；
- `--debug-every N`：每隔 N 个输入序号保存一张 debug 图，默认 `1`；
- `--show-debug` / `--no-show-debug`：开启或关闭可选窗口，默认关闭，仅支持 dry-run 图片/视频；
- `--playback-fps FPS`：窗口播放帧率，`0` 自动选择，非零值范围为 `[0.1, 1000]`；
- `--start-paused` / `--no-start-paused`：视频是否在第一帧暂停，默认不暂停；
- `--max-frames N`：处理 N 帧后退出，0 表示不限制；
- `--loop`：循环图片/视频文件；
- `--dry-run`：不启动串口线程，可用于离线回放或相机采集检查。此模式安全控制器会因 transport 未运行而拒绝有效瞄准命令；
- `--transmit-period-ms N`、`--io-timeout-ms N`、`--reconnect-ms N`：传输层时间参数；
- `--help`：显示完整帮助。

`--debug-dir` 会在启动时创建目录（包括必要的父目录）；图像文件命名为 `frame_000000.png`、`frame_000001.png` 等，编号使用 `Frame.sequence`。图像由 `TraditionalDetector::draw_debug()` 生成，不打开 GUI 窗口；当检测回调成功且序号满足 `debug-every` 时才写出。

未提供 `--intrinsics` 时，程序仍可启动用于输入和检测 smoke test，但会明确警告；不得把默认矩阵当作现场标定结果。

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

超时、取流、元数据、转换或缓冲释放错误会进入现有读帧失败安全路径，打印具体 SDK 操作/错误码，写出 CSV 和 summary，随后以返回码 `1` 退出；实时失败不会记为文件 EOF。本轮没有相机自动重连。可保存 `--debug-dir` 图像用于检查检测与颜色，实时 MVS 输入不接受 `--show-debug` 或暂停选项。

## 可选窗口调试

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

在任一窗口获得键盘焦点后：

- 空格：暂停或继续；暂停期间不读取下一帧，也不追加 CSV 或重复保存图像；
- `n`：前进一帧并保持暂停；播放中按 `n` 也会进入单步状态；
- `q`、Esc 或关闭任一窗口：正常退出并写出已处理帧的 summary；
- Ctrl+C / SIGTERM：播放和暂停期间都响应停止请求。

默认按 `FrameSource::nominal_fps()` 播放；源帧率不可用时回退为 30 FPS，可用 `--playback-fps 10` 放慢。播放等待计入检测和保存已花费的时间，处理慢时仍逐帧显示，不丢帧追赶；GUI 事件处理使实际帧率可能低于指定值。自动帧率的等待范围限制为 0.1–1000 FPS。

图片输入会在首帧自动暂停，方便观察；按 `n` 继续到下一帧或 EOF。视频读到 EOF 或达到 `--max-frames` 后关闭窗口；需要反复观察时使用 `--loop`，或先用 `--start-paused` 再单步。窗口显示每个成功读取的帧，`--debug-every` 只控制文件保存，保存的图像仍由原来的 `draw_debug()` 生成，不包含窗口信息栏。

窗口仅允许与 `--dry-run` 和文件输入一起使用，因此暂停不会挂起实时云台控制。不启用窗口时不调用 HighGUI，也不加入播放等待；无桌面环境可继续使用原有的 `--debug-dir`、CSV 和 summary。若配置文件启用了窗口，可用 `--no-show-debug --no-start-paused --playback-fps 0` 覆盖；构建不支持窗口或缺少显示环境时，显式请求窗口会报告错误。

跟踪仍使用读取时的单调时间戳，播放速度和暂停会改变帧间时间；窗口回放用于观察检测结果，不能据此评估实际目标角速度、预测精度或现场控制性能。`step_duration_us` 只记录运行步骤，不包含窗口显示、保存或播放等待。

## 配置文件

命令行支持通过 `--config PATH` 读取简单的 UTF-8 文本配置。格式为一行一个 `key=value`，空行和以 `#` 开头的行会被忽略；不支持隐式未知键，拼写错误会直接报错。

示例：

```text
# offline replay configuration
input.type=video
input.source=sample.mp4
input.camera_index=0
input.loop=false

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
camera.intrinsics=800,800,320,240
```

当前支持的键：

- `input.type`
- `input.source`
- `input.camera_index`
- `input.width`
- `input.height`
- `input.fps`
- `input.loop`
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
- `camera.intrinsics`
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

配置文件先于命令行应用；同名命令行参数覆盖配置文件值。例如：

```bash
build/srm_auto_aim_app \
  --config replay.conf \
  --intrinsics 810,810,320,240 \
  --no-loop \
  --debug-every 10
```

配置文件只负责运行参数，不包含协议字段、视觉开火许可或下位机固件配置；当前入口仍不会生成非零 `fire_flag`。

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
```

其中：

- `step_duration_us` 是本次运行步骤从读帧开始到安全命令写入 transport 的单调耗时；
- 三个 `transport_*` 字段是该行写入时的累计传输计数，可能受后台收发线程并发影响；
- `frame_time_us` 是 `steady_clock` 时间点相对于其时钟 epoch 的微秒值，只适合在同一进程/同一日志内比较延迟，不是 Unix 日历时间；
- `source_time_seconds` 是输入文件提供的源时间；图片和 MVS 输入为 0；
- `safety_reasons` 使用 `|` 分隔；包含逗号、引号或换行的字段按照 CSV 规则转义；
- 输入 EOF 也会产生一行 `end_of_stream=1`，这用于区分正常回放结束和处理失败；缺失的帧和管线字段留空，所有行保持与表头一致的 21 列。

CSV 记录用于离线诊断，不应直接作为下位机协议输入。

## key=value 运行摘要

指定 `--summary PATH` 后，程序在正常完成输入、达到 `--max-frames`、窗口退出、MVS 读帧失败或收到停止信号并完成安全停止后写出摘要。启动失败没有运行步骤，因此不生成运行摘要。摘要使用稳定的 `key=value` 行格式，至少包含：

```text
processed_frames=...
log_rows=...
dry_run=...
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

## 安全约束

- 程序默认关闭视觉开火，当前命令入口没有启用视觉开火意图；默认和异常路径的 `fire_flag` 都为 0。
- `--dry-run` 不打开 CDC 设备；普通模式会启动传输线程并按照正式 18 字节 CDC 命令发送。
- 图片/视频 EOF、MVS 读帧失败、停止信号和正常退出前会尝试发送安全命令，然后停止传输线程。
- 日志、debug 图像或摘要写入失败会使程序报告错误；传输对象通过析构路径停止后台线程，不把异常留成运行中的收发线程。
- 真实设备、云台和发射测试不由本入口自动获得许可，必须按照根目录 `questions.md` 执行。
