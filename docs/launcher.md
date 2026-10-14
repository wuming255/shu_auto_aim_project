# 一条命令启动与参数速查

## 1. 最常用的运行方式

项目根目录新增两个文件：

- `run.sh`：启动脚本，选择运行模式、加载配置、设置已有 MVS SDK 的进程环境。
- `run.conf`：常用参数。长期使用的路径、相机参数、窗口设置写在这里，以后不必反复输入。

已构建项目后，在项目根目录运行：

```bash
bash run.sh
```

默认回放 `demo.avi` 至 EOF，不开窗口，不启动 CDC，并保存检测图、CSV 和摘要。当前 `runtime.max_frames=0` 表示无帧数上限。`demo.avi` 是本地素材，迁移到其他机器时需提供该视频或修改 `input.source`。

首次使用或代码更新后构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

脚本找不到可执行文件时会提示构建命令。脚本不自动编译或安装 SDK。

## 2. 选择功能

| 命令 | 功能 |
|---|---|
| `bash run.sh` 或 `bash run.sh run` | 按 `run.conf` 运行 |
| `bash run.sh video` | 视频回放，文件取自配置 |
| `bash run.sh image --source target.png` | 图片输入 |
| `bash run.sh video --show-debug --start-paused` | 视频窗口调试，首帧暂停 |
| `bash run.sh mvs` | 海康 USB 采集 |
| `bash run.sh mvs --show-debug --max-frames 0` | 海康实时检测预览，q/Esc 或关闭窗口退出 |
| `bash run.sh mvs --show-debug --record replay_debug/camera-01.avi` | 预览并保存原始 BGR 视频，持续至用户停止 |
| `bash run.sh camera` | OpenCV camera index 输入 |
| `bash run.sh list` | 枚举海康 USB 相机并退出 |
| `bash run.sh info` | 打开所选海康设备、查询参数并退出，不改节点、不取流 |
| `bash run.sh --help` | 脚本中文帮助，无需已构建程序 |
| `bash run.sh app-help` | 原程序完整参数帮助 |

`mvs`/`camera` 模式会清空文件源并关闭暂停和播放限速，窗口开关继续取配置或追加的 `--show-debug`/`--no-show-debug`。相机预览逐帧刷新，不暂停、不单步、不做回放节流。`video`/`image` 只覆盖输入类型，文件路径仍需匹配。不写模式时完全使用配置的输入设置。

各模式继续使用配置中的 `runtime.dry_run` 和帧数上限；随附配置为 dry-run、帧数不限。短检查可显式加 `--max-frames 120`，持续运行按 Ctrl+C 结束。MVS 相机无需出现为 `/dev/video*`，应使用 `mvs` 而非 `camera` 模式。

## 3. 配置一次，以后直接启动

例如平时主要做视频逐帧观察，在 `run.conf` 修改：

```ini
input.type=video
input.source=demo.avi
runtime.dry_run=true
runtime.max_frames=0
runtime.show_debug=true
runtime.start_paused=true
runtime.playback_fps=30
```

以后只需 `bash run.sh`。平时主要使用海康时可改为：

```ini
input.type=mvs
input.source=
camera.mvs_serial=ACTUAL_SERIAL
camera.mvs_index=-1
runtime.dry_run=true
runtime.max_frames=120
runtime.show_debug=false
runtime.start_paused=false
runtime.playback_fps=0
```

`ACTUAL_SERIAL` 替换成 `bash run.sh list` 输出的真实序列号；仅有一台相机时可以把序列号留空。上面都是修改现有文件中的对应行，不必重复添加。

配置规则：

- 一行一个 `key=value`，整行 `#` 注释；不支持行尾注释。
- 值不加引号，路径有空格也直接写，例如 `input.source=my videos/test.avi`。
- 不执行 shell，不展开 `$HOME`、`~` 或 `$(...)`；需要时使用绝对路径。
- 所有相对路径都相对于**项目根目录**，包括 `--config`、`--source`、输出路径、`SRM_APP` 和 `MVS_ROOT`。从别处运行也一样。
- 优先级为：**配置文件 → 模式预设 → 追加的应用参数**。不支持的键和非法值会报错。

多套配置可分别保存，显式 `--config` 会替换默认 `run.conf`，不是叠加：

```bash
bash run.sh --config my-settings.conf
bash run.sh video --config my-settings.conf --source "my videos/test.avi"
```

## 4. 可选参数速查

下列参数均可追加到脚本命令后。表中的默认值指随附 `run.conf`，可能与直接运行原程序的默认值不同。

### 输入、回放与窗口

| 命令行参数 | 配置键 | 功能/随附默认值 |
|---|---|---|
| `--input image\|video\|camera\|mvs` | `input.type` | 输入类型，默认 video |
| `--source PATH` | `input.source` | 文件路径，默认 demo.avi |
| `--camera-index N` | `input.camera_index` | OpenCV 设备索引，默认 0 |
| `--max-frames N` | `runtime.max_frames` | 默认 0，不限制 |
| `--loop` / `--no-loop` | `input.loop` | 循环文件输入，默认 false |
| `--show-debug` / `--no-show-debug` | `runtime.show_debug` | 两个检测窗口，默认 false；相机实时预览 |
| `--start-paused` / `--no-start-paused` | `runtime.start_paused` | 视频首帧暂停，默认 false；图片窗口自动暂停 |
| `--playback-fps FPS` | `runtime.playback_fps` | 窗口播放速度；0 自动，非零范围 0.1–1000 |
| `--calibration PATH` | `camera.calibration` | 完整 K/畸变/尺寸/方向 YAML，默认空 |
| `--extrinsics PATH` | `camera.extrinsics` | 处理后相机到云台 R/t YAML，默认空，要求完整内参 |
| `--intrinsics fx,fy,cx,cy` | `camera.intrinsics` | 旧四参数方式，默认未设置，仅供 dry-run，与完整标定互斥 |
| `--camera-rotation DEG` | `camera.rotation_deg` | 仅相机旋转，默认 0，顺时针 0/90/180/270 |
| `--file-rotation DEG` | `input.rotation_deg` | 仅文件追加旋转，默认 0 |
| `--file-orientation DEG` | `input.orientation_deg` | 已有像素方向，仅旧文件有明确记录时填写；新录像自动读 sidecar |

窗口显示检测叠加图与亮度二值图，均可用 **q/Esc**或关闭任一窗口退出。文件回放还支持**空格**暂停/继续、**n**单步；相机预览忽略这两个键，拒绝显式暂停/非零播放速度选项。窗口仅支持 dry-run，依赖 HighGUI 和桌面环境。缺少支持时明确报错，不能把 GUI 加入 live 控制流程。窗口回放的暂停/变速会改变跟踪时间间隔，主要用于观察检测。

`build/srm_calibrate_camera` 是独立的离线标定入口，不通过脚本模式启动。先实测板型、内角点行列和方格尺寸，再按 [标定说明](calibration.md) 处理图片目录或视频。相机与文件方向分开设置；方向在输入层生效，检测、预览和录像保持一致。新录像自动携带已存方向，回放通常无需再旋转。

### 海康相机

| 命令行参数 | 配置键 | 功能/随附默认值 |
|---|---|---|
| `--mvs-serial SERIAL` | `camera.mvs_serial` | 真实序列号；默认空 |
| `--mvs-index N` | `camera.mvs_index` | 枚举索引；默认 -1，仅单设备自动选择；非负索引与序列号互斥 |
| `--mvs-library PATH` | `camera.mvs_library` | 指定 4.7.x 主运行库；默认空，自动搜索 |
| `--camera-width N` | `input.width` | 默认 0，保留当前宽度 |
| `--camera-height N` | `input.height` | 默认 0，保留当前高度 |
| `--camera-fps FPS` | `input.fps` | 默认 0，保留当前；与窗口 playback-fps 不同 |
| `--exposure-us US\|default` | `camera.exposure_us` | 微秒曝光；默认 default 保留，显式正值关闭自动曝光 |
| `--gain VALUE\|default` | `camera.gain` | 相机节点单位；默认保留，0 是显式增益值 |
| `--pixel-format NAME\|default` | `camera.pixel_format` | 设备支持的格式符号；默认保留 |
| `--camera-timeout-ms N` | `camera.timeout_ms` | 取帧超时，默认 1000，范围 1–10000 ms |

尺寸/FPS 也适用于 OpenCV camera；MVS 会检查节点范围/增量、权限和读回。先 `info` 查看真实范围，再配置；曝光/增益/格式的 `default` 表示保留当前状态，不是恢复出厂设置。采集会设置连续模式并关闭触发，未指定的其他节点保留。当前没有相机自动重连。

### 输出与通信

| 命令行参数 | 配置键 | 功能/随附默认值 |
|---|---|---|
| `--debug-dir PATH` | `runtime.debug_dir` | 检测叠加图，默认 replay_debug/latest/images |
| `--debug-every N` | `runtime.debug_every` | 每 N 帧存图，默认 20，必须大于 0 |
| `--log PATH` | `runtime.log` | CSV，默认 replay_debug/latest/run.csv |
| `--summary PATH` | `runtime.summary` | 摘要，默认 replay_debug/latest/summary.txt |
| `--record PATH` / `--no-record` | `runtime.record` | 原始 BGR 录像，默认空（关闭），路径须为尚不存在的 .avi |
| `--record-fps FPS` | `runtime.record_fps` | 视频播放元数据 FPS，默认 0 使用输入名义 FPS 或回退 30；不控制采集速度 |
| `--record-codec FOURCC` | `runtime.record_codec` | 默认 MJPG（有损）；FFV1 可无损；依赖 OpenCV FFmpeg 编解码支持 |
| `--dry-run` / `--live` | `runtime.dry_run` | 默认 true；--live 启动真实 CDC 传输 |
| `--device PATH` | `transport.device` | CDC 设备，默认 /dev/ttyACM0 |
| `--transmit-period-ms N` | `transport.transmit_period_ms` | 发送周期，默认 10 ms |
| `--io-timeout-ms N` | `transport.io_timeout_ms` | 串口 I/O 超时，默认 20 ms |
| `--reconnect-ms N` | `transport.reconnect_ms` | CDC 重连间隔，默认 100 ms |

重复使用默认输出位置会覆盖 CSV/摘要及同名图像，旧的其他图像不会自动删除。需要保留一次结果时，使用不同批次目录；应用先创建 debug 目录，因此下例日志父目录也会存在：

```bash
bash run.sh --debug-dir replay_debug/session-01/images \
  --log replay_debug/session-01/run.csv \
  --summary replay_debug/session-01/summary.txt
```

关闭输出可把配置值留空，或追加 `--debug-dir "" --log "" --summary ""`。单独改变 log/summary 路径时需确保父目录存在。debug 图片是检测叠加图，不是原始录像。

### 开启录像

```bash
bash run.sh mvs --show-debug --record replay_debug/session-01.avi --max-frames 0
# 不开窗口录像：
bash run.sh mvs --no-show-debug --record replay_debug/session-02.avi
# 后续重新运行检测观察：
bash run.sh video --source replay_debug/session-01.avi --show-debug --no-record --max-frames 0
```

也可以在 `run.conf` 设置 `runtime.record=replay_debug/session-01.avi`。路径固定且**不覆盖已有录像及其 `.metadata.yml`**，下一次录制请换新路径；`--no-record` 可临时关闭。与 CSV/摘要的覆盖规则不同，录像拒绝同名文件与输出路径冲突。复制或重命名视频时一并保留 sidecar，回放才能核对方向和存储尺寸；失败后留下的 sidecar 不代表已成功封口。

参考 Recorder 保存检测前的原图和四元数文本。本项目保存输入层的 BGR 帧，用于重新检测，不包含框线、mask、窗口栏或虚构云台姿态；默认 MJPG 会有压缩损失，需要逐像素保留时加 `--record-codec FFV1`。现有 CSV 可另存处理结果与主机时间。

同步录像也限定为 dry-run。每个成功读取的帧写一次，和 `debug-every` 无关；FPS 是播放元数据，不保证等于实测相机帧率。正常退出时封口并完整回读校验，再发布指定文件；长视频退出会多花一些时间。写入/编码/封口错误返回 1，保留诊断中的 `.partial-XXXXXX.avi` 而不发布最终视频；零帧不生成视频。完整内容、尺寸限制、文件系统要求和失败行为见 [原始 BGR 录像说明](application.md#原始-bgr-录像)。

`--live` 要求完整 `--calibration` 和 `--extrinsics` 才能启动 CDC，当前入口仍保持 `fire_flag=0`。真实 CDC/云台操作按 [questions.md](../questions.md) 执行。未配置标定时 dry-run 采集和检测观察可继续，默认矩阵不代表标定结果。卡顿定位请查看摘要中的 acquisition、detection、pipeline、录像、绘图/保存、GUI 显示/等待和完整 loop 耗时，口径见 [分层耗时说明](application.md#分层耗时与卡顿定位)。

## 5. 海康 SDK 和其他构建目录

若 NUC 已有 `/opt/MVS/lib/64`，脚本会自动为本次进程加入 SDK 模块搜索路径，无需每次手动 export。非标准位置可一次在终端设置：

```bash
export MVS_ROOT="$HOME/sdk/MvCamCtrlSDK"
bash run.sh list
bash run.sh info
bash run.sh mvs
```

也可修改 `run.sh` 中 `mvs_root` 的默认根目录。脚本只设置 `MVCAM_COMMON_RUNENV`（已有非空值时保留）与 `LD_LIBRARY_PATH`（前置 SDK lib/64），不会修改系统文件。`--mvs-library` 仅选择主库，SDK 根目录仍需保留配套模块。

构建时仍需 MVS 开发头文件；缺头文件或关闭 MVS 的构建不能通过脚本自动获得相机支持。当前完整 4.7 头文件与真实设备采集的待验证事项见 [部署文档](deployment.md#海康-mvs-构建与运行环境) 和 [现场清单](../questions.md)。

可选其他构建产物：

```bash
SRM_APP=build-release/srm_auto_aim_app bash run.sh
```

脚本使用 `exec` 启动原程序，传递 Ctrl+C 和程序退出码；应用失败返回 1，未知脚本模式返回 2。`list`/`info` 提前退出，不生成运行报告；采集启动失败也不会生成新摘要，勿把旧文件当成本次结果。
