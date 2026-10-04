# 运行入口与离线日志

## 作用

`build/srm_auto_aim_app` 是当前项目的通用运行入口，用于把已有的 `FrameSource`、传统检测器、阶段六算法管线、阶段七安全决策和 CDC 传输层组合起来。

本入口当前只绑定 OpenCV 的图片、视频文件和 camera index 输入，不绑定海康 MVS SDK。MVS 适配仍需在现场环境确认后接入 `FrameSource` 接口。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
```

可执行文件：

```text
build/srm_auto_aim_app
```

## 输入模式

默认输入模式为 OpenCV camera index 0。离线图片和视频示例：

```bash
build/srm_auto_aim_app \
  --input image \
  --source sample.png \
  --dry-run \
  --intrinsics 800,800,320,240 \
  --log run.csv

build/srm_auto_aim_app \
  --input video \
  --source sample.mp4 \
  --device /dev/ttyACM0 \
  --intrinsics 800,800,320,240 \
  --log run.csv
```

常用选项：

- `--input image|video|camera`：输入类型；
- `--source PATH`：图片或视频路径；
- `--camera-index N`：OpenCV camera index；
- `--device PATH`：USB CDC 设备，默认 `/dev/ttyACM0`；
- `--intrinsics fx,fy,cx,cy`：相机内参；
- `--log PATH`：写入 CSV 运行日志；
- `--max-frames N`：处理 N 帧后退出，0 表示不限制；
- `--loop`：循环图片/视频文件；
- `--dry-run`：不启动串口线程，仅用于离线检查。此模式安全控制器会因 transport 未运行而拒绝有效瞄准命令；
- `--transmit-period-ms N`、`--io-timeout-ms N`、`--reconnect-ms N`：传输层时间参数；
- `--help`：显示完整帮助。

未提供 `--intrinsics` 时，程序仍可启动用于输入和检测 smoke test，但会明确警告；不得把默认矩阵当作现场标定结果。

## 安全约束

- 程序默认关闭视觉开火，当前命令入口没有启用视觉开火意图；默认和异常路径的 `fire_flag` 都为 0。
- `--dry-run` 不打开 CDC 设备；普通模式会启动传输线程并按照正式 18 字节 CDC 命令发送。
- 图片/视频 EOF、停止信号和正常退出前会尝试发送安全命令，然后停止传输线程。
- 真实设备、云台和发射测试不由本入口自动获得许可，必须按照根目录 `questions.md` 执行。

## CSV 日志字段

`RuntimeCsvLogger` 输出以下字段：

```text
frame_sequence
frame_read
end_of_stream
frame_time_us
source_time_seconds
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

`frame_time_us` 是 `steady_clock` 时间点相对于其时钟 epoch 的微秒值，只适合在同一进程/同一日志内比较延迟，不是 Unix 日历时间。`safety_reasons` 使用 `|` 分隔；包含逗号、引号或换行的字段按照 CSV 规则转义。

CSV 记录用于离线诊断，不应直接作为下位机协议输入。
