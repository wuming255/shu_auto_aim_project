# NUC、下位机与相机现场信息快照

日期：2026-10-14

## 保存位置与采集边界

本文件保存在开发机项目目录：

```text
/home/wuming/Projects/shu_auto_aim_project/dev_log/2026-10-14-nuc-field-snapshot.md
```

本轮现场命令通过 `ssh nuc` 执行，信息直接返回开发机后写入本文件。没有在 NUC 保存 Markdown、录像、CSV 或 summary。为取得 summary 收发计数，曾在 NUC `/tmp` 建立临时标定文件和 FIFO；测试结束后已删除。最后一次只读串口采集结束后停止所有 NUC 操作，后续可以断开 SSH。

本轮没有修改 NUC 项目源代码、配置、系统软件、MVS 安装、udev 规则或相机节点参数。相机当前图像获取正常。真实标定尚未提供；live 测试使用的占位内外参只用于通过软件启动安全门，不能用于视觉解算、云台控制或后续配置。

## 一、NUC 主机与软件环境

通过 `ssh nuc` 获取：

| 项目 | 现场值 |
|---|---|
| 主机名 | `nuc-NUC13ANKi7` |
| 内核 | `6.8.0-138-generic` |
| 系统 | Ubuntu 22.04.5 LTS，Jammy |
| 架构 | `x86_64` |
| CMake | 3.22.1 |
| g++ | Ubuntu 13.4.0-6ubuntu1~22~ppa2 |
| OpenCV | 4.5.4，通过 `pkg-config opencv4` 获取 |
| yaml-cpp | 0.7.0，通过 `pkg-config yaml-cpp` 获取 |
| 用户 | `wuming`，uid 1013，gid 1015 |
| 用户组 | `wuming dialout video render` |
| MVS Runtime | `/opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3` |
| MVS Runtime 大小 | 7,527,128 bytes |
| MVS Runtime 权限 | `-rwxrwxrwx`，所有者/组显示为 `UNKNOWN:UNKNOWN` |

`ldconfig` 能找到 OpenCV 和 yaml-cpp；本次输出没有发现 OpenVINO 或 `MvCameraControl` 被系统全局注册。程序通过显式 `--mvs-library` 成功加载 MVS Runtime。

NUC 项目路径为：

```text
/home/wuming/wuming/shu_auto_aim_project
```

该路径下未发现 `.git`，因此本轮不能用 Git HEAD 标识 NUC 上的源码版本。现场构建缓存确认：

```text
CMAKE_BUILD_TYPE=Debug
SRM_ENABLE_GUI=ON
SRM_ENABLE_MVS=ON
SRM_MVS_INCLUDE_DIR=/home/wuming/wuming/APP/mvs-reference-headers
```

NUC 二进制 SHA-256：

```text
c44b8f0f27ee90b710f559763864a38ccfe3c0e1ef2eea4e04f998bb1c63d91f  build/srm_auto_aim_app
```

关键文件时间：

```text
build/srm_auto_aim_app 2026-10-14 21:26:41 +0800
src/app/main.cpp       2026-10-14 20:52:17 +0800
src/io/mvs_camera.cpp  2026-10-14 14:13:45 +0800
CMakeLists.txt         2026-10-14 20:50:44 +0800
```

## 二、USB 总线与下位机 CDC

### USB 设备

现场 `lsusb` 和 `lsusb -t`：

```text
Bus 002 Device 003: ID 2bdf:0001 Hikrobot MV-CS016-10UC
Bus 001 Device 007: ID 0483:00ca STMicroelectronics Vision Comm port
```

相机位于 USB 3 链路：

```text
Bus 002 Port 2, 5000M
```

下位机 CDC 位于 USB 12M 链路，驱动为 `cdc_acm`：

```text
Bus 001 Port 3, Class Communications / CDC Data, Driver cdc_acm, 12M
```

### 设备节点与稳定标识

```text
/dev/ttyACM0
crw-rw---- 1 root dialout 166, 0
```

稳定设备链接：

```text
/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00 -> ../../ttyACM0
/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0 -> ../../ttyACM0
```

udev 关键属性：

```text
ID_VENDOR=YueLuEmbedded
ID_VENDOR_ID=0483
ID_MODEL=Vision_Comm_port
ID_MODEL_ID=00ca
ID_SERIAL=YueLuEmbedded_Vision_Comm_port_3855315E3439
ID_SERIAL_SHORT=3855315E3439
ID_USB_DRIVER=cdc_acm
ID_PATH=pci-0000:00:14.0-usb-0:3:1.0
```

当前用户属于 `dialout`，能够访问 `/dev/ttyACM0`。后续配置应优先考虑 `/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00`，不要只依赖可能随拔插变化的 `/dev/ttyACM0`；是否修改程序默认配置需要后续确认。

### 只通信 live 测试

用户确认现场满足不装弹/移除弹夹、关闭人工发射许可、云台和发射机构处于安全状态，并允许执行只通信测试。测试使用：

- 真实 MVS 相机输入；
- `--live` 启动 transport；
- `fire_flag=0` 的安全命令路径；
- `--no-show-debug`、`--no-record`；
- 处理上限 120 帧；
- 临时、格式合法但不真实的内参/外参，仅用于满足 live 启动校验；
- 相机处理方向显式设为 180°。

测试没有使用真实标定参数，也没有启用视觉开火或真实云台目标控制。summary 通过临时 FIFO 直接读回开发机，随后临时目录删除。

实际结果：

```text
processed_frames=120
dry_run=0
calibration_loaded=1
extrinsics_loaded=1
image_width=1440
image_height=1080
orientation_deg=180
step_count=120
frame_count=120
end_of_stream_count=0
error_count=0
candidate_count=0
safe_count=120
fire_ready_count=0
nonzero_fire_request_count=0
transport_sent_frame_count=91
transport_received_frame_count=962
transport_communication_error_count=0
```

该结果证明在本次连接状态下，NUC 程序可以打开 `/dev/ttyACM0`、启动正式 transport、发送安全帧并接收可解析的下位机状态帧。`transport_sent_frame_count=91` 小于处理帧数是因为发送线程按 10 ms 周期发送，不是每帧发送。

## 三、海康 MVS 相机

### 枚举信息

```text
MVS sdk=4.7.0.3
library=/opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3
usb_device_count=1
index=0
model=MV-CS016-10UC
serial=DA1830801
vid:pid=2bdf:0001
```

### 节点报告

本次 `--camera-info --mvs-serial DA1830801` 查询只打开设备并读取节点，没有开始取流，也没有设置节点：

| 节点 | 权限/当前值 | 范围或支持值 |
|---|---|---|
| Width | RW，1440 | 32–1440，增量 4 |
| Height | RW，1080 | 8–1080，增量 4 |
| OffsetX | RW，0 | 0，增量 4 |
| OffsetY | RW，0 | 0，增量 4 |
| ExposureTime | RW，3000 µs | 15–9,999,810 µs |
| Gain | RW，16.0114 | 0–16.9807 |
| AcquisitionFrameRate | RW，160 | 0.1–100000 |
| ResultingFrameRate | RO，160 | 只读 |
| PixelFormat | RW，BayerRG8 | 支持 Mono8/10/12、RGB8Packed、BGR8Packed、YUV422 和 BayerRG8/10/12 系列 |
| ExposureAuto | RW，Off | Off/Once/Continuous |
| GainAuto | RW，Off | Off/Once/Continuous |
| BalanceWhiteAuto | RW，Off | Off/Once/Continuous |
| TriggerMode | RW，Off | Off/On |
| AcquisitionMode | RW，Continuous | Continuous/SingleFrame |
| AcquisitionFrameRateEnable | RW，1 | — |

本次没有主动改曝光、增益、白平衡、分辨率、ROI、像素格式、FPS 或触发模式。节点报告中的 160 是设备名义/结果帧率，不能直接当作持续采集实测 FPS。

### 120 帧 dry-run

执行了无窗口、无录像、无日志、无 debug 图输出的 120 帧 MVS dry-run。结果：

```text
processed_frames=120
log_rows=0
```

程序只给出未配置真实内参和外参的诊断警告；相机枚举、参数查询、取流、BayerRG8 到 BGR 转换和 120 帧处理均成功。由于未保存 summary，本次 dry-run 没有额外的阶段耗时或 CSV 文件。

### 只读状态帧采样

live 程序停止后，以只读方式打开 `/dev/ttyACM0`，不发送任何字节，读取约 2 秒自然回传的正式 30 字节状态帧。串口设置恢复为打开前状态。

```text
readonly_state_frame_count=1993
last_state:
  yaw_deg=-0.0104105
  pitch_deg=-0.0127093
  roll_deg=-0.246326
  mode=1
  color=3
  bullet_speed_m_s=0
```

采样范围：

```text
yaw_deg:          -0.0132293 ..  0.0124050
pitch_deg:        -0.0391438 ..  0.00828917
roll_deg:         -0.248129  .. -0.224157
mode:              1 .. 1
color:             3 .. 3
bullet_speed_m_s:  0 .. 0
```

这里的 `mode=1`、`color=3` 是按正式协议解析出的原始整数值，当前不能仅凭协议结构推断其业务语义。`bullet_speed=0` 说明本次静态采样没有获得非零弹速，符合未发射/未进入有效测速状态的现场现象，但仍需确认下位机字段定义。

## 四、方向与运行配置

NUC 当前 `run.conf` 的关键配置：

```text
camera.rotation_deg=180
input.rotation_deg=0
runtime.dry_run=true
runtime.max_frames=0
runtime.show_debug=false
camera.mvs_serial=
camera.mvs_index=-1
camera.exposure_us=default
camera.gain=default
camera.pixel_format=default
camera.timeout_ms=1000
camera.calibration=
camera.extrinsics=
transport.device=/dev/ttyACM0
transport.transmit_period_ms=10
transport.io_timeout_ms=20
transport.reconnect_ms=100
```

用户反馈当前相机图像获取和画面正常；NUC 配置仍按 `camera.rotation_deg=180` 处理。该方向已经在本次 live/dry-run 几何摘要中体现为 `orientation_deg=180`。真实标定必须绑定这个处理后方向 180°，不能使用方向 0° 的内外参直接替代。

## 五、已确认事实与未完成事项

### 已确认

- NUC 能识别并独占打开唯一的 `MV-CS016-10UC` 相机。
- 相机当前输出为 1440×1080 BayerRG8，转换和连续取流正常。
- 当前用户具有相机和 `/dev/ttyACM0` 的访问权限。
- 下位机 CDC 能被正式程序打开，正式状态帧能够持续被解析。
- 只通信 live 测试 120 帧完成，通信错误为 0，所有安全帧 `fire_flag=0`。
- 只读串口采样收到 1993 个状态帧，未发送数据。
- 当前图像方向按 180° 处理，用户确认图像获取正常。

### 不能由本轮推断

- 不能把 160 FPS 节点值当成持续运行实测吞吐或端到端延迟。
- 不能把占位内外参作为真实标定结果。
- 不能从 `mode=1`、`color=3` 单独推断业务含义。
- 不能从 `bullet_speed=0` 推断下位机测速模块故障。
- 没有执行真实 GUI 桌面窗口测试；本次 SSH 无图形显示环境。
- 没有执行断开/重插 USB 的自动重连测试。
- 没有执行云台方向、静态目标 PnP、弹道、发射或视觉开火测试。
- 没有确认镜头型号、焦距、真实棋盘格规格、曝光策略、外参、云台坐标轴和视觉开火规则。

## 六、后续需要用户确认的问题

这些不是本轮断开 SSH 的阻塞项，但会影响后续项目开发和现场配置：

1. `mode=1` 和 `color=3` 在下位机正式定义中分别代表什么状态？当前自瞄模式、目标颜色是否与现场预期一致？
2. `bullet_speed=0` 是否因为当前未装弹/未发射而正常？进入允许测试状态后，回传弹速的单位和有效范围是否仍按正式协议的 m/s？
3. 是否将稳定设备路径配置为：

   ```text
   /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00
   ```

   还是继续使用 `/dev/ttyACM0`？
4. 相机镜头型号、焦距、当前焦点和是否使用 ROI 是什么？
5. 标定棋盘格的内角点列数、内角点行数和单格实测边长是多少？
6. 云台坐标原点、yaw/pitch 正方向、相机到云台外参的测量/求解方式何时可以提供？
7. 视觉开火角度/位置误差、稳定时间、目标丢失滞回、触发频率和人工许可规则是否已有正式定义？在这些信息确认前继续保持 `fire_flag=0`。

## 七、断开前建议

- 保留本文件和此前的 `dev_log/2026-10-14-nuc-mvs.md`，二者分别记录本次完整现场快照和早期 MVS 修复背景。
- 断开 SSH 前不需要在 NUC 执行清理命令；本轮临时目录、FIFO 和进程已经结束或删除。
- 后续开发以本文件中的实际相机节点、USB 标识、CDC 状态帧和方向配置为现场适配依据；真实标定和协议字段业务语义仍按上面的确认项补齐。

## 八、SSH 恢复后的追加确认

用户要求在断开前再次确认 SSH 是否可用。追加检查时间为 `2026-10-14T22:23:21+08:00`，结果：

```text
ssh_ok host=nuc-NUC13ANKi7 user=wuming
project_readable=1
```

本次追加检查仍未修改 NUC 文件或设备参数。当前 NUC 配置和文件摘要如下：

```text
run.conf sha256:
9ee281bb98b522bed13691901c0c94980a0ff5a59cc2a90ba9a16997377bee0d

build/srm_auto_aim_app sha256:
c44b8f0f27ee90b710f559763864a38ccfe3c0e1ef2eea4e04f998bb1c63d91f

CMakeLists.txt sha256:
2d4df0b38ff7b369d5bb2a296b5f77aef1d7278fd844225063eb3c1ac758e64a

src/app/main.cpp sha256:
a6f10b745d645f91d658c063ce6586754fb9f7ac406ab104f015f7d4724d723d

src/io/mvs_camera.cpp sha256:
ea58b0f481238280ccb6e5c77c020fd027b0def17e68f3188a6747c66935f846
```

稳定 CDC 链接仍指向 `/dev/ttyACM0`，设备权限仍为 `root:dialout`。追加的 `--camera-info` 与前一轮完全一致：MVS 4.7.0.3、`MV-CS016-10UC`、`DA1830801`、1440×1080、BayerRG8、曝光 3000 µs、增益 16.0114、结果帧率 160、Trigger Off、Continuous。

追加协议核对发现：正式规则 [self_aim_protocol.md](../shu_rules/self_aim_protocol.md) 第 4.1 节写明当前分支的 `mode` 固定发送 `0`，`color` 写入机器人 ID；而现场只读状态持续为 `mode=1`、`color=3`。这说明现场下位机固件/配置与文档中的“当前分支”至少存在一项差异，不能由上位机自行改写或把 `mode=1` 解释成枚举。后续应向下位机负责人确认固件版本、`mode=1` 的含义以及机器人 ID 是否确为 3，并决定是更新正式协议资料还是调整下位机/上位机版本匹配关系。

## 九、追加现场性能与稳定路径确认

### 稳定 CDC 路径 live 测试

使用稳定设备路径而不是 `/dev/ttyACM0`，执行 60 帧安全 live 测试：

```text
--device /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00
```

占位内外参、相机方向和安全前提与前一轮相同，只用于测试通信路径。结果：

```text
processed_frames=60
dry_run=0
calibration_loaded=1
extrinsics_loaded=1
image_width=1440
image_height=1080
orientation_deg=180
step_count=60
frame_count=60
end_of_stream_count=0
error_count=0
safe_count=60
nonzero_fire_request_count=0
transport_sent_frame_count=48
transport_received_frame_count=503
transport_communication_error_count=0
stable_path_live_exit_status=0
```

这证明当前用户权限、udev 稳定链接、serial 打开、相机取流和正式 CDC 收发可以一起工作。NUC 的 `run.conf` 尚未改为稳定链接，仍是 `/dev/ttyACM0`；是否修改需要用户确认。

### 600 帧真实相机 dry-run

执行无窗口、无录像、无日志和无 debug 图输出的 600 帧 MVS dry-run。没有真实内参/外参，因此该测试只用于采集和性能观察，不用于 PnP 或云台角度判断。

结果：

```text
processed_frames=600
dry_run=1
calibration_loaded=0
extrinsics_loaded=0
image_width=1440
image_height=1080
orientation_deg=0
step_count=600
frame_count=600
end_of_stream_count=0
error_count=0
candidate_count=0
safe_count=600
nonzero_fire_request_count=0
transport_sent_frame_count=0
transport_received_frame_count=0
transport_communication_error_count=0
throughput_dry_run_exit_status=0
```

阶段耗时：

| 阶段 | 平均 | 最大 |
|---|---:|---:|
| acquisition | 2.345 ms | 12.569 ms |
| detection | 3.933 ms | 9.273 ms |
| pipeline | 0.008 ms | 0.024 ms |
| loop | 6.803 ms | 21.889 ms |

按 600 帧总 loop 时间估算，本次 Debug 构建实际处理速率约为 147 帧/秒；相机节点报告的 160 不是本次实测吞吐。该结果只代表当前 NUC、当前 1440×1080 BayerRG8、当前场景和无窗口/无保存配置，不能代表持续长时间运行、GUI、录像或真实目标负载。

注意：该 dry-run 未显式设置 `--camera-rotation 180`，所以 summary 中 `orientation_deg=0` 是本次命令的处理方向；之前 live 测试显式使用 180°。现场正常运行和真实标定必须统一显式方向，不能仅根据这次性能测试的方向字段生成标定。

随后按现场配置显式加入 `--camera-rotation 180` 重复执行 600 帧 MVS dry-run。结果：

```text
processed_frames=600
dry_run=1
calibration_loaded=0
extrinsics_loaded=0
image_width=1440
image_height=1080
orientation_deg=180
step_count=600
frame_count=600
end_of_stream_count=0
error_count=0
safe_count=600
nonzero_fire_request_count=0
transport_sent_frame_count=0
transport_received_frame_count=0
transport_communication_error_count=0
rotation_180_dry_run_exit_status=0
```

旋转 180° 时平均 loop 为 7.132 ms，约 140 帧/秒；采集平均 4.289 ms，检测平均 2.309 ms。两次测试均无取流错误、无异常退出、无开火请求，确认正式方向参数不会破坏当前采集和安全 dry-run 链路。

## 十、用户确认后的配置收敛

用户确认以下处理方向：

- NUC 使用稳定 CDC 路径 `/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00`；
- 协议字段按正式文档理解，`mode` 应为 `0`；
- 当前没有真实相机内参、畸变和相机到云台外参，继续保持 `fire_flag=0` 和安全 dry-run。

源码核对结果：`mode`、`color` 只出现在下位机到上位机的 `GimbalState` 回传结构中；上位机发送结构只有 yaw、pitch 和 `fire_flag`，没有可设置的 `mode` 字段。因此上位机没有可以把现场回传 `mode=1` 改成 `0` 的配置项或发送逻辑。该差异仍属于下位机状态语义/固件版本确认项，不能由上位机猜测修复。

已按用户确认修改 NUC 的 `run.conf`，并同步修改本机 [run.conf](../run.conf)：

```text
transport.device=/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00
```

修改后的 NUC `run.conf` SHA-256 为：

```text
152b15c7bcc0793ca6e52c017db8fa09abb12efeef29310cfad7481a8405e25d
```

使用 `--config run.conf`、临时占位内外参和 `--live` 进行了 60 帧安全测试：

```text
config_live_exit_status=0
processed_frames=60
dry_run=0
calibration_loaded=1
extrinsics_loaded=1
image_width=1440
image_height=1080
orientation_deg=180
step_count=60
frame_count=60
end_of_stream_count=0
error_count=0
candidate_count=240
safe_count=60
fire_ready_count=0
nonzero_fire_request_count=0
transport_sent_frame_count=45
transport_received_frame_count=480
transport_communication_error_count=0
```

这次测试证明程序能够从修改后的配置加载稳定设备路径和 180° 方向，并完成相机、检测和 CDC 安全闭环。占位内外参仅用于接口验证，不可用于真实瞄准。

配置测试后再次只读查询相机节点，当前实际状态为：曝光 5000 µs、增益 16.0114、采集帧率 150、BayerRG8、白平衡自动连续、Trigger Off、Continuous。`run.conf` 中对应的曝光、增益和像素格式仍为 `default`，因此本轮没有通过配置主动写入这些相机参数；相机节点状态与早先快照中的 3000 µs/160 FPS/白平衡关闭不同，后续现场运行前应由负责人决定并记录正式曝光、帧率和白平衡设置。

曾有一次未带标定参数的 `--live` 配置测试被程序正确拒绝，原因是 live 必须同时提供完整内外参。该测试脚本因等待不存在的 FIFO summary 发生超时，远程残留临时目录已清理，未留下运行进程或设备动作。
