# 部署与交付检查

## 适用范围

本文档描述当前软件主体在开发机或现场 NUC 上的构建、离线验收和运行入口使用方式。它不替代根目录 `questions.md` 中的现场安全、相机、CDC、标定、云台和发射测试清单。

当前软件交付边界：

- 支持图片、视频文件、OpenCV camera index 和可选 MVS USB 相机输入；
- 支持正式自瞄 CDC 协议对应的 transport；
- 支持 `--dry-run` 离线运行及不启动 CDC 的相机采集检查；
- 默认不启用视觉开火，所有默认和异常路径保持 `fire_flag=0`；
- MVS 软件适配已接入，真实相机采集和完整 SDK ABI 仍需最终确认；现场标定参数通过配置提供。

## 构建与自动化验收

在项目根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

OpenCV HighGUI 是窗口调试的可选依赖，配置输出会显示窗口支持状态。无桌面环境仍可执行默认的非 GUI 回放；需要完全关闭窗口编译时：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DSRM_ENABLE_GUI=OFF
cmake --build build -j2
```

重新启用使用 `-DSRM_ENABLE_GUI=ON`，并确保 OpenCV 提供可用的 HighGUI 后端。

CTest 至少应包含：

- 正式协议编解码和流式解析；
- 伪终端 transport 收发、拆包和重连；
- 图片/视频输入；
- 检测、PnP、选择、跟踪、补偿和安全控制；
- 应用配置、CSV、运行统计；
- `srm_auto_aim_app` 离线 smoke test。

MVS 启用时额外包含 `srm_mvs_camera_test` 和 `srm_mvs_app_smoke_test`：用模拟 SDK 验证选择、参数、帧缓冲所有权、清理、缺符号/版本错误和采集失败摘要；测试不要求真实 SDK 或相机。当前默认有头文件构建为 15 项；关闭 MVS 或缺头文件时为 13 项。

当前离线 smoke test 使用 `tests/data/application_smoke.ppm`，由 CTest 自动验证：

- 程序能启动并处理一帧；
- debug 图像、CSV 和 summary 文件能够生成；
- summary 包含 `processed_frames=1`、`dry_run=1` 和 `nonzero_fire_request_count=0`；
- 应用入口没有因为检测不到目标而产生非零开火请求。

## 海康 MVS 构建与运行环境

编译需要开发 SDK 的完整 include 目录（`MvCameraControl.h` 及其引用的 `CameraParams.h`、`PixelType.h`、错误和旧接口头文件），无需在编译时链接海康运行库。可在个人用户目录提供头文件：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
  -DSRM_ENABLE_MVS=ON -DSRM_MVS_INCLUDE_DIR="$HOME/sdk/MVS-4.7/include"
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

将示例路径替换为实际开发 SDK include。CMake 自动搜索 `/opt/MVS/include`、`/opt/MvCamCtrlSDK/include`，最后尝试本地参考项目 `sp_vision_25-main/io/hikrobot/include`，并打印具体来源。参考项目目录被 gitignore 排除，不是干净 checkout 的必要组成部分；部署不能依赖它存在。没有头文件时默认降级为离线/OpenCV 构建，MVS 请求明确报不可用；`-DSRM_ENABLE_MVS=OFF` 可显式关闭。

本次提供的 `MvCamCtrlSDK_Runtime-4.7.0_x86_64_20251113.tar.gz` 仅包含运行库和安装/环境脚本，缺少对应开发头文件、采集示例与完整 API 文档。已测得其库版本为 `4.7.0.3`，参考项目附带库为 `4.1.2.2`；当前本地编译使用参考头文件回退，只能证明已验证的符号、初始化、枚举和离线转换调用。版本门接受 `4.7.x`，无法证明所有 4.7 版本/帧结构 ABI 匹配；最终部署应补齐对应开发 SDK 头文件，并重新构建与验证。不得用参考项目 4.1 运行库替代本次运行库。

运行需要保留整个 SDK `lib/64` 目录及配套文件；仅复制 `libMvCameraControl.so` 可能缺少 SDK 内部按需加载的 USB/转换模块。已存在 `/opt/MVS` 的 NUC 可在当前 shell 设置：

```bash
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export LD_LIBRARY_PATH="/opt/MVS/lib/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
build/srm_auto_aim_app --list-cameras \
  --mvs-library /opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3
```

用户目录解压的 SDK 同理把两处 `/opt/MVS` 换成实际根目录；`MVCAM_COMMON_RUNENV` 指向 `lib`，`LD_LIBRARY_PATH` 加入 `lib/64`。这些设置仅影响当前 shell，不需管理员权限；本项目不执行 `setup.sh`、`set_env_path.sh` 或 USB/system tuning 脚本。`--mvs-library` 仅指定主运行库，不替代模块搜索环境。图片/视频/OpenCV 输入不加载该库，`ldd build/srm_auto_aim_app` 不应出现海康直接依赖。

有头文件构建还提供手动离线转换检查：

```bash
build/srm_mvs_sdk_conversion_test \
  /opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3
```

该程序加载真实 SDK，用合成设备描述建立未打开设备的本地句柄，检查 RGB8、Mono8 和四种 Bayer8 转 BGR 的长度与已知颜色；不枚举、不打开设备、不取流。它不纳入无运行库的默认 CTest，须设置同样的运行环境并显式传入库路径。通过不能代表真实帧布局、相机颜色或采集已经验证。

NUC 枚举、参数查询和 120 帧 dry-run 命令见 [application.md](application.md#海康-mvs-usb-输入)，验证记录项目见根目录 [questions.md](../questions.md)。不指定曝光、增益、尺寸、FPS 或像素格式时保留设备当前值；先保存 `--camera-info` 的实际报告，再按节点范围和标定配置调整。

## 离线回放

单张图片：

```bash
build/srm_auto_aim_app \
  --input image \
  --source sample.png \
  --dry-run \
  --intrinsics fx,fy,cx,cy \
  --debug-dir replay_debug \
  --log replay.csv \
  --summary replay_summary.txt
```

视频文件：

```bash
build/srm_auto_aim_app \
  --input video \
  --source sample.mp4 \
  --dry-run \
  --intrinsics fx,fy,cx,cy \
  --debug-dir replay_debug \
  --debug-every 5 \
  --log replay.csv \
  --summary replay_summary.txt
```

建议每次离线回放同时保留：

```text
原始图片/视频
配置文件
CSV 日志
summary 文件
debug 图像目录
```

这样可以在不连接下位机的情况下复现输入、检测结果、安全状态和耗时统计。离线回放中的 `dry_run=1` 表示 transport 未启动，不代表真实云台已经完成适配。

有桌面显示服务时，可在视频 dry-run 命令中添加 `--show-debug --start-paused`，查看检测叠加图和二值图。空格暂停/继续，`n` 单步，`q`/Esc 或关闭任一窗口退出；`--playback-fps 10` 可放慢播放。显示与 `--debug-every` 的保存频率独立，详细行为见 [application.md](application.md#可选窗口调试)。

## 运行输出检查

重点检查：

- `summary.txt` 中 `error_count`、`end_of_stream_count` 和耗时是否合理；
- `candidate_count` 是否与 debug 图像中的检测结果一致；
- `safety_state` 是否在目标丢失或无通信时回到 `safe`；
- `nonzero_fire_request_count` 是否为 `0`；
- CSV 中 `fire_flag` 是否全为 `0`；
- transport 计数和通信错误是否与运行模式一致；
- debug 图像是否能够定位灯条颜色、几何筛选和装甲板配对问题。

## 现场接入前的配置原则

- 先使用 `--dry-run` 和离线数据确认路径、权限、日志目录和配置文件无误；
- 现场设备路径、相机参数、内参、外参、弹速范围和延迟参数必须以实际确认结果为准；
- 不得把参考项目的 `SP`、CRC16、四元数或旧字段加入当前 CDC 配置；
- 不得通过修改配置文件绕过 `SafetyController` 或开启未经现场许可的自动发射；
- 真实设备接入顺序、安全前提和发射条件统一按 `questions.md` 执行。

## 交付前清单

### 软件文件

- `build/srm_auto_aim_app`；
- 与现场运行参数对应的 `key=value` 配置文件；
- `docs/` 中的应用、协议、架构、工作流和本部署文档；
- `questions.md`；
- 离线回放样本和对应 CSV/summary/debug 输出（如有）。

### 自动化结果

- CMake 配置成功；
- 编译成功；
- CTest 全部通过；
- `srm_auto_aim_app --help` 正常；
- 至少一次 dry-run 离线 smoke test 成功；
- 默认和异常路径确认 `fire_flag=0`。

### 现场结果

现场 NUC、MVS、CDC、云台、标定和发射结果不能由本地测试代替，必须在 `questions.md` 对应项目中逐项记录实际命令、日志、参数、现象和安全前提。
