# 部署与交付检查

## 适用范围

本文档描述当前软件主体在开发机或现场 NUC 上的构建、离线验收和运行入口使用方式。它不替代根目录 `questions.md` 中的现场安全、相机、CDC、标定、云台和发射测试清单。

当前软件交付边界：

- 支持图片、视频文件和 OpenCV camera index 输入；
- 支持正式自瞄 CDC 协议对应的 transport；
- 支持 `--dry-run` 离线运行；
- 默认不启用视觉开火，所有默认和异常路径保持 `fire_flag=0`；
- 尚未在代码中绑定海康 MVS SDK，也不把现场标定参数写死。

## 构建与自动化验收

在项目根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

CTest 至少应包含：

- 正式协议编解码和流式解析；
- 伪终端 transport 收发、拆包和重连；
- 图片/视频输入；
- 检测、PnP、选择、跟踪、补偿和安全控制；
- 应用配置、CSV、运行统计；
- `srm_auto_aim_app` 离线 smoke test。

当前离线 smoke test 使用 `tests/data/application_smoke.ppm`，由 CTest 自动验证：

- 程序能启动并处理一帧；
- debug 图像、CSV 和 summary 文件能够生成；
- summary 包含 `processed_frames=1`、`dry_run=1` 和 `nonzero_fire_request_count=0`；
- 应用入口没有因为检测不到目标而产生非零开火请求。

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
