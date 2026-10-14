# 相机预览倒置排查与安装方向配置

日期：2026-10-14

## 问题与根因

用户报告真实相机调试窗口仍上下倒置，并授权通过 SSH 检查 NUC；随后明确所有代码修改在笔记本，NUC 项目只用于运行。

检查笔记本和 NUC `/home/wuming/wuming/shu_auto_aim_project`：两端 `run.conf` 的 `camera.rotation_deg` 都为 0。NUC 可执行文件帮助已包含 `--camera-rotation`，输入源代码包含旋转调用，因此不是缺少旋转功能的旧版本。启动脚本传递配置及末尾参数，未覆盖相机方向；调试窗口只添加信息栏并调用 imshow，未额外翻转像素。已有方向功能保持默认原始方向，没有启用针对倒置画面的安装校正。

取回既有 `frame_009000.png`（1440×1080）供检查。最新 `frame_009100.png` 无法完整解码，未据此判断方向或修改原图。

## 修改与运行验证

仅修改笔记本 `run.conf`：`camera.rotation_deg=180`，补充安装方向注释。文件旋转仍为 0。旋转在输入层执行，使检测、PnP、mask、窗口和新录像使用一致方向。未修改 NUC 源码、配置或程序，未改 SDK 和相机节点。

本机运行：

```bash
ctest --test-dir build -R '^(srm_launcher_smoke_test|srm_calibration_test|srm_recording_app_smoke_test)$' --output-on-failure
```

结果：3/3 通过，覆盖脚本、几何旋转和录像回放相关行为。此次仅改安装配置，没有修改 C++ 代码。

在 NUC 的独立临时目录 `/tmp/srm-orientation-f4qC8R` 中运行：

```bash
bash /home/wuming/wuming/shu_auto_aim_project/run.sh mvs --dry-run --camera-rotation 180 --max-frames 3 --no-show-debug --no-record --debug-every 1 --debug-dir /tmp/srm-orientation-f4qC8R/images --log /tmp/srm-orientation-f4qC8R/run.csv --summary /tmp/srm-orientation-f4qC8R/summary.txt
```

实际目录通过 `mktemp -d /tmp/srm-orientation-XXXXXX` 创建，未覆盖原输出。相机 MV-CS016-10UC / DA1830801，SDK 4.7.0.3，读到 3 帧 1440×1080，`orientation_deg=180`、`error_count=0`、`safe_count=3`、非零开火请求和 transport 收发均为 0，退出成功。SDK 输出 `XOpenDisplay Fail`，本次关闭 GUI，采集及保存仍成功；不据此声称桌面窗口已验证。

## 使用及边界

NUC 当前文件未被修改，运行时应显式加参数：

```bash
bash run.sh mvs --dry-run --show-debug --camera-rotation 180
```

已验证参数与真实采集路径生效，最终视觉方向仍需用户在桌面确认；若“上下颠倒”实际指仅垂直镜像而非整体倒转，应根据有方向标记的图像进一步区分，不能将镜像当作旋转。相机后续机械安装改变时重新确认安装方向。新的标定应绑定处理后方向 180；旧方向内外参不可通过改标签直接复用，旧录像也不会因相机设置而自动旋转。
