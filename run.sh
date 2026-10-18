#!/usr/bin/env bash
# 快捷入口；长期参数放在 run.conf，临时参数直接追加到命令后。
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'HELP'
用法：bash run.sh [模式] [应用参数...]

  run        按 run.conf 运行（省略模式时相同）
  video      视频回放，路径取配置或 --source PATH
  image      图片回放，需要配置图片路径或 --source PATH
  mvs        海康 USB 采集，可用 --show-debug 实时预览
  camera     OpenCV camera index 采集，可用 --show-debug 实时预览
  list       枚举 MVS USB 设备后退出
  info       查询 MVS 设备参数后退出，不修改节点或取流
  app-help   显示原程序的完整参数
  --help     显示本说明

示例：
  bash run.sh
  bash run.sh video --show-debug --start-paused
  bash run.sh image --source "my images/target.png"
  bash run.sh list
  bash run.sh info --mvs-serial ACTUAL_SERIAL
  bash run.sh mvs --max-frames 120
  bash run.sh mvs --live --show-debug
  bash run.sh mvs --show-debug --record replay_debug/session-01.avi
  bash run.sh --config my-settings.conf

默认配置：项目根目录 run.conf（离线视频、dry-run、运行到 EOF）。
--config PATH 替换默认配置；末尾应用参数覆盖配置和模式预设。
所有相对路径均相对于项目根目录，含空格路径请加引号。
默认输出 replay_debug/latest/，重复运行会覆盖同名文件。

环境变量：SRM_APP 指定可执行文件，默认 build/srm_auto_aim_app。
          MVS_ROOT 指定 SDK 根目录，默认 /opt/MVS。
详细说明：docs/launcher.md
HELP
}

mode=run
if (($#)); then
  case "$1" in
    -h|--help) usage; exit 0 ;;
    run|video|image|mvs|camera|list|info|app-help) mode="$1"; shift ;;
    --*) ;;  # 不写模式，直接传入应用选项。
    *) printf '未知模式：%s；使用 bash run.sh --help 查看用法。\n' "$1" >&2; exit 2 ;;
  esac
fi

# 固定相对路径基准，不依赖调用者所在目录。
cd -- "$ROOT"
app="${SRM_APP:-$ROOT/build/srm_auto_aim_app}"
if [[ ! -x "$app" ]]; then
  printf '找不到可执行文件：%s\n请先在项目根目录执行：\n' "$app" >&2
  printf '  cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug\n  cmake --build build -j2\n' >&2
  exit 1
fi
if [[ "$mode" == app-help ]]; then
  exec "$app" --help
fi

# 仅为本次进程设置已有 SDK 的模块环境；保留调用者显式环境设置。
mvs_root="${MVS_ROOT:-/opt/MVS}"
if [[ -d "$mvs_root/lib/64" ]]; then
  mvs_root="$(cd -- "$mvs_root" && pwd)"
  export MVCAM_COMMON_RUNENV="${MVCAM_COMMON_RUNENV:-$mvs_root/lib}"
  export LD_LIBRARY_PATH="$mvs_root/lib/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

config_args=(--config "$ROOT/run.conf")
for argument in "$@"; do
  if [[ "$argument" == --config ]]; then
    config_args=()
    break
  fi
done

preset=()
case "$mode" in
  video|image) preset=(--input "$mode") ;;
  mvs|camera)
    preset=(--input "$mode" --source "" --no-start-paused --playback-fps 0)
    ;;
  list) preset=(--list-cameras) ;;
  info) preset=(--camera-info) ;;
esac

# 数组保留带空格参数；exec 让 Ctrl+C 和退出码直接交给原程序。
exec "$app" "${config_args[@]}" "${preset[@]}" "$@"
