# 棋盘格内参标定、图像方向与云台外参

## 当前交付范围

`build/srm_calibrate_camera` 从图片目录或视频进行离线棋盘格标定，不访问相机或 CDC，不依赖窗口。输出完整 K、畸变、处理后分辨率/方向、总体及逐视图重投影误差。主程序通过 `--calibration` 加载，并把 K 和畸变直接传给 PnP。

目前没有经过确认的现场棋盘格规格、内参或外参数值。`build/calibration_tool_smoke/` 中的数据是自动化测试生成的合成数据，不能用于真实设备。

## 采集与运行

先记录真实板型、**每行内角点数（columns）**、**每列内角点数（rows）**及单个方格边长（毫米）。内角点数不是方格数；当前只接受 `chessboard`，不支持把圆点阵当棋盘格处理。每个方向至少 2、至多 100 个内角点，方格边长大于 0、至多 1000 mm。

采集期间固定镜头焦距、焦点、分辨率、ROI 和图像方向。让完整棋盘覆盖中心、四周、不同距离及不同倾斜角，避免过曝、拖影和大量近重复帧。至少 8 个成功视图是求解下限，不是标定充分性的保证。工具仅拒绝相对首视图平均角点位移始终不足 2 px 的明显重复数据，没有自动保证覆盖充分。

在项目根目录构建并查看帮助：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
build/srm_calibrate_camera --help
```

下面的变量须先填写实测值；`INPUT_VIDEO` 是完好且已经封口的输入视频，`NEW_OUTPUT_DIR` 是尚不存在、父目录已存在的结果目录：

```bash
build/srm_calibrate_camera --input video --source "$INPUT_VIDEO" \
  --output "$NEW_OUTPUT_DIR" --board chessboard \
  --columns "$INNER_COLUMNS" --rows "$INNER_ROWS" --square-size-mm "$SQUARE_MM"
```

图片目录使用 `--input images --source "$IMAGE_DIR"`，支持 PNG/JPG/JPEG/BMP/TIF/TIFF/PPM，按文件名字典序排列；建议零填充编号，并把标定原图放在独立目录，避免混入 mask、叠加图或不同尺寸的预览图。

| 选项 | 默认与行为 |
|---|---|
| `--every N` | 1；每 N 个输入采样一次。视频仍顺序解码，图片目录跳过未选文件 |
| `--max-views N` | 60；成功检测达到此数量即停止，至少为 8 |
| `--max-frames N` | 3000；检查的源输入数量上限，必须大于 0 |
| `--max-rms PX` | 0 表示未设置；正值超限时不发布 `camera.yml`，保留诊断 |
| `--file-rotation DEG` | 0；实际追加顺时针旋转，合法值为 0/90/180/270 |
| `--file-orientation DEG` | 默认读取 sidecar，无 sidecar 时假设 0；显式值仅声明已存像素方向，必须与已有 sidecar 一致 |

根据源视频和板的移动速度选择采样间隔；高 FPS 相邻帧可能几乎相同，不要为了凑够数量提前接受一段缺少倾斜/边缘覆盖的数据。`--max-rms` 是离线数据质量门限，由实际验证确定，不是开火阈值。

## 输出与验收

结果目录必须新建，重复使用会拒绝。输入视频打不开、板规格缺失或非法时会失败；不修改源文件。

| 文件 | 内容 |
|---|---|
| `sample_geometry.yml` | 第一个采样帧的尺寸和已处理方向，后续采样必须一致 |
| `detected_*.png` | 每个采样帧的角点诊断图，包含未检测成功的帧 |
| `views.csv` | 第一段为源帧、源时间、检测结果和视图索引；空行后第二段为逐视图 RMS |
| `undistorted_first.png` | 首个成功视图的去畸变检查图 |
| `camera.yml` | 成功求解并满足可选 RMS 上限后发布的候选标定 |

逐张确认角点定位和顺序，检查总体及逐视图 RMS，复查离群视图、图像覆盖和倾斜分布，再用未参与求解的图像验证去畸变及已知尺寸目标的 PnP 距离/角度。低 RMS 不能证明畸变可靠或外参正确；覆盖不足时高阶畸变系数也可能不稳定。工具不自动删除离群帧或决定现场精度是否合格。

失败返回码为 1，已创建的诊断目录保留。修正采样或输入后使用新结果目录。若源录像有解码/封口错误，应先取得完好原始数据，不以损坏文件推断板规格或现场内参。

## 方向与尺寸的一致性

方向相对原生传感器像素定义为顺时针 0/90/180/270 度；90/270 会交换宽高。输入层先完成 BGR 转换和配置的旋转，随后检测、PnP、预览与录像共用 `Frame.bgr`，不会只把窗口翻正。

| 输入情况 | 使用方式 |
|---|---|
| 实时相机需要旋转 | `--camera-rotation` / `camera.rotation_deg`，仅影响 MVS/OpenCV camera |
| 原生方向的旧文件需要转正 | `--file-rotation` / `input.rotation_deg`，仅影响图片/视频 |
| 旧文件像素已经旋转，但无元数据 | `--file-orientation` / `input.orientation_deg` 声明已有方向，不再次改像素 |
| 新录像自带 sidecar | 自动读取已有方向，文件额外旋转通常保持 0 |

例如经确认需要顺时针 180 度的安装：原生录像标定时使用 `--file-rotation 180`，运行相机使用 `--camera-rotation 180`；若录像本来由相机旋转后保存，则读取 sidecar 即可，不再追加 180 度。此例不代表当前现场已确认需要旋转。

新录像旁的 `<完整录像文件名>.metadata.yml` 必须随录像一起保存、复制或重命名，例如 `raw.avi.metadata.yml`。文件输入先核对 sidecar 中的存储尺寸，交付方向为“已有方向 + 文件额外旋转”模 360。显式已有方向与 sidecar 不一致会拒绝。没有 sidecar 时默认 0 只是软件假设，必须从采集记录核实。

`camera.yml`、外参文件与每帧交付的宽高/方向必须一致；不一致时报错，不自动缩放 K。180 度虽不交换宽高，仍会被方向检查区分。改变镜头、焦点、ROI、尺寸、方向或机械安装后需要重新验证并按需重标；尺寸/方向字段不能识别“同尺寸但换了镜头或 ROI”的变化，须保留采集配置。

## 标定文件格式与加载

使用 OpenCV FileStorage YAML（`%YAML:1.0`），当前 `schema_version` 必须为整数 1：

| 必需字段 | 类型及含义 |
|---|---|
| `image_width`、`image_height` | 正整数，处理后的像素尺寸 |
| `orientation_deg` | 整数 0/90/180/270，处理后的方向 |
| `camera_matrix` | `!!opencv-matrix` 的有限 3×3 单通道矩阵，正焦距、零 skew、末行为 `[0,0,1]` |
| `distortion_coefficients` | 显式数值序列，按 OpenCV 畸变系数顺序；缺字段不会默认为零 |

工具还写 `board_type`、`inner_columns`、`inner_rows`、`square_size_m`、`rms_px`、`view_rms_px` 以便追溯；当前常规求解输出五项畸变。外部文件的系数数量按现有 `ArmorPoseSolver` 的 OpenCV 模型校验。

离线加载示例（替换为实际新结果目录）：

```bash
build/srm_auto_aim_app --input video --source "$INPUT_VIDEO" --dry-run \
  --calibration "$NEW_OUTPUT_DIR/camera.yml" --summary replay_summary.txt
```

若标定时额外旋转了原生文件，回放同一原生文件也须提供相同 `--file-rotation`。四参数 `--intrinsics fx,fy,cx,cy` 保留用于旧的 dry-run 检查，与完整标定互斥；它没有畸变、尺寸或方向绑定，不能用于 `--live`。没有标定时只允许采集/检测诊断，不据默认值确认距离或角度。

## 相机到云台外参

棋盘格单独录像提供的是相机与棋盘间的观测，无法单独求出相机到云台的 R/t。参考手眼工具还要求图像、对应姿态及正确采集模型，本工具不伪造这些数据。

本项目使用：

```text
p_gimbal = rotation_camera_to_gimbal * p_camera + translation_camera_to_gimbal
```

`p_camera` 在**处理后**的 OpenCV 相机坐标中：x 向图像右、y 向图像下、z 向前；云台坐标 x 向前、y 向左、z 向上。平移单位为米，表示相机原点在云台坐标中的位置。不能把相反方向的变换直接填入，也不能只改文件方向标签来复用旋转前的 R/t。

外参文件同样必须含 `schema_version: 1`、`image_width`、`image_height`、`orientation_deg`，并与完整内参一致，另有：

| 字段 | 必须提供的内容 |
|---|---|
| `camera_frame` | 字符串 `opencv_processed` |
| `gimbal_frame` | 字符串 `x_forward_y_left_z_up` |
| `translation_unit` | 字符串 `m` |
| `rotation_camera_to_gimbal` | `!!opencv-matrix` 的有限 3×3 旋转矩阵，满足正交性及行列式 +1 |
| `translation_camera_to_gimbal` | `[tx, ty, tz]`，三个有限数，单位米 |

使用 `--extrinsics PATH` / `camera.extrinsics` 加载；必须同时给完整 `--calibration`。软件能拒绝非法矩阵、反射、错误单位/坐标声明和几何不符，不能证明合法数值来自真实测量。默认轴映射只供离线诊断。

现场需先明确云台原点、轴向、角度正负及图像/下位机姿态配对关系，再决定测量或手眼求解方式，保留独立静态目标验证数据；具体待办见 [questions.md](../questions.md)。`--live` 在完整内外参加载后才能启动 CDC，仍保持视觉开火关闭。
