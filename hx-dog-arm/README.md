# hx-dog-arm 仿真环境

本项目使用 **Gazebo Harmonic（gz-sim8）**，并非 MuJoCo。保留五个主线环境、物理交互插件和原创轮腿机械臂代理，不包含 agent 或实验框架。

| 环境 | 内容 |
| --- | --- |
| `01_grid_baseline` | 基准走廊、两端上下排电柜与按钮 |
| `02_outdoor_switchyard` | 日照、碎石、防雨柜、遮阴 |
| `03_utility_tunnel` | 台阶、坡道、管线与遮挡 |
| `04_access_control_rooms` | 双机房、实体卡、读卡器、门锁与门把手 |
| `05_rotary_thermal_lab` | 旋钮解锁、按钮与动态温度枪 |

各场景包含 SDF、元数据、地图、俯视 SVG 和尺寸说明；每场景 8 柜，初始 4 红 4 绿。SDF 不依赖远程模型下载。插件保留按压驻留、旋转解锁、刷卡/门锁状态与可视反馈。温度显示不等于已实现测温操作策略。

## 构建和启动

安装 Gazebo Harmonic、CMake、C++17 编译器和 Python 3。可使用已配置 ROS 2 Jazzy / Gazebo Harmonic 的 Linux 开发环境，但本归档无需启动 ROS、Nav2 或模型服务。

在本目录执行，CMake 需要能找到 `gz-sim8`、`gz-plugin2`、`gz-transport13` 开发包：

```sh
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 tools/view_environment.py 01_grid_baseline
# 无窗口：
python3 tools/view_environment.py 01_grid_baseline --headless
```

启动器只把归档机器人加入原世界、按 `scene.json` 设置出生点，在临时目录启动；不改写主场景。其他场景替换参数即可。原世界与机器人可分别用 `gz sdf -k <文件>` 验证。

没有 Gazebo 时可进行离线检查：

```sh
python3 tools/validate_environment.py
cmake -S . -B build -DBUILD_GAZEBO_PLUGIN=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

## 模型与来源

`robots/wheel_legged_proxy` 为原创基础几何代理：固定支腿、四轮、升降/伸缩操作机构、旋转腕、双指夹爪、RGB、激光与 IMU。参数为工程占位，不是 M20s 数字孪生，不能据此验证 M20s 越障或真实控制性能。

五个世界、地图、机器人 SDF/URDF 与交互插件保留源项目主线文件；不混入实验目录中的修改版世界、可动腿或优化控制配置。生成器 `tools/generate_environments.py` 保留用于复现几何；机器人生成源码位于 `src/hx_grid_sim/hx_grid_sim/robot.py`。

归档来源：`hx-dog-arm` 主线工作目录，2026-11-16。`ENVIRONMENT_MANIFEST.json` 登记文件 SHA-256。仅为环境建档，不移植历史实验成绩。不含 Qwen/VLA、Harness、导航/操作策略、模型权重、远程部署、凭据、日志、视频或实验报告。

本次在 macOS 完成 SDF/JSON 结构、生成一致性、机器人装配及 C++ 交互状态机检查。因本机未安装 Gazebo，**未验证插件链接、Gazebo 启动和完整动态交互**；需在 Harmonic 环境执行上述构建与启动检查。
