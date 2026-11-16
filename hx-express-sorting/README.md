# hx-express-sorting 仿真环境

MuJoCo 3.3.7 快递分拣工作站：智元 G2 CRS、双 SwiftPicker、到货桌、侧置红绿按钮、传送带、四轮接收推车及 **7 件实体包裹**。保留真实尺寸、惯量、关节限位、有限力执行器、碰撞及标签纹理。塑料袋采用刚体近似，非柔性包装仿真。

## 入口与启动

在本目录执行（Python 3.10–3.12）：

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m http.server 8080 --bind 127.0.0.1
```

打开 http://127.0.0.1:8080/preview/index.html 查看环境。浏览器仅显示从主模型导出的初始状态，不播放历史轨迹，也不执行物理仿真。

实时物理查看器另开终端：

```sh
source .venv/bin/activate
python scripts/view_sim.py    # Linux
# macOS 使用：mjpython scripts/view_sim.py
```

原生查看器逐步运行 MuJoCo，保留物理按钮触发与传送带循环驱动。鼠标交互遵循 MuJoCo viewer；归档不包含自动分拣策略。

## 唯一主模型和生成

- `scenes/g2_operation.xml`：与源工作站逐字节一致的唯一操作模型。
- `assets/g2_description/`：上游 URDF/Xacro、碰撞网格及来源 SHA。
- `scenes/preview.json`：原始布局构建输入；内含旧布局机器人元数据，**不能用于证明当前机器人配置**。当前配置以 `g2_configuration.json` 和主 XML 为准。
- `scripts/build_mujoco.py`、`hxsim/workcell_assets.py`、`hxsim/conveyor_assets.py`：重建主模型及标签。
- `hxsim/runtime.py` / `workstation.py` / `mobile.py`：从源项目保留环境初始化、有限力步进、底盘和传送带/按钮逻辑；已去除任务规划、任务评分和 agent 接口。
- `scripts/export_environment.py`：从同一主模型重新导出首页初始几何。

```sh
python scripts/build_mujoco.py
python scripts/export_environment.py
python scripts/validate_environment.py
```

修改模型后重新构建和验证，不将旧任务结果作为新模型证据。构建器对上游两个不合法的主惯量做保迹修复，具体值见 `g2_configuration.json`；仿真可加载不等于实机标定。

## 来源与范围

归档源：`dashpulsar/hx-express-sorting` 工作目录，2026-11-16。模型与网格保持源文件；归档入口与依赖做了精简。G2 来源 `AgibotTech/genie_sim` 提交 `795b3fc77e4dd9f3ef972302ab90420c35450f62`，每个上游文件的路径和 SHA 见 `assets/g2_description/provenance.json`，MPL-2.0 文本见该目录 `LICENSE`。生成适配代码与修改说明随包提供。Three.js 许可见 `preview/vendor/LICENSE.three`。

不含 Harness、VLA/VLM、权重、任务轨迹、版本回放、视频、实验报告、凭据或远程部署。`ENVIRONMENT_MANIFEST.json` 登记本归档文件的 SHA-256；原模型的成功报告未移植。
