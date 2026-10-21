# ICV-SIM

智能网联车离线规划、控制与仿真

本项目面向 NEVC 智能网联车竞赛的前期开发与算法验证。由于官方平台暂未开放，项目目前使用本地离线场景进行控制算法和车辆运动仿真，并通过网页查看仿真回放。

项目同时提供 Python 和 C++ 两种离线运行后端。C++ 控制核心使用 **CMake 配置、Visual Studio 2022 编译**；项目还包含一个 SimOne SDK 接入程序，用于后续开展官方平台联调。

> 当前所有仿真结果均来自自建离线场景，不代表 NEVC 官方测试结果或比赛成绩。SimOne SDK 程序尚未连接官方比赛平台，实际车辆控制与接口标定仍待平台开放后验证。

## 查看仿真效果

克隆项目后，直接用浏览器打开 `web/index.html`。点击页面顶部的 **Python / C++** 切换按钮，可以查看两种后端生成的轨迹。

```bash
git clone https://github.com/chenqianairuisi-stack/ICV-SIM.git
cd ICV-SIM
```

已经生成的回放数据随项目提供，**只查看仿真效果不需要先安装 CMake、Visual Studio 或 Python**。

- [交互式仿真回放](web/index.html)
- [详细技术说明：代码结构、控制方法、软件操作和队友交接](README.html)
- [本地运行结果](output/RESULTS.md)
- [泊车动画](output/parking.gif) · [城市道路动画](output/city.gif)

GitHub 文件预览页面不会直接运行交互式网页。下载或克隆仓库后，请在本地浏览器打开 `web/index.html`。

## 仿真场景

项目根据现有竞赛资料中的任务示例搭建了九个离线测试场景：

| 场景 | 内容 |
|---|---|
| 信号灯 / 红转绿 | 红灯前停车，绿灯后起步通行 |
| 信号灯 / 绿灯通行 | 绿灯状态下沿路线通过 |
| 前方障碍物起步 | 规划绕行路线并通过障碍物 |
| 垂直泊车 | 倒车入位、停车保持后驶出 |
| 城市连续道路 | 沿参考路线行驶，并处理限速、信号灯和障碍物 |
| 弯道车道保持 | 检查弯道上的路径跟踪 |
| 紧急制动 | 检查接近静止障碍物时的制动行为 |
| 跟车 | 检查与前车保持距离的行为 |
| 行人横穿 | 检查行人横穿时的减速和让行行为 |

每个场景都会导出轨迹和指标数据。C++ 后端生成的数据保存在 `output/cpp/`，Python 后端的数据保存在 `output/`。

## 控制方法

当前控制方案**不是 MPC**：

- **横向控制**：Pure Pursuit（纯追踪），根据参考路线计算转向。
- **纵向控制**：P（比例）速度控制，根据目标车速计算加速或制动指令。
- **行为决策**：使用规则处理停车、限速、跟车、紧急制动和泊车阶段切换。
- **车辆运动**：使用运动学自行车模型更新车辆状态。Python 后端使用 SciPy RK45 积分；C++ 后端使用 RK4 积分。

离线仿真的闭环流程如下：

```text
读取车辆与场景状态
        ↓
行为规则选择目标车速和行驶阶段
        ↓
Pure Pursuit 计算转向，P 控制计算加速度
        ↓
车辆模型更新位置、速度和朝向
        ↓
检查碰撞、道路边界和任务完成情况
        ↓
保存轨迹与指标
```

## 使用 CMake GUI 和 Visual Studio 2022

CMake 用于配置项目并生成 Visual Studio 工程，Visual Studio 2022 用于查看、编译和调试 C++ 代码。

### 有本机 SimOne SDK

1. 确认已安装 CMake、Visual Studio 2022 和 NEVC 提供的 SimOne SDK。
2. 克隆仓库后，在项目根目录运行 `open_cmake_gui.bat`，或执行：

   ```powershell
   cmake --preset vs2022
   ```

3. CMake GUI 配置完成后，运行 `open_visual_studio.bat` 打开生成的工程；也可以在命令行构建和运行测试：

   ```powershell
   cmake --build --preset vs2022-release --parallel 4
   ctest --preset vs2022-release
   ```

`vs2022` 预设默认将 SDK 路径设为 `E:/Sim-One/SimOneAPI`。如果 SDK 安装在其他目录，请在 CMake GUI 中将 `SIMONE_SDK_ROOT` 改为本机路径。

### 没有 SimOne SDK

选择 `vs2022-core` 预设可以构建 C++ 离线控制核心和测试，不需要 SimOne SDK：

```powershell
cmake --preset vs2022-core
cmake --build --preset vs2022-core-release --parallel 4
ctest --preset vs2022-core-release
```

## 重新运行仿真

安装 Python 依赖后，可以运行 Python 离线仿真：

```powershell
python -m pip install -r requirements.txt
python run_offline.py
```

如果已用 CMake 构建 C++ 后端，可重新计算 C++ 的九个场景：

```powershell
cmake --build build/vs2022 --config Release --target run_cpp_offline
```

运行后打开 `web/index.html`，即可查看更新后的轨迹。运行测试：

```powershell
python -m unittest discover -s tests -v
```

## 主要文件

```text
ICV-SIM/
├── CMakeLists.txt          # C++ 工程、测试和可选 SimOne SDK 程序的构建配置
├── CMakePresets.json       # Visual Studio 2022 和离线核心的 CMake 预设
├── cpp/                    # C++ 控制核心、Python 调用接口、测试和 SDK 接入程序
├── icv/                    # Python 车辆模型、路径规划、控制器和仿真逻辑
├── web/                    # 交互式仿真回放页面及 Python/C++ 轨迹数据
├── output/                 # Python 结果、报告、动画和验证截图
│   └── cpp/                # C++ 后端生成的 CSV 轨迹和 JSON 报告
├── tests/                  # Python 安全检查及 C++ 后端对照测试
├── tools/                  # 结果整理、网页验证和交付检查工具
├── open_cmake_gui.bat      # 启动 CMake GUI
├── open_visual_studio.bat  # 打开生成的 Visual Studio 2022 工程
├── run_offline.py          # Python 离线仿真入口
└── README.html             # 完整技术说明和操作指南
```

## 当前验证情况与限制

本机已使用 Visual Studio 2022 编译 C++ 控制核心和 SimOne SDK 接入程序；C++ 的 CTest 测试、Python 测试及网页回放检查均已通过。C++ SDK 程序通过了本机 SDK 的版本探测，但这不等于已连接或控制 NEVC 官方模拟车辆。

目前仍需在官方平台开放后完成以下工作：

- 核对竞赛手册、正式题目和官方运行环境。
- 确认官方 SDK 版本、车辆控制接口、传感器名称和数据格式。
- 标定转向方向与比例、油门和制动量、挡位及灯光指令。
- 在官方场景中逐项联调，并按官方规则检查和记录结果。

离线场景使用预先设置的地图、路线及理想化状态信息，车辆采用简化运动学模型。现有场景用于算法开发和流程验证，不能说明已经覆盖全部正式赛题，也不能替代官方平台测试。

## 许可证与第三方代码

第三方代码、资源及其许可证信息见 `vendor/` 和对应目录说明。使用和分发项目内容时，请遵循各自许可证。
