# ICV-SIM

智能网联车离线规划、控制与仿真

本项目面向 NEVC 智能网联车竞赛的前期开发与算法验证。在官方平台尚未开放、暂时无法登录联调的情况下，以 Python 场景工具搭建本地闭环仿真，并使用 **CMake + Visual Studio 2022 构建 C++ 控制核心和 SimOne SDK 接入程序**。

项目将场景工具、C++ 控制算法和网页展示分开：离线场景可分别由原 Python 算法和编译出的 C++ 控制核心闭环运行，浏览器支持对照两套轨迹；C++ 同时提供读取 SimOne GPS/障碍物与下发车辆控制指令的 SDK 接入程序。

- [完整 HTML 技术说明：文件结构、函数、控制公式、CMake/Visual Studio 使用和队友交接](README.html)（下载仓库后用浏览器打开）
- [本地运行结果汇总](output/RESULTS.md)
- [泊车动画](output/parking.gif) / [城市道路动画](output/city.gif)
- 下载或克隆仓库后，双击 `web/index.html` 查看交互回放，双击 `README.html` 阅读浏览器版详细说明。GitHub 文件预览页面不会直接运行 HTML。

## 功能与场景

根据现有竞赛资料中的四类示例任务，构建信号灯通行、绕障起步、垂直泊车和城市道路场景，并增加车道保持、紧急制动、跟车和行人横穿测试。信号灯包含红转绿和全绿两种情况，共九个测试用例。

- **信号灯通行**：红灯前减速停车，绿灯后恢复行驶。
- **绕障起步**：规划平滑变道路线，绕过起点前方障碍物。
- **垂直泊车**：生成包含前进、倒车的路径，完成入位、停留和驶出。
- **城市道路**：沿参考路线行驶，结合限速、信号灯和障碍物进行控制。
- **安全测试**：验证车道保持、紧急制动、前车跟随及行人避让行为。

项目支持导出 CSV 轨迹、JSON 指标报告和 GIF 动画，并提供无需登录即可打开的网页回放界面。

## 车辆如何运动

当前实现**不使用 MPC**，采用以下方案：

| 模块 | 实现方法 |
|---|---|
| 路径规划 | 直线路径、五次多项式变道、Reeds–Shepp 泊车路径 |
| 横向控制 | Pure Pursuit（纯追踪），计算前轮转角 |
| 纵向控制 | P（比例）速度控制，计算加速度 |
| 行为决策 | 基于规则的停车、限速、跟车、紧急制动和泊车阶段切换 |
| 车辆模型 | 运动学自行车模型，通过 SciPy RK45 积分更新状态 |

默认每隔 **0.05 秒**执行一次控制循环：

**读取车辆与场景状态 → 更新行为决策 → 计算转向和加速度 → 更新车辆运动 → 检查碰撞与任务状态 → 保存记录。**

## 代码结构

- `run_offline.py`：离线仿真入口，执行场景并保存结果。
- `icv/model.py`：车辆参数、状态、车身轮廓与运动模型。
- `icv/planning.py`：参考路线、变道与泊车路径规划。
- `icv/controller.py`：转向、速度、跟车和紧急制动控制。
- `icv/scenarios.py`：道路、信号灯、障碍物及任务配置。
- `icv/simulation.py`：闭环仿真、任务切换、指标统计与本地检查。
- `CMakeLists.txt`、`CMakePresets.json`：使用 CMake GUI 配置 VS2022、C++ 测试和 NEVC SDK 链接。
- `cpp/`：C++ 控制核心、Python 对照 DLL、原生测试及 SimOne SDK 接入程序。
- `web/`：交互式轨迹回放页面。
- `output/`：轨迹、报告、动画和验证截图。
- `tests/`：安全检查与回归测试。
- `tools/`：动画导出、说明生成和验证工具。
- `vendor/`：第三方规划算法及其来源、许可证。
- `nevc_bridge.py`、`config/`：待正式联调的 NEVC 基础接口适配与配置。

详细文件说明、控制公式、参数位置和操作方法统一放在 [README.html](README.html)，下载仓库后用浏览器打开。

## 运行方式

Windows 已安装 CMake 和 Visual Studio 2022 的电脑可从仓库根目录运行：

```bash
git clone https://github.com/chenqianairuisi-stack/ICV-SIM.git
cd ICV-SIM
cmake --preset vs2022
cmake --build --preset vs2022-release --parallel 4
ctest --preset vs2022-release
```

`open_cmake_gui.bat` 可用 CMake GUI 配置 SDK 工程，`open_visual_studio.bat` 打开生成的 Visual Studio solution。无本机 NEVC SDK 时用 `vs2022-core` preset 构建独立离线算法核心。

安装依赖并运行全部场景：

```bash
python -m pip install -r requirements.txt
python run_offline.py
```

运行测试：

```bash
python -m unittest discover -s tests -v
```

仅 Python 离线运行时，用浏览器打开 `web/index.html` 查看回放。Windows 用户也可使用 `cmake --build build/vs2022 --config Release --target run_cpp_offline` 运行 C++ 后端，再在同一页面切换对比。

## 当前进展与边界

目前九个场景均通过已设置的本地检查，并完成默认控制周期及更小控制周期的复核。

这些结果来自自建离线环境，**不代表 NEVC 官方测试或比赛成绩**。C++ SimOne SDK 可执行程序已用本机 SDK 3.0.0001 编译并通过无连接的版本探测，仍须平台开放后确认官方运行环境、SDK 版本、传感器名称、转向正负号和控制量标定。程序读取已配置的 GPS、感知障碍物/交通灯，发送转向、油门/制动、档位和灯光指令；官方联调尚未进行。离线场景使用已知地图、参考路线和理想状态信息，车辆采用运动学模型，未覆盖轮胎侧滑等复杂动力学。现有资料提供的是部分任务示例，也不能据此认定覆盖全部正式赛题。

后续将在官方平台开放后，完成接口标定、车辆控制适配和真实比赛场景验证，并根据任务需求进一步评估 MPC 等控制方法。
