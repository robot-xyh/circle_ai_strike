<!-- toc -->

# 构建开发调试环境

## 构建基础环境

所有环境基于`Ubuntu 22.04`操作系统。

由于采用的开发板`Orange Pi 5 Max`，目前为止[官网](http://www.orangepi.cn/html/hardWare/computerAndMicrocontrollers/service-and-support/Orange-Pi-5-Max.html)显示的操作系统只支持到`Ubuntu 22.04`。所以本项目的基础环境也基于`Ubuntu 22.04`操作系统。

你有两种选择来构建本项目的基础环境:

- [Ubuntu Native or WSL2](#Ubuntu-native-or-wsl2)
- [DevContainer](#devcontainer)

如果你使用`Ubuntu Native or WSL2`, 跳转到[Ubuntu Native or WSL2](#Ubuntu-native-or-wsl2)。

如果你使用`DevContainer`, 跳转到[DevContainer](#devcontainer)。

### Ubuntu Native or WSL2

#### 安装 ROS2 Humble

参考[官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)文档进行安装。

> [!NOTE]
> 完成上述安装后，你可以参考[PX4 Gazebo Simulation](./px4-gazebo-simulation.md)文档，在`Ubuntu Native`或`WSL2`中同时安装仿真飞行器部分。
> 同时，你可以参考[安装`QGroundControl`](./install-qgroundcontrol.md)文档，在`Ubuntu Native`或`WSL2`中安装`QGroundControl`。
> 这样你可以在同一个环境中进行仿真与开发。


完成上述过程后，还需要通过系统包管理安装以下软件包：

```bash
sudo apt-get update

```

#### 安装系统依赖包

```bash
sudo apt-get install -y --no-install-recommends build-essential \
git \
sudo \
cmake \
python3-pip \
python3-vcstool \
python3-colcon-common-extensions \
ros-humble-ament-cmake \
ros-humble-aruco-opencv \
ros-humble-cv-bridge \
ros-humble-usb-cam \
ros-humble-image-view \
ros-humble-image-transport \
ros-humble-rqt-image-view \
ros-humble-image-view \
ros-humble-plotjuggler \
ros-humble-plotjuggler-ros
```

#### 安装Python依赖包

我们推荐使用`uv`来安装Python依赖包：

安装`uv`：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

安装Python工具：

```bash
uv tool install ruff@latest && \
uv tool install kconfiglib@latest && \
uv tool install clang-tidy@latest && \
uv tool install clang-format@latest && \
uv tool install cmakelang@latest && \
uv tool install prek@latest && \
uv tool install commitlint@latest
```

安装`Gazebo`依赖包（仿真）：

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update

sudo apt install -y ros-humble-ros-gzharmonic-bridge
```

#### 切换 ROS2 RMW 为 CycloneDDS（强烈建议）

ROS2 Humble 默认的 RMW 实现是 `rmw_fastrtps_cpp`（Fast-DDS）。在**单机收发大消息**（典型如相机
`sensor_msgs/Image` 1MB 量级，30Hz）场景下，Fast-DDS 会出现明显的吞吐瓶颈与抖动。本项目实测：

| RMW | `gz topic -hz` (Gazebo 原生) | `ros2 topic hz /top/image_raw` |
| --- | --- | --- |
| `rmw_fastrtps_cpp`（默认） | ~30 Hz | **5–7 Hz** |
| `rmw_cyclonedds_cpp` | ~30 Hz | **21–25 Hz** |

ROS 侧实际帧率直接 ×3–4，并且 jitter / `max interval` 也明显下降。这会直接影响 YOLO 检测帧率与
`vision_tracking_rates_ctrl` 控制环的反馈速率。**所有运行 ROS2 的环境**（仿真机 / 真机 / DevContainer）
都建议切换。

安装：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp
```

设置环境变量（`bash` 与 `zsh` 都要写一份）：

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.zshrc
# 当前 shell 立即生效
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

验证：

```bash
echo $RMW_IMPLEMENTATION   # 应为 rmw_cyclonedds_cpp
ros2 doctor --report 2>/dev/null | grep -i "middleware name"  # 应为 rmw_cyclonedds_cpp
```

> [!IMPORTANT]
> 切换 RMW 后，**所有 ROS 节点必须使用同一个 RMW** 才能互相发现。务必：
> 1. 在所有相关 shell（`launch` / `topic hz` / `yolo` / 调试节点）都 export 同一个值；
> 2. 重启所有正在运行的 ROS2 节点（`pkill -9 -f "ros2 launch"`、关掉旧的 `ros2 topic hz` 等）；
> 3. 重启 Gazebo（不需要重装，但 `ros_gz_bridge` 需要重启以走新的 RMW）。

> [!NOTE]
> **真机部署也需要安装** `ros-humble-rmw-cyclonedds-cpp` 并设置 `RMW_IMPLEMENTATION`。
> 真机同样存在大消息（USB 相机 → YOLO → 调试链路）单机回环的瓶颈；同时 Humble 版 Fast-DDS 的
> 共享内存 transport 在某些 kernel 上偶发卡顿，CycloneDDS 走 UDP localhost 默认更稳。
> 真机的开发板（如 Orange Pi 5 Max）资源更紧张，CycloneDDS 的提升相对更明显。

> [!NOTE]
> 如果你需要跨主机通讯（地面站 ↔ 飞机 / 多飞机），CycloneDDS 也可以工作。需要时可在
> `~/.cyclonedds.xml` 里配置 `NetworkInterfaceAddress` / `Domain` 等；本项目目前的单机
> SITL 与单机真机场景**直接默认配置即可**，无需额外 XML。

### DevContainer

你也可以使用[DevContainer](https://code.visualstudio.com/docs/devcontainers/containers)来创建本项目的基础环境。


本项目针对`x86_64`架构和`aarch64`架构提供了`DevContainer`配置，你可以直接使用`Cursor IDE`打开本项目，然后点击`Remote-Containers: Reopen in Container`来启动`DevContainer`。

> [!NOTE]
> 你需要安装[Docker](https://docs.docker.com/get-docker/)来使用DevContainer。
> 如果你想在`x86_64`架构的设备上使用`aarch64`架构的构建，你还需要安装[QEMU](https://www.qemu.org/download/)。

> [!NOTE]
> DevContainer环境默认没有仿真飞行器的部分，你需要手动安装。请参考[PX4 Gazebo Simulation](./px4-gazebo-simulation.md)文档进行安装。

> [!NOTE]
> 本项目默认`DevContainer`针对`Cursor IDE`进行配置，如果你使用`VSCode`，有部分配置需要手动调整。

## 项目源代码

通过项目仓库克隆源代码到基础环境即可。

> [!NOTE]
> 通常你应该保持项目仓库的目录结构为`path/to/ws/src/circle_pilot`。
> ```bash
> mkdir -p ~/ws/src
> cd ~/ws/src
> git clone ssh://git@git.circleai.tech:8222/drone/circle_pilot.git
> ```
> `DevContainer`环境需要你先克隆代码然后启动`DevContainer`，会在创建时自动完成本步骤，无需手动执行。

## 安装项目依赖

### 安装第三方代码库
```bash
cd circle_pilot
mkdir -p third
vcs import third < circle_pilot.repos
# For PX4 Autopilot simulation
vcs import third < px4.repos
```

> [!NOTE]
> 仅需要在`Ubuntu Native`或`WSL2`执行本步骤安装依赖。
> `DevContainer`环境会在创建时自动完成本步骤，无需手动执行。

### 安装pytorch (GPU version)

#### NVIDIA（CUDA）

先确认本机 CUDA 版本（`nvidia-smi` 右上角），再选择对应索引：

- **CUDA 11.8**: `cu118`
- **CUDA 12.1**: `cu121`
- **CUDA 12.4**: `cu124`

```bash
# 示例：CUDA 12.8
python3 -m pip install torch --index-url https://download.pytorch.org/whl/cu128
```

其他版本将 `cu121` 替换为 `cu118` 或 `cu124` 等。安装后可用 `python3 -c "import torch; print(torch.cuda.is_available())"` 验证。

#### Intel Arc（XPU）

Intel Arc **不使用 CUDA**，不要选用 `cu118` / `cu121` 等索引；需安装带 **XPU** 后端的官方轮子。请先按系统装好 Intel GPU 驱动及 PyTorch 文档要求的运行库（如 Level Zero等），详见 [Getting Started on Intel GPU](https://docs.pytorch.org/docs/stable/notes/get_start_xpu.html)。

稳定版：

```bash
python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/xpu
```

预览版（nightly）：

```bash
python3 -m pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/xpu
```

验证应使用 **XPU**（不是 `cuda`）：

```bash
python3 -c "import torch; print(torch.xpu.is_available())"
```

业务代码中请将 `.cuda()` / `"cuda"` 改为 `.to("xpu")` / `"xpu"`。

#### GPU 设备权限（NVIDIA / Intel Arc）

若无法访问 GPU（如 `/dev/dri`），可将当前用户加入 `render` 与 `video` 组（NVIDIA 与 Intel Arc 均可能需要）：

```bash
sudo usermod -aG render,video $USER
```

执行后需重新登录或新开会话使组权限生效。

### 安装 `yolo_ros` 第三方Python库的依赖


```bash
cd circle_pilot/third/yolo_ros
python3 -m pip install -r requirements.txt
```

特别注意：若numpy是2.x.x版本，需要降级：
```bash
python3 -m pip install "numpy<2" --user
```

#### 安装pytorch (CPU version)
```bash
python3 -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```


## 构建`PX4 Autopilot`

如果你需要仿真运行或者编译固件，你需要先构建`PX4 Autopilot`。

> [!NOTE]
> 如果只是编译`ROS2`相关项目，并且连接真机运行，则不需要构建`PX4 Autopilot`。

```bash
cd circle_pilot/third_party/PX4-Autopilot
source ./Tools/setup/ubuntu.sh
sudo apt update && sudo apt upgrade -y
make
```

## 构建本项目

当你完成基础环境构建后（不论是`Ubuntu Native`或`WSL2`，还是`DevContainer`），你可以执行以下命令来构建本项目：

- 切换到对应目录
```bash
cd /path/to/ws
```

- 引用ros的相关配置

**bash**
```bash
source /opt/ros/humble/setup.bash
```

**zsh**
```zsh
source /opt/ros/humble/setup.zsh
```

- 开始构建

```
colcon build
```

## 静态检测

使用[`prek`](https://prek.j178.dev/)工具来安装静态检测工具。你可以执行以下命令来安装静态检测工具：

```bash
cd circle_pilot
prek install --hook-type pre-commit --hook-type commit-msg
```

> [!NOTE]
> `prek`工具通过`pip`安装，请参阅[安装Python依赖包](#安装python依赖包)。


> [!NOTE]
> `prek`中`clang-tidy`工具需要先编译源代码才能正确执行。


安装完成后,执行`git commit`时会自动执行静态检测.

你也可以手动执行静态检测：

```bash
cd circle_pilot
prek run
```

你也可以运行特定的静态检测工具：

```bash
cd circle_pilot
prek run ruff
```
