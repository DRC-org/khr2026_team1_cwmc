# khr2026_team1_cwmc

関西春ロボコン 2026 チーム 1 CWMC（Controller With Micro Controller）

CAN 通信を行い、各アクチュエータやセンサ等を制御します。PlatformIO 上の micro-ROS を用いて、ラズパイと通信しています。

## 開発環境の構築

### 必要なソフトウェア

- VS Code
- [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)（VS Code 拡張機能）
- WSL2
- Docker
- ROS 2 Kilted Kaiju（WSL 上にインストール）

### セットアップ

### 1. WSL2 のセットアップ

WSL2 上に Ubuntu をインストールします。

```sh
wsl --install
```

インストールできたら、Windows ターミナルから Ubuntu に接続します。

```sh
sudo apt update
sudo apt upgrade
```

### 2. ROS 2 Kilted Kaiju のインストール

以下のコマンドをすべて実行し、ROS 2 Kilted Kaiju をインストールします。

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt install ros-kilted-desktop
source /opt/ros/kilted/setup.bash
echo -e "\n# ROS 2\nsource /opt/ros/kilted/setup.bash" >> ~/.bashrc
```

### 3. Build Essential のインストール

```sh
sudo apt update
sudo apt install build-essential
```
