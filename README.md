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

#### 1. WSL2 のセットアップ

WSL2 上に Ubuntu をインストールします。

```sh
wsl --install
```

インストールできたら、Windows ターミナルから Ubuntu に接続します。

```sh
sudo apt update
sudo apt upgrade
```

#### 2. ROS 2 Kilted Kaiju のインストール

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

#### 3. Build Essential のインストール

```sh
sudo apt update
sudo apt install build-essential
```

---

## ESP32 コード仕様

### 概要

ESP32はmicro-ROSを使用してRaspberry Piと通信し、M3508モーターの制御を行います。

### 主な機能

- **CAN通信**: M3508モーター（C620 ESC）との通信
- **PID制御**: モーター速度のフィードバック制御
- **ROS 2通信**: 
  - `/cmd_vel` サブスクライバー（速度指令受信）
  - `/odom` パブリッシャー（オドメトリ送信）
- **マルチタスク**: FreeRTOSによる制御タスクとMicro-ROSタスクの並列実行

### システム構成

```
RaspberryPi (ROS 2) <--WiFi--> ESP32 <--CAN--> M3508モーター x4
                      (micro-ROS)
```

### トピック仕様

#### サブスクライブ: `/cmd_vel` (geometry_msgs/Twist)
- `linear.x`: 前進速度 [m/s]
- `angular.z`: 回転速度 [rad/s]

#### パブリッシュ: `/odom` (nav_msgs/Odometry)
- 送信周期: 約50Hz (20ms)
- frame_id: `odom`
- child_frame_id: `base_link`

### 制御仕様

#### PID制御パラメータ

```cpp
#define KP 1.0f   // 比例ゲイン
#define KI 0.05f  // 積分ゲイン
#define KD 50.0f  // 微分ゲイン
#define INTEGRAL_LIMIT 1000.0f  // 積分ワインドアップ防止
```

#### ロボットパラメータ（要実測！）

```cpp
const float WHEEL_RADIUS = 0.05f;  // [m] ホイール半径
const float WHEEL_BASE = 0.3f;     // [m] 左右ホイール間距離
```

**⚠️ 重要**: これらの値は実測値に変更してください！

### セットアップ手順

#### 1. WiFi設定の確認

`src/main.cpp` の以下の部分を環境に合わせて変更：

```cpp
char ssid[] = "DRC";
char psk[] = "28228455";
IPAddress agent_ip(192, 168, 0, 101);  // RaspberryPiのIPアドレス
size_t agent_port = 8888;
```

#### 2. ロボットパラメータの実測

1. **ホイール半径 (WHEEL_RADIUS)** の測定
   - ホイールの直径を測定し、半径を計算
   - 単位: メートル [m]

2. **ホイールベース (WHEEL_BASE)** の測定
   - 左右のホイール中心間の距離を測定
   - 単位: メートル [m]

3. `src/main.cpp` の42-43行目を実測値に変更

#### 3. コンパイルとアップロード

```sh
# コンパイル
pio run

# ESP32にアップロード
pio run --target upload

# シリアルモニタで動作確認
pio device monitor
```

### PIDゲインのチューニング方法

#### 初期値（推奨）

```cpp
#define KP 1.0f
#define KI 0.05f
#define KD 50.0f
#define INTEGRAL_LIMIT 1000.0f
```

#### チューニング手順

1. **P制御のみで調整** (KI=0, KD=0)
   - KPを徐々に上げて応答を確認
   - 振動し始める手前の値を採用

2. **D制御を追加** (KI=0)
   - KDを追加してオーバーシュートを抑制
   - 安定性が向上するまで調整

3. **I制御を追加**
   - KIを小さい値から徐々に増やす
   - 定常偏差（目標値とのズレ）が減るまで調整
   - 積分ワインドアップに注意

#### トラブルシューティング

| 症状 | 原因 | 対策 |
|------|------|------|
| 振動が激しい | KPが大きすぎる | KPを下げる |
| 応答が遅い | KPが小さすぎる | KPを上げる |
| オーバーシュート | KDが小さい | KDを上げる |
| 定常偏差が残る | KIが小さい | KIを上げる |
| 急に暴走する | 積分ワインドアップ | INTEGRAL_LIMITを下げる |

### 動作確認手順

#### 1. RaspberryPi側でmicro-ROS Agent起動

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### 2. トピック確認

```bash
# トピック一覧を確認
ros2 topic list

# cmd_vel を手動送信（前進0.1m/s）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# odom を確認
ros2 topic echo /odom
```

#### 3. Navigation起動

```bash
ros2 launch robot_control robot_bringup.launch.py
```

### タスク構成

#### ControlTask (Core 1, 最高優先度)
- 実行周期: 3ms
- 処理内容:
  - CAN受信処理
  - PID制御計算
  - CAN送信
  - オドメトリ計算・送信（21msごと）

#### MicroROSTask (Core 0, 標準優先度)
- 処理内容:
  - WiFi通信
  - cmd_vel受信
  - executor処理

### デバッグ情報

シリアルモニタで以下の情報が約1秒ごとに表示されます：

```
Target RPM: [100, 100, 100, 100]
Out Current: [500, 480, 510, 490]
```

### 注意事項

1. **frame_id の一致**: RaspberryPi側のTF設定と一致させること
   - `odom_msg.header.frame_id` = "odom"
   - `odom_msg.child_frame_id` = "base_link"

2. **WiFi接続**: ESP32とRaspberryPiが同じネットワークに接続されていること

3. **CAN配線**: CAN_H, CAN_L, GNDが正しく接続されていること

4. **電源**: モーターとESCに適切な電源が供給されていること
