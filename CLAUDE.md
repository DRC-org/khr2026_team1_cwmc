# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## このプロジェクトについて

**khr2026_team1_cwmc** (Controller With Micro Controller) は、関西春ロボコン 2026 用の ESP32 ファームウェアです。

CAN バスを介して M3508 モーターと LSM9DS1 IMU を制御し、WiFi 経由で micro-ROS を使用して Raspberry Pi と通信します。

関連リポジトリ（同じ `../` ディレクトリに存在）:
- `khr2026_team1_rspi`: Raspberry Pi 上の中央制御プログラム
- `khr2026_team1_bt_controller`: ブラウザで動く Bluetooth コントローラ

## ビルドコマンド

このプロジェクトは VS Code 拡張機能の **PlatformIO** を使用しています。

```sh
pio run                                     # ビルド
pio run --target upload                     # ESP32 にアップロード
pio device monitor                          # シリアルモニタ (115200 baud)
pio run --target upload && pio device monitor  # アップロード + モニタ
pio run --target clean                      # ビルド成果物のクリーン
```

**重要**: プラットフォームは `espressif32 @ 6.5.0` に固定。ビルド前スクリプト `patch/apply_microros_patch.py` が micro-ROS Colcon ビルドの問題を自動パッチする。

## アーキテクチャ概要

### システム全体のデータフロー

```
Bluetooth コントローラ
  ↓ (BT JSON)
khr2026_team1_rspi (RobotController ノード)
  ├─ M3508Controller (逆運動学 → 4輪 RPM)
  └─ pub: wheel_control (WheelMessage @ 50ms)
      ↓ (ROS 2 / micro-ROS over WiFi)
    ESP32 cwmc_node
      ├─ MicroROSTask [Core 0]: ROS 2 通信
      └─ ControlTask [Core 1]: PID 制御 (3ms) → CAN → M3508
```

### マルチコアタスク構成

```
Core 0: MicroROSTask (priority 2)        Core 1: ControlTask (priority MAX-1)
├── WiFi 接続 / micro-ROS agent 通信     ├── CAN メッセージ受信
├── wheel_control トピック受信            ├── PID 制御計算 (3ms 周期)
└── wheel_feedback トピック送信 (50ms)   └── M3508 へ電流指令を CAN 送信
```

共有データはバイナリセマフォ `DataMutex` で保護。`volatile` で宣言された `target`/`current`（RPM）、`output_currents`、`pid_gains`、`pid_terms` を共有。

### 通信バス

| Bus | 対象 | 速度 | 詳細 |
|-----|------|------|------|
| CAN (TWAI) | 4x M3508 モータ | 1 Mbps | TX: GPIO 16, RX: GPIO 4 |
| I2C | LSM9DS1 IMU | 400 kHz | ヨー角（地磁気偏角 8.16° 補正） |
| WiFi / USB | Raspberry Pi (micro-ROS) | UDP / Serial | `platformio.ini` で切り替え |

## CAN ライブラリ (`lib/can/`)

### クラス設計

```
CanTransmitter (interface)   CanReceiver (interface)
        ↑                           ↑
        └──────── CanCommunicator ──┘
                  (TWAI ドライバ使用)
```

- `CanTransmitter::transmit(CanTxMessage)` — メッセージ送信
- `CanReceiver::receive()` — メッセージ受信（3ms ごとに呼ぶ）
- `CanReceiver::add_receive_event_listener(ids, callback)` — CAN ID フィルタ付きコールバック登録

### CanDest: CAN ID マッピング

```cpp
// lib/can/src/values/can_dest.hpp
enum class CanDest : can::CanId {
    central   = 0x000,
    dc_0      = 0x100, dc_1 = 0x101, dc_2 = 0x102,
    m3508_1   = 0x201, m3508_2 = 0x202, m3508_3 = 0x203, m3508_4 = 0x204,
    servo_cone = 0x300, servo_ball = 0x301,
    power_0   = 0x400, power_1 = 0x401,
};
```

### CanTxMessageBuilder: 送信メッセージ構築

```cpp
CanTxMessage msg = CanTxMessageBuilder()
    .set_dest(CanDest::m3508_1)  // または .set_id(0x200)
    .set_command(0x10)
    .set_value(100)
    .set_omake({0, 0, 0})        // 3 バイトの追加データ
    .build();
can_comm->transmit(msg);
```

### M3508 向け CAN フォーマット

**送信 (ESP32 → C620 ESC):**
- ID: `0x200`、DLC: 8 バイト
- `[0:1]` 左前、`[2:3]` 右前、`[4:5]` 左後、`[6:7]` 右後 (int16_t, big-endian)
- 電流変換: `milli_ampere * 16384 / 20000`

**受信 (M3508 → ESP32):**
- ID: `0x201`-`0x204`（各モータ）
- `[2:3]` 現在 RPM (int16_t)

## micro-ROS 接続管理 (`src/main.cpp`)

**状態遷移:** `WAITING → CONNECTED → DISCONNECTED → WAITING`

- `WAITING`: 100ms ごとに Agent へ ping を送り、成功したら `CONNECTED` へ
- `CONNECTED`: エンティティ生成、50ms 周期でフィードバック送信、100ms 周期で生存確認
- `DISCONNECTED`: モータ安全停止（電流 0 送信）→ エンティティ破棄 → `WAITING` へ

## ROS 2 メッセージ

**WheelMessage** (`robot_msgs/msg/WheelMessage.msg`):
```
WheelSpeeds m3508_rpms         # 4輪の目標/現在 RPM (fl/fr/rl/rr)
PIDGains[<=1] m3508_gains      # kp/ki/kd（オプション）
FeedbackPIDTerms[<=1] m3508_terms  # 各輪の P/I/D 項（オプション）
```

トピック:
- `wheel_control` — Raspberry Pi → ESP32（指令）
- `wheel_feedback` — ESP32 → Raspberry Pi（フィードバック）

## PID 制御仕様 (`src/main.cpp`)

- 制御周期: 3ms (`#define DT 0.003f`)
- 出力制限: ±5000 mA (`#define CLAMPING_OUTPUT 5000`)
- 積分制限: ±10000.0 (`#define INTEGRAL_LIMIT 10000.0f`)
- アンチワインドアップ: 飽和方向と逆向きのエラーのみ積分継続（方向性アンチワインドアップ）
- ゲインはランタイムで調整可能（Raspberry Pi から受信）: kp: 0-10, ki: 0-1, kd: 0-1

## Raspberry Pi 側 (`khr2026_team1_rspi`)

### 主要ノード

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| `ros2_node.py` (RobotController) | `robot_control` | Bluetooth 入力受付・逆運動学・指令送信 |
| `cmd_vel_bridge_node.py` | `auto_nav` | `/cmd_vel` (Twist) → `wheel_control` 変換（自律走行時） |

### 逆運動学（4輪オムニ）

`constants.py` のパラメータ:
- `WHEEL_RADIUS = 0.04925` m
- `GEAR_RATIO = 19.20320855614973`
- `L_X = 0.1725` m（前後方向）、`L_Y = 0.2425` m（左右方向）

### Raspberry Pi 開発コマンド

```sh
# ワークスペースルートで
colcon build --packages-select robot_control --symlink-install
source install/setup.bash
ros2 run robot_control ros2_node

colcon build --packages-select auto_nav --symlink-install
ros2 run auto_nav cmd_vel_bridge_node
```

## コーディング規約

- C++20 (`-std=gnu++2a`) と `-Wall` を使用
- コミットメッセージ: 日本語 + Conventional Commits プレフィックス (`feat:`, `fix:` 等)
- コメントは原則書かない。書く場合は **なぜ** その実装が必要かを簡潔に記述

## 注意事項

1. **Serial vs WiFi 切り替え**: `platformio.ini` の `board_microros_transport` で変更。Serial 使用時は `Serial.print()` によるデバッグ出力禁止（micro-ROS フレーム破損）

2. **WiFi パワーセーブ**: WiFi 使用時は `esp_wifi_set_ps(WIFI_PS_NONE)` で DTIM スリープを無効化済み（`src/main.cpp`）

3. **`lib/can_old/`**: 旧 CAN 実装の残骸。新規コードでは使用しない

4. **WiFi 設定**: 開発環境の SSID/パスワード/Agent IP は `src/main.cpp` に直書き (`ssid = "DRC"`, `psk = "kumachan"`, `agent_ip = 192.168.1.101`, port `8888`)
