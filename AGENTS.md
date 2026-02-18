# AGENTS.md

このファイルは、AI エージェントがこのリポジトリ内のコードで作業する際のガイダンスを提供します。

## このプロジェクトについて

**khr2026_team1_cwmc** (Controller With Micro Controller) は、関西春ロボコン 2026 用の ESP32 ファームウェアです。

CAN バスを介して M3508 モーターと LSM9DS1 IMU を制御し、WiFi 経由で micro-ROS を使用して Raspberry Pi と通信します。

他のノードは:

- `khr2026_team1_rspi` リポジトリ: Raspberry Pi 上の中央制御プログラム
- `khr2026_team1_bt_controller` リポジトリ: ブラウザで動く Bluetooth コントローラ

## ビルドコマンド

このプロジェクトは、VS Code の拡張機能である **PlatformIO** を使用しています。すべてのコマンドは PlatformIO CLI を介して実行されます:

```sh
# ビルド
pio run

# ESP32 にアップロード
pio run --target upload

# シリアルモニタ (115200 baud)
pio device monitor

# ビルド + アップロード + モニタ
pio run --target upload && pio device monitor

# ビルド成果物のクリーン
pio run --target clean
```

**参考**: プラットフォームは `espressif32 @ 6.5.0` に固定されており、micro-ROS のビルドエラーを回避しています。ビルド前スクリプト (`patch/apply_microros_patch.py`) が micro-ROS Colcon ビルドの問題を自動的にパッチします。

## アーキテクチャ概要

### マルチコアのタスク構成

```
Core 0: MicroROSTask (priority 2)        Core 1: ControlTask (priority MAX-1)
├── WiFi / micro-ROS agent               ├── CAN message RX processing
├── JSON command parsing                 ├── PID control (3ms period)
├── Feedback telemetry (50Hz)            └── Motor current TX via CAN
└── ROS 2 pub/sub (robot_feedback,
    robot_control topics)
```

共有データはバイナリセマフォ (`DataMutex`) で保護されています。

### 通信バス

| Bus | Target | Speed | Details |
|-----|--------|-------|---------|
| CAN (TWAI) | 4x M3508 motors | 1 Mbps | TX: GPIO 16, RX: GPIO 4. TX ID: 0x200, RX IDs: 0x201-0x204 |
| I2C | LSM9DS1 IMU | 400 kHz | Yaw angle with magnetic declination correction (8.16°) |
| WiFi / USB | Raspberry Pi (micro-ROS) | UDP / Serial | Communication method can be changed in `platformio.ini` |

### ROS 2 のメッセージ

Raspberry Pi からのコマンドとそれへのフィードバックは、JSON ペイロードを持つ `std_msgs/String` を使用します。制御メッセージには `m3508_rpms`（各車輪の目標RPM: fl/fr/rl/rr）とオプションの `pid_gains`（kp/ki/kd）が含まれます。フィードバックには実際のRPM、出力電流、PID項目の内訳が含まれます。

### カスタムライブラリ (`lib/`)

- **`can/`** — CAN/TWAI 抽象化。インターフェース (`CanTransmitter`, `CanReceiver`) とコールバックベースのイベントリスナーを使用。TX メッセージはビルダーパターンで構築。
- **`lsm9ds1_control/`** — SparkFun LSM9DS1 のラッパーで、ヨー角の計算を行う。

### PID 制御

- 周期: 3ms、出力は ±5000 mA に制限
- アンチワインドアップ: 積分制限 10000.0
- ゲインは Raspberry Pi からの JSON コマンドで実行時に調整可能 (kp: 0-10, ki: 0-1, kd: 0-1)

## コーディング規約

- C++20 (`-std=gnu++2a`) と `-Wall` が有効になっています。新しい C++ 機能を活用し、コードの安全性と可読性を向上させます。
- コミットメッセージは日本語で、Conventional Commits プレフィックス（`feat:`, `fix:`）を使用します。
- ソースコードのドキュメントとコメントは日本語で記述します。
- 原則としてコメントは書かず、自己説明的なコードを心がけてください。
- どうしてもコメントが必要な場合は、**何をしているのか** よりも **なぜそれをしているのか** を簡潔に記載するようにしてください。
