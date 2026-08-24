# DENSO robot ros2 振動対策パッチ 適用手順書

本手順書は、`DENSORobot/denso_robot_ros2` に対する **実機動作の振動対策パッチ**（制御周期を厳密な 8ms に固定する修正）の配布・適用方法をまとめたものです。

配布物：`denso_8ms_fix.patch`（gitパッチ 1ファイル）

---

## 1. このパッチの概要

DENSO 実機（RC8/RC9、SYNCスレーブモード）は **8ms（125Hz）周期**で目標関節位置を要求します。
標準の `controller_manager/ros2_control_node` は制御ループに実時刻由来の周期を渡すため、ロボットが要求する厳密な 8ms グリッドと一致せず、**動作が振動的**になる場合があります。

本パッチは、実機用の専用制御ノード `denso_ros2_control_node` を追加し、制御ループへ **厳密に 8ms 刻みの時刻**を渡すことでこの問題を解消します。

### 変更されるファイル（計7ファイル）

| 種別 | ファイル |
|---|---|
| 新規 | `denso_robot_control/src/denso_ros2_control_node.cpp` |
| 変更 | `denso_robot_control/CMakeLists.txt` |
| 変更 | `denso_robot_control/package.xml` |
| 変更 | `denso_robot_bringup/launch/denso_robot_bringup.launch.py` |
| 変更 | `denso_robot_control/launch/denso_robot_control.launch.py` |
| 変更 | `denso_robot_moveit_config/robots/vs060/config/denso_robot_controllers.yaml` |
| 変更 | `denso_robot_moveit_config/robots/cobotta/config/denso_robot_controllers.yaml` |
| 変更 | `denso_robot_moveit_config/robots/hsr065a1_n32/config/denso_robot_controllers.yaml` |

---

## 2. 前提環境

- ROS2 Humble
- `DENSORobot/denso_robot_ros2` をクローン済みのワークスペース
- 実機（RC8/RC9）または同等構成
- `git`, `colcon` が利用可能

---

## 3. パッチ適用手順（受領側）

### 3-1. パッチファイルを配置
配布された `denso_8ms_fix.patch` を、クローンした `denso_robot_ros2` の **リポジトリルート**に置きます。

```bash
cd <your_ws>/src/denso_robot_ros2
```

### 3-2. 事前チェック（競合確認）
適用前に、必ずドライラン（`--check`）で競合が無いか確認してください。

```bash
git apply --check denso_8ms_fix.patch
```

- **何も出力されなければ適用可能**です。次に進んでください。
- エラーが出た場合は「7. トラブルシューティング」を参照してください。

### 3-3. パッチ適用

```bash
git apply denso_8ms_fix.patch
```

適用後、変更内容を確認する場合：

```bash
git status
git diff --stat
```

---

## 4. ビルド

変更のあるパッケージのみを対象にビルドします。

```bash
cd <your_ws>
colcon build --packages-select \
  denso_robot_control \
  denso_robot_bringup \
  denso_robot_moveit_config
source install/setup.bash
```

> ワークスペース全体をビルドしても問題ありません（`colcon build` のみ）。

---

## 5. 実行方法

これまでどおりの bringup コマンドで起動します。実機起動時（`sim:=false`）、制御ノードが自動的に新しい `denso_ros2_control_node` に切り替わります。

```bash
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py \
  model:=vs060 sim:=false ip_address:=192.168.0.1
```

- `model` はご使用の機種に合わせてください。
- RViz2 の MotionPlanning から「Plan & Execute」で動作させる手順は従来と変わりません。

### 適用確認
起動ログに次のようなメッセージが出れば、8ms 固定周期で動作しています。

```
Using fixed virtual control period of 8.000 ms
```

---

## 6. 既知の制約（重要）

### update_rate は必ず 125（Hz）にすること

本ノードは、コントローラ設定 `denso_robot_controllers.yaml` の `update_rate` から制御周期を算出します。

```yaml
controller_manager:
  ros__parameters:
    update_rate: 125  # Hz  ← 8ms。必ず 125 のままにすること
```

- 本パッチは、対象機種の yaml を `update_rate: 1000 → 125` に変更済みです。
- **この値を 125 以外（例：1000）に変更しないでください。**
  125 以外にすると制御周期がロボットの要求する 8ms と一致しなくなり、**振動が再発したり、動作が正しく行われません**。
- 独自に yaml を管理している場合や、上記 3 機種以外の機種を使用している場合は、その機種の `denso_robot_controllers.yaml` の `update_rate` を **必ず 125 に設定**してください。

---

## 7. トラブルシューティング

### `git apply --check` でエラーが出る
お手元の `denso_robot_ros2` のバージョンが、パッチ作成元と異なる可能性があります。以下を試してください。

- 空白差異を無視して適用:
  ```bash
  git apply --check --whitespace=fix denso_8ms_fix.patch
  git apply --whitespace=fix denso_8ms_fix.patch
  ```
- 3-way マージで適用（部分競合を手動解決可能に）:
  ```bash
  git apply --3way denso_8ms_fix.patch
  ```
- それでも競合する場合は、リポジトリのコミット状態（適用先のベース）をご確認のうえ、配布元へご連絡ください。

### ビルドで `controller_manager` が見つからない
本パッチは `denso_robot_control` に `controller_manager` への依存を追加します。未インストールの場合は導入してください。

```bash
sudo apt install ros-humble-controller-manager
```

---

## 8. 元に戻す（ロールバック）

パッチを取り消す場合：

```bash
cd <your_ws>/src/denso_robot_ros2
git apply -R denso_8ms_fix.patch
```

その後、再ビルドしてください。

---

## 付録：配布側でのパッチ作成方法（参考）

配布側は、修正ブランチと `humble`（適用先ベ���ス）との差分からパッチを生成します。

```bash
git checkout copilot/fix-move-group-joint-state-receive-issue
git diff humble...copilot/fix-move-group-joint-state-receive-issue > denso_8ms_fix.patch
```
