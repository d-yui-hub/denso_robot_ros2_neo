# 加速度スパイク調査ノート

## 目的
VMB2515でMoveIt実行時、加減速点で加速度指令にスパイクが発生する。原因を特定する。

## 環境 / 対象
- リポジトリ: d-yui-hub/denso_robot_ros2_neo
- ブランチ: add-debug-publisher-humble
- 対象ロボット: VMB2515（gitに追跡されていない手動配置フォルダ。ブランチ操作の影響を受けない）
  - denso_robot_descriptions/robots/vmb2515/
  - denso_robot_moveit_config/robots/vmb2515/
- WS: ~/patch_test_ws
- 動作用スクリプト: movetest.py（plan_only=False、4点を無限ループでPlan&Execute）
  - max_velocity_scaling_factor = 0.2
  - max_acceleration_scaling_factor = 0.01（現状。エラー未発生）

## 追加した計測（write() 内、verbose時のみpublish）
- /vmb2515/debug/cmd_position     (rad)
- /vmb2515/debug/cmd_velocity     (rad/s)
- /vmb2515/debug/cmd_acceleration (rad/s^2)
- /vmb2515/debug/cmd_dt           ([wall_dt, period])
- 速度/加速度の分母は period_sec（= getPeriod() = ctrl_->get_Duration() = 8ms固定）
- wall_dt = getTime() の前回writeとの差分（= write()の実呼び出し間隔）

## 主要な発見（bag: accel_0.01, 9.3s, 各トピック約2908サンプル）
- period（名目）: mean=8.000ms, std=0.000（= 設定 update_rate:125Hz と一致）
- wall_dt（実測write間隔）: mean=3.205ms, std=0.396, min=2.642, max=4.337（≒312Hz）
- → **設定125Hz(8ms)に対し、実際は約312Hz(3.2ms)でwriteされている＝約2.5倍の周期矛盾**
- cmd_acceleration 波形: 基本は矩形だが、加減速の立ち上がり/立ち下がりで大きなスパイク（棘）
- sign-changes/joint: position≈[4,3,0,4,0,0]（滑らか）, velocity≈[17,14,16,15,16,15], acceleration≈[24,23,25,22,25,24]
- position波形は滑らか → 軌道自体は正常。問題は微分（差分）で顕在化

## 設定確認済み
- 全ロボットの denso_robot_controllers.yaml: `update_rate: 125  # Hz`（VMB2515含む）
- denso_ros2_control_node.cpp: `cm.get_update_rate()` を取得し `1e9 / update_rate` をDurationとして返す（設計上は8ms）
- getPeriod(): `ctrl_->get_Duration()` を返す（DENSO名目スレーブ周期=8ms）
- write() 実体: denso_robot_control.cpp の DensoRobotControl::write() → rob_->ExecSlaveMove(pose, joint_)
- 1000Hzループ: denso_robot_hw.cpp の SpinNode 内 std::thread（WallRate(1000)）が drobo->Update() を回す（write()はここではなくros2_control本体ループから）

## 仮説
125Hz(8ms)前提で生成された軌道点を、実際には312Hz(3.2ms)でwriteしている。
→ 同一軌道点の重複write、または補間なしの飛び → cmd_[i]が「同値が続く→急に飛ぶ」パターン
→ 差分がゼロ/大を繰り返す → 2階差分（加速度）で大スパイク（= 観測された加減速点の棘）

## 未実行の次アクション
1. check_duplicate.py で cmd_position の「前回と同値」割合を確認（重複writeの有無 = 仮説の決定打）
2. ロボット動作中に `ros2 topic hz /vmb2515/debug/cmd_position` で実周期を直接測定（312Hzか125Hzか）
3. launch起動ログで controller_manager が実際に使う update rate を確認
   （例: `grep -rn "update rate" ~/.ros/log/`）

## 対策候補（原因確定後に選択）
- 加速度計算の分母を period_sec(8ms固定) から wall_dt（実測）ベースに変更
- write呼び出しを125Hz(8ms)に矯正、または重複指令をスキップ
- update_rate が実際に適用されているか（controller_managerのreal-timeループ）を是正

## 実験メモ
- 正常時bag: accel_0.01（~/ 直下。ホーム直下に保存された）
- 解析スクリプト: ~/analyze_bag.py（NS=/vmb2515, TOPICS=debug/cmd_*）
- 加速度を上げる実験を行う場合は、bag記録を先に開始してから movetest.py を実行し、
  加速度値ごとに `-o accel_0.05` 等で分けて記録する。エラー発生時は加速度値をメモ。
