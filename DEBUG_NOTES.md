# 加速度スパイク調査ノート

## 目的
VMB2515でMoveIt実行時、加減速点で加速度指令にスパイクが発生する。原因を特定する。
関連する根本課題は「DENSOロボットの8ms SYNCスレーブ周期」と「ros2_control側のwrite周期」の整合。

## 主軸ブランチ / PR
- **作業ブランチ（主軸）: copilot/add-visualization-publisher**
- **PR #7**: Add verbose-gated cmd_interface telemetry at write() boundary for acceleration spike diagnosis
  - base: fix/denso-8ms-v3
  - 作成: Copilot coding agent（タスク fc533de3-9361-4d40-ae97-3b10d27c82de）
- 旧ノート（add-debug-publisher-humble ブランチのDEBUG_NOTES.md）は削除し、本ブランチに一本化。

## 関連PRの系譜（8ms周期問題の調査履歴）
- **PR #1** (copilot/fix-vibration-issues → humble): 固定周期のDENSO ros2_control node追加。
  read/update/write を virtual_time + fixed_period で駆動。sleep/WallRateを排除���、
  **write()のSYNCブロッキングのみをペーシング源とする設計**。
  VS060/COBOTTA/HSR065A1-N32 の update_rate を 1000→125 に変更。
- **PR #2**: GetFixedPeriod() を `cm.get_update_rate()` で取得（125Hz=8msを確実化）。
- **PR #4**: virtual_time を `cm->now()` の実クロックepochに合わせ、move_groupのjoint_states棄却を修正。
- **PR #5** (copilot/fix-robot-buffer-underflow): SYNC制御ループの優先度昇格(SCHED_FIFO/nice)と
  read/update/write の overrun計測（20ms超でログ）。8msループがCPU負荷で間に合わない問題に対処。
- **PR #6**: overrun診断強化（delta=wall-cpu 表示）。
- **PR #7** (本ブランチ): cmd_position/velocity/acceleration/dt の telemetry追加（今回の計測コード）。

## 追加した計測（write() 内、verbose時のみpublish）
- /vmb2515/debug/cmd_position     (rad)
- /vmb2515/debug/cmd_velocity     (rad/s)
- /vmb2515/debug/cmd_acceleration (rad/s^2)
- /vmb2515/debug/cmd_dt           ([wall_dt_sec, period_sec])
- 速度/加速度の分母は period_sec（= getPeriod() = ctrl_->get_Duration() = 8ms固定）
- wall_dt = getTime() の前回writeとの差分（= write()の実呼び出し間隔）
- verbose_==true かつ slave mode の write() でのみpublish（安全ゲート）

## ⚠️ PR #7 レビューで既に指摘済みの重要点
- CodeReviewコメント: 速度・加速度を period_sec(=8ms固定) で計算しているが、
  **スケジューラのタイミングジッタ（実write間隔が制御周期と異なること）こそが調査対象**。
  → derivativeの計算には **実測経過時間(wall_dt) を使うべき**、period は別途診断値として出す。
- 実装は「要件で固定8ms優先」との指示により period_sec のまま維持された。
- **今回のbag解析(8ms vs 3.2ms)がこの指摘を裏付けた。**

## 主要な発見（bag: accel_0.01, 9.3s, 各トピック約2908サンプル）
- period（名目）: mean=8.000ms, std=0.000（= 設定 update_rate:125Hz と一致）
- wall_dt（実測write間隔）: mean=3.205ms, std=0.396, min=2.642, max=4.337（≒312Hz）
- → **設定125Hz(8ms)に対し、実際は約312Hz(3.2ms)でwriteされている＝約2.5倍の周期矛盾**
- cmd_acceleration 波形: 基本は矩形だが、加減速の立ち上がり/立ち下がりで大きなスパイク（棘）
- sign-changes/joint: position≈[4,3,0,4,0,0]（滑らか）, velocity≈[17,14,16,15,16,15], acceleration≈[24,23,25,22,25,24]
- position波形は滑らか → 軌道自体は正常。問題は微分（差分）で顕在化。

## 設定確認済み
- 全ロボットの denso_robot_controllers.yaml: `update_rate: 125  # Hz`（VMB2515含む）
- denso_ros2_control_node.cpp: `cm.get_update_rate()` を取得し `1e9 / update_rate` をDurationとして返す（設計上は8ms）
- getPeriod(): `ctrl_->get_Duration()` を返す（DENSO名目スレーブ周期=8ms）
- write() 実体: denso_robot_control.cpp の DensoRobotControl::write() → rob_->ExecSlaveMove(pose, joint_)
- 1000Hzループ: denso_robot_hw.cpp の SpinNode 内 std::thread（WallRate(1000)）が drobo->Update() を回す
  （write()はここではなくros2_control本体ループから）

## 仮説
- 設計(PR #1)は「SYNC write()ブロッキングのみがペーシング」= 8msごとにwriteが戻るはず。
- しかし実測 wall_dt=3.2ms → **write()が8msブロックせず約3.2msで戻っている**疑い。
  → SYNCブロッキングが期待通り効いていない / バッファに余裕があり即時リターンしている等。
- 結果: 125Hz(8ms)前提の軌道点を312Hz(3.2ms)でwrite → 同一点の重複write or 補間なしの飛び
  → cmd_[i]が「同値が続く→急に飛ぶ」→ 差分がゼロ/大 → 2階差分(加速度)で大スパイク。

## 未実行の次アクション
1. check_duplicate.py で cmd_position の「前回と同値」割合を確認（重複writeの有無 = 仮説の決定打）
2. ロボット動作中に `ros2 topic hz /vmb2515/debug/cmd_position` で実周期を直接測定（312Hzか125Hzか）
3. launch起動ログで controller_manager が実際に使う update rate を確認
   （例: `grep -rn "update rate" ~/.ros/log/`）
4. PR #5/#6 の [CYCLE_OVERRUN] ログを確認し、write()フェーズの実時間を見る
   （write()が8msブロックしているか、即時リターンしているか）

## 対策候補（原因確定後に選択）
- 加速度計算の分母を period_sec(8ms固定) から wall_dt（実測）ベースに変更（レビュー指摘の対応）
- write()のSYNCブロッキングが効かない原因を是正（バッファ充填、SLVMODE設定、send/recv format）
- write呼び出しを125Hz(8ms)に矯正、または重複指令をスキップ

## 実験メモ
- 正常時bag: accel_0.01（~/ 直下。ホーム直下に保存された）
- 解析スクリプト: ~/analyze_bag.py（NS=/vmb2515, TOPICS=debug/cmd_*）
- 加速度を上げる実験を行う場合は、bag記録を先に開始してから movetest.py を実行し、
  加速度値ごとに `-o accel_0.05` 等で分けて記録する。エラー発生時は加速度値をメモ。
- movetest.py: max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.01（現状、エラー未発生）
