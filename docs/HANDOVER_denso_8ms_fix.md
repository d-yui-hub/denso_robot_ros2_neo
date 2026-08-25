# 引き継ぎメモ（統合版）：denso_robot_ros2 振動対策（8ms化 / 仮想時間導入）

> 新規Copilotチャットの冒頭に貼るための引き継ぎ資料。調査の再実行を省くための確定情報集。
> 初版8章 + これまでの追記 + 標準版vs修正版のループ対比（第4-2章）を1ファイルに統合。

---

## 0. 一行サマリ
DENSO RC8/RC9 の実機動作が振動的になる問題を、ROS2 制御ループに**仮想時間（8ms固定周期）**を導入して解決した。実装は fork `d-yui-hub/denso_robot_ros2_neo` に3つのstacked PRとして存在。残課題は「長期運転での仮想時間ドリフト」と「write()エラーハンドリング」。

---

## 1. 環境・基本情報
- リポジトリ: `d-yui-hub/denso_robot_ros2_neo`（fork���: `DENSORobot/denso_robot_ros2`）
- 作業ベースブランチ: `humble`
- 環境: ROS2 Humble / 実機 VMB2515（開発中・**VS060相当と仮定**）/ RC9 / `denso_robot_bringup`
- 実行コマンド:
  ```
  ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vmb2515 sim:=false ip_address:=192.168.0.1
  ```
- 主対象ファイル:
  - `denso_robot_control/src/denso_ros2_control_node.cpp`（新規追加：8ms化ノード）
  - `denso_robot_bringup/launch/denso_robot_bringup.launch.py`
  - `denso_robot_control/launch/denso_robot_control.launch.py`
  - `denso_robot_control/CMakeLists.txt` / `denso_robot_control/package.xml`
  - `denso_robot_moveit_config/robots/{vs060,cobotta,hsr065a1_n32}/config/denso_robot_controllers.yaml`
  - （標準実装の参考）`ros-controls/ros2_control` humble `controller_manager/src/ros2_control_node.cpp`
- 注意: `vmb2515` の MoveIt 設定ディレクトリは**存在しない**（robots配下は vs060 / cobotta / hsr065a1_n32 の3機種のみ）。planning group = `arm`、joints = `joint_1..joint_6`（vs060相当）。

---

## 2. ロボット側の通信仕様（SYNC / スレーブモード）
- **要求周期**: RC8/RC9 は **8ms 周期（125Hz）ごとの目標関節位置**を要求。
- **通信コマンド**: `slvMove`（`ExecSlaveMove`）で目標位置送信（SYNCスレーブモード）。
- **ロボット側バッファ**: 受信した `slvMove` は **3段バッファ**に格納され、**8msごとに1つ消費**。
- **write()の挙動**:
  - バッファに空きあり → `write()` は瞬時（約2〜4ms）に完了。
  - 3バッファ満載 → 空きができるまで返らない（**8msブロッキングで律速**）。
- **アンダーフロー**: バッファが空のまま8msを迎えると発生 → **スレーブモード解除**。
- **エラー時**: slvMove失敗でスレーブモード解除されると、**エラークリアしない限り動作不可**。
- **ペーシングの原則**: **SYNC `write()` のブロッキングが唯一の正しいペーシング源**。ソフト側で二重にクロック（sleep/WallRate/周期スキップ）を持たせると、バッファ枯渇→ロボット停止のリスク。

---

## 3. 不具合の原因（確定）
- ROS2標準の `controller_manager/ros2_control_node` は `update()` に**実時刻（wall/steady）由来のtime/period**を渡す。
- 出力される目標位置が「厳密な8ms刻み」とズレ、軌道サンプリングがずれる。
- **バッファ平滑化は送信タイミングのジッタは吸収するが、値そのものの格子ズレ（位相ノイズ）は吸収できない** → 隣接スロットの位置差分（=速度指令）が振動 → これが振動の正体。
- 副次問題：`/joint_states` のスタンプがゼロエポック（`rclcpp::Time(0,0)`）だと、`move_group` のシステム時刻ベースの10秒recencyチェックに失敗し、**起動時クラッシュ**。

---

## 4. 対策（確定した設計）
制御ループに**仮想時間（virtual_time）**を導入し、**厳密に8ms刻みで進む時刻**を read/update/write に渡す。

### 設計原則（厳守）
- ループ内で `now()`（実時刻取得）を**呼ばない**。
- ループ内に `sleep()` / `WallRate` を**一切入れない**（唯一のペーシング源は SYNC `write()` ブロッキング）。
- `virtual_time` は毎周期 `+= fixed_period`（8ms）で前進。
- `virtual_time` の**初期値だけ** `cm->now()`（実クロック起点）にする（起動時1回のみ、PR#4）。

### 確定した制御ループ
```cpp
const auto fixed_period = GetFixedPeriod(*cm);   // 8ms
auto virtual_time = cm->now();                   // 起点のみ実クロック（1回だけ）
while (rclcpp::ok()) {
  cm->read(virtual_time, fixed_period);
  cm->update(virtual_time, fixed_period);        // 厳密に8ms刻みの時刻を渡す
  cm->write(virtual_time, fixed_period);         // ← 8msブロッキングで律速
  virtual_time = virtual_time + fixed_period;     // += 8ms
}
```

### fork最新版（humble）からの変更点一覧（計7ファイル）
| ファイル | 変更内容 |
|---|---|
| `denso_robot_control/src/denso_ros2_control_node.cpp` | **新規追加。** 実機用の専用制御ノード |
| `denso_robot_bringup/launch/denso_robot_bringup.launch.py` | 実機用ノードを標準→`denso_ros2_control_node` に差替（`sim:=false`時） |
| `denso_robot_control/launch/denso_robot_control.launch.py` | 同上 |
| `denso_robot_control/CMakeLists.txt` | ノードのビルド/インストール、`controller_manager` 依存追加 |
| `denso_robot_control/package.xml` | `controller_manager` を `exec_depend`→`depend` へ |
| `.../{vs060,cobotta,hsr065a1_n32}/.../denso_robot_controllers.yaml` | `update_rate: 1000`→`125`（Hz）。8ms周期に一致 |

---

## 4-2.【追記】標準 ros2_control_node と 修正版ノードのループ構造・中身の対比

### なぜこの章があるか
「なぜ標準の `controller_manager/ros2_control_node` を使わず、専用ノードを自作したか」を後任者が一目で理解するための対比。「Read/Update/Write を while 無限ループで回して大丈夫か（コールバックのように即座に返さなくてよいか）」という疑問への回答も兼ねる。

### 結論
- **標準ノードも、ブランチ前から別スレッドでの `while` 無限ループで Read/Update/Write を回していた。** コールバック方式ではない。したがって修正版の「別スレッド＋whileループ」構造は**標準作法そのもの**。
- 標準版と修正版の違いは**構造ではなくループの中身**。

### 構造は同じ
| | 標準 ros2_control_node | 修正版 denso_ros2_control_node |
|---|---|---|
| executor | mainスレッドで `executor->spin()` | 別スレッドで `executor->spin()` |
| 制御ループ | **別スレッド `cm_thread` 内の `while(rclcpp::ok())`** | **mainスレッドの `while(rclcpp::ok())`** |
| Read/Update/Write | コールバックではなく能動的に叩く関数 | 同左 |

※ mainに置くスレッドは逆だが、「executorとは別スレッドで無限ループを回す」本質は同一。

### ループの中身の違い（今回の対策の核心）
| 観点 | 標準版（ブランチ前） | 修正版（本ブランチ） | 効果 |
|---|---|---|---|
| **update()に渡す時刻** | `cm->now()`（実時刻）を毎周期取得 | `virtual_time`（8ms固定刻みの仮想時間） | 振動対策（8ms格子と一致） |
| **period** | `measured_period`（実測した実時間差・ばらつく） | `fixed_period`（厳密に8ms固定） | 振動対策・速度計算の安定 |
| **ペーシング** | `std::this_thread::sleep_until(next_iteration_time)`（ソフトsleepで周期生成） | **なし**。write()のSYNCブロッキングのみ | **二重クロック解消＝アンダーフロー対策** |
| **リアルタイム設定** | SCHED_FIFO / メモリロック / CPUアフィニティ 等あり | なし（write()同期に依存） | 優先度への依存度を下げる設計 |

### 標準版の該当コード（参照）
`ros-controls/ros2_control`（humble）`controller_manager/src/ros2_control_node.cpp`：
- 制御ループは `std::thread cm_thread(...)` 内の `while (rclcpp::ok())`。
- ループ内で `cm->read/update/write(cm->now(), measured_period)` と**実時刻**を渡す。
- 末尾で `std::this_thread::sleep_until(next_iteration_time)` により **update_rate 由来の周期をソフトsleepで生成**（＝二重クロックの発生源）。
- main側は `executor->spin()`。

### この対比から導かれる因果（総括）
- **振動の原因** = 標準版が `cm->now()` と `measured_period` を渡し、目標位置が8ms格子とずれた。→ 仮想時���(8ms固定)化で解消。
- **「起動後しばらくしてのアンダーフロー」の原因** = 標準版の `sleep_until`（update_rate=1000 → 1ms周期のソフトクロック）がロボットの8ms消費と非同期にwriteを試み、3段バッファ残量を積分的に食い潰した。→ `sleep_until` 削除＋write()律速で解消。
- 「振動」と「しばらく後のアンダーフロー」は**同一の根（標準版の実時刻＋sleepペーシング）から生じた別症状**であり、仮想時間化＋sleep削除で**同時に解消**された。

### 「無限ループで大丈夫か」への回答（後任者向け）
- Read/Update/Write は **ROSが呼ぶコールバックではなく、こちらが能動的に叩く関数**。「即座に制御を返す」概念は当てはまらない。
- executor（購読・サービス・TF等）は**別スレッド**で回り続けるため、制御ループが無限whileでもROS処理は停止しない。
- ループが暴走（CPU100%）しないのは **write()が約8msブロックして律速**するため。sleepは不要かつ入れてはいけない（二重クロック回避）。
- 前提条件：①executorが別スレッドでspinしていること、②write()がブロッキングであること。本ノードは両方を満たす。

---

## 5. 起動直後のアンダーフローについて（確定：発生しない）
- ループに意図的なウェイトを入れていないことが、そのまま起動時対策。
- 起動直後はバッファに空きあり → `write()` が瞬時完了 → read/update/write が高速連続実行 → **3バッファが高速充填**。
- 充填後は `write()` の8msブロッキングで自然に8msグリッドへ同期。
- **結論：本ループ構成により起動直後のアンダーフローは発生しない。**
  ※ コード内コメントの「underrun mitigation は phase-2」は本確定結論より前の記述。

---

## 6. update_rate の役割と誤設定リスク（重要）
- 本ノードは `GetFixedPeriod()` で `cm.get_update_rate()` を読み、`fixed_period = 1e9 / update_rate` を算出。update_rate=125 → 8ms。
- **update_rate は「ループ周期」を決めない**（周期は write() ブロッキングが決める）。決めるのは **`fixed_period` の値**。
- したがって **update_rate=125 以外にすると `fixed_period` が実周期(8ms)と乖離して破綻**する。
  - 例：update_rate=1000 にすると `fixed_period`=1ms。ループ実周期はwrite()律速で8msのまま、仮想時間だけ1/8速で進む → 動作が異常に遅く・振動再発。
- yamlの `update_rate: 125` は**必須設定**。125固定が前提。
- （改善候補）`fixed_period` を8ms固定定数化し、update_rate が125以外なら警告/エラーを出すガード追加が堅牢。

---

## 7. PR構成（stacked / 数珠つなぎ）
**#4 → #2 → #1 → humble** の順にマージ。すべてOpen/Draft。

| PR | 内容 | head | base |
|----|------|------|------|
| #4 | `virtual_time` を実クロック起点(`cm->now()`)に（move_group起動時クラッシュ修正） | `copilot/fix-move-group-joint-state-receive-issue` | `copilot/copilotfix-update-rate-fetching` |
| #2 | 周期取得を `get_parameter("update_rate")`→`cm.get_update_rate()` に | `copilot/copilotfix-update-rate-fetching` | `copilot/fix-vibration-issues` |
| #1 | 専用ノード新規追加・launch差替・yaml変更 | `copilot/fix-vibration-issues` | `humble` |

補足：`copilot/copilotcopilotfix-update-rate-fetching`（"copilot"が1つ多い）**未使用の残骸ブランチ**あり。実害なしだが��乱注意（削除検討可）。

### PR#4の詳細
- **問題**: `virtual_time` 初期値がゼロエポック → `/joint_states` スタンプが1970年起点 → move_groupの10秒recencyチェックに無条件失敗 → **起動時クラッシュ**。
- **修正**: 初期値を `rclcpp::Time(0,0)` → `cm->now()` に変更（1行）。8ms刻みの厳密性は維持。

---

## 8. 他社ROS2ドライバの時刻設計比較（参考）
| メーカ | 制御ループ主体 | update()の時刻 | ペーシング源 | 格子ズレ耐性 |
|---|---|---|---|---|
| **Fanuc** | 標準 `ros2_control_node` | 実クロック由来 | ROSタイマ＋ロボット側補間 | 中 |
| **ABB (abb_ros2, EGM)** | 標準 `ros2_control_node` | 実クロック由来（HW I/Fでは無視） | EGM UDP 4ms/250Hz同期 | 高 |
| **DENSO (今回)** | **専用ノード** | **仮想時間（8ms格子）** | SYNC `write()` ブロッキング（8ms） | 送信側で格子化して確保 |

### ABBが格子ズレに強い理由（DENSOにない吸収機構）
- ①位置＋速度FF指令（2つのcommand interface）→ 位相ジッタに一次ロバスト。
- ②受信側の平滑化（EGMの `lp_filter` / `max_speed_deviation` / `pos_corr_gain`）。
- ③受信側リサンプル（EGMがロボット側4msグリッドで最新値をサンプリング）。
- DENSOのslvMoveは位置のみ・全サンプル消費・受信側フィルタが弱い → **送信側（ROS）で仮想時間による格子化が本質的に必要**。

---

## 9. use_sim_time を採用しない設計判断
- **use_sim_time**: trueにするとノードは壁時計でなく `/clock` 配信時刻を使う（本来シミュレータ用）。
- **不採用の理由**:
  1. 実機であり、全系をsim timeにするとログ/rosbag/診断/外部連携が混乱。
  2. `/clock` 配信責任が重い（滞ると全系停止リスク）。
  3. 全ノード（move_group/RViz/TF/broadcaster）に設定必要 → 付け忘れで即ハイブリッド化しTF extrapolationエラー。
  4. ドリフト問題は use_sim_time でも解決しない。
  5. 目的（update()に渡す時刻だけ格子化）に対し過剰。
- **結論**: 影響範囲を制御ループ内に限定した現在の局所実装の方が、実機では副作用が小さく妥当。

---

## 10. 時刻設計の3案と長期ドリフト問題（検討中）
### ドリフト問題
- 現在案（`virtual_time += 8ms` 積算）は、実write周期の平均が8msから僅かにズレると現実時刻から徐々に乖離。
- 目安：実周期が平均8.001ms（0.0125%ズレ）なら 1時間≈0.45秒 / 1日≈10.8秒 / 1ヶ月≈約5.4分。

### 長期乖離で壊れるも��
| 影響先 | 問題 | 深刻度 |
|---|---|---|
| move_group recencyチェック | スタンプが実時刻からズレ→プランニング/実行拒否（PR#4の問題が長期で再発） | 高 |
| TF (tf2) | extrapolation error、RViz異常、センサ融合失敗 | 高 |
| センサ融合/時刻同期 | 実時刻スタンプのセンサと同一時刻軸で扱えず破綻 | 高 |
| rosbag / timeout / ログ | 突合ずれ、タイムアウト誤判定、解析困難 | 中〜低 |

### 3案
| 案 | 内容 | 長所 | 短所 |
|---|---|---|---|
| **A. 仮想時間（現在）** | `virtual_time += 8ms` | 等間隔・離散時間系として正しい | ドリフトする |
| **B. グリッド丸め** | 毎回 `now()` を8ms境界にfloor丸め | ドリフトしない | 等間隔性が崩れる（重複/飛び）、単調クランプ必要 |
| **C. 仮想時間＋緩やか同期（推奨）** | 基本A、乖離が閾値超で1ステップ微小量だけ現実時刻へ寄せる（PLL的） | 等間隔性維持＋長期追従 | 実装やや複雑 |

- 制御ループの時刻としては仮想時間（A/C）が制御理論的に正しい。
- Bは「同一グリッド重複→同じ指令の二重投入」の副作用あり。
- 推奨は案C（等間隔性と長期追従の両立）。

---

## 11. write()エラーハンドリング（未実装・要対応）
### 現状の欠陥
- 8ms化ノードは `cm->write()` の戻り値を見ておらず、例外も捕捉していない。
- slvMove失敗でスレーブモード解除されても、ループは `rclcpp::ok()` の間回り続けエラーを垂れ流す。

### あるべき処理
1. write()戻り値を検知：`OK` 以外でループを止める（最低限）。
2. ros2_control流儀：HW I/F（`DensoRobotHW::write()`）が失敗時に `return_type::ERROR` を返し、controller_managerがdeactivate。
3. 復帰設計：
   - 原則A（安全停止）：ループ停止・deactivate、人手/上位系の明示復帰を待つ（産業標準）。
   - 例外B（自動リカバリ）：軽微なエラーのみ `ExecClearError`＋スレーブ再投入。回数制限必須。復帰後 `virtual_time = cm->now()` で再アンカー。
   - 推奨：原則A、特定エラーのみB。
- 要確認：`DensoRobotHW::write()` がslvMove失敗時に何を返しているか。

---

## 12. DENSO公式パッケージの出来栄え評価（参考）
- 総評: ロボット通信の知見は深いが、ROS2/現代C++/OSS品質基準の経験が浅いベンダーチーム品質。ハード知見◎・ソフト工学△。
- 構造的欠陥（今回の温床）: `read/write` に time/period概念が欠���。SYNCが時刻に敏感なのにROS2制御ループの時刻設計と結びつける視点がなかった → 仮想時間対策はこの欠落を補う正しい方向。

---

## 13. 配布（ユーザ展開）方法：gitパッチ
- 方式：`humble` との差分を `.patch` 化して配布（`git diff humble...copilot/fix-move-group-joint-state-receive-issue > denso_8ms_fix.patch`）。
- 配布物：`denso_8ms_fix.patch` ＋ 手順書 `docs/README_denso_8ms_fix_patch.md`。
- 検証済み：パッチ各hunkのindex blob SHA（`cd2884c`, `4c93a1a` 等）が本家 `DENSORobot/denso_robot_ros2`（コミット `2805694`）の現行ファイルと一致 → **クリーンに適用可能**。
- 受領側手順：`git apply --check` → `git apply` → `colcon build`。バージョン差異時は `--3way`。
- 手順書には**update_rate=125必須**を明記。長期ドリフト/エラー処理は手順書には記載しない方針。

---

## 14. 未決事項 / 次アクション候補
- [ ] 時刻設計の最終決定：案A（現状維持）／案C（仮想時間＋緩やか同期）。長期連続運転想定なら案C推奨。
- [ ] write()エラーハンドリング実装（第11章）。まず `DensoRobotHW::write()` の `return_type` 調査から。
- [ ] update_rate=125以外で破綻する件のガード追加（第6章）。
- [ ] 3つのstacked PRのレビュー・承認・マージ（#1→#2→#4の順）。
- [ ] 実機でのドリフト量計測（`virtual_time` vs `cm->now()` の差分ログ）。
- [ ] 残骸ブランチ `copilot/copilotcopilotfix-update-rate-fetching` の削除検討。
- [ ] （任意）本家 `DENSORobot/denso_robot_ros2` へのPR提出。

---

## 15. Copilotへの依頼時のヒント
- 本メモのリポジトリ名・ブランチ名・PR番号・ファイルパスは確定情報。再調査不要で参照可。
- コード引用は上記フルパスを `getfile` で直接読めばよい。
- 標準版ループの参照元：`ros-controls/ros2_control` humble `controller_manager/src/ros2_control_node.cpp`。
- 他社比較の裏取り：`PickNikRobotics/abb_ros2` の `abb_hardware_interface` read/write、`abb_rws_client` のEGM設定。
