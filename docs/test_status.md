# G431モーションコントローラ + ROS2 bridge 試験状況

最終更新: 2026-10-05

## この文書の位置づけ

**前回版からの修正**: 前バージョンは`~/canable2-fw-src`を読まず、このリポジトリのコメント/READMEのみを根拠にしていたため、実際にこのセッション内で実機（motor0 + VESC 1台）で検証済みの項目の大半を「未検証」としていた。本版はこのセッションで実際に行った実機試験の結果を反映する。

試験条件: G431（canable2-fw-src, atcan-firmware branch, commit afe4754相当）、USB CDC経由、VESC 1台をCAN motor_index=0へ接続。motor1〜3は未配線（正しく無視されることのみ確認）。4軸同時の負荷試験は行っていない。

## 判定ルール

- PASS: 本セッション内で実機にて期待挙動を確認した。
- 未完了: 試験を開始したが、ユーザー指示により途中で中断した。
- 未検証: 実装はあるが本セッションで試験していない。
- 未確認: ソースコードを確認しておらず判断できない。

## 実機PASS事項（本セッションで確認）

### 基本通信・診断

| 項目 | 結果 |
|---|---|
| ASCII→binary handshake、GET_INFO、protocol_version=2確認 | PASS |
| HEARTBEAT、READ_PARAM診断poll | PASS |
| TELEMETRY受信・4軸regroup（motor1〜3はvalid=falseで正しくpublish） | PASS |
| set_enable/clear_faultサービス | PASS |

### CURRENTモード

| 項目 | 結果 |
|---|---|
| `effective_current_cmd`がROS2指令値と常に一致（ブリッジ正確性） | PASS |
| 0.5A指令で実回転確認（41.29 rad/s） | PASS |
| 符号反転（逆方向回転）確認 | PASS |
| `actual_current`（VESC側計測）とのズレ | 観測済み、VESC側センシング特性と判断（ブリッジ起因ではない） |

### VELOCITYモード

| 項目 | 結果 |
|---|---|
| 閉ループ速度追従（動作確認レベル） | PASS |
| `vel_kp`変更が実際の閉ループ挙動に反映されること | PASS（0.10/0.15/0.20/0.25で発進せず、0.30で発進、0.35で悪化を確認） |
| 停止時（target=0）に`effective_current_cmd`が即座に0へ、積分残留なし | PASS |
| `velocity_limit`が全sourceでOVER_SPEED fault閾値として働くこと | PASS（自爆して発見、設計通りと判断） |

**ゲイン値そのもの（`vel_kp=0.30`等）は性能チューニングの結論ではない**。「カスケードが実装されパラメータ変更が正しく反映される」ことの機能確認として打ち切った値であり、実負荷での再チューニングが前提。

### POSITIONモード

| 項目 | 結果 |
|---|---|
| 誤差→pos_kp→velocity_ref→vel_kp/ki→current_cmdの計算経路一致 | PASS |
| `profile_enable=false`で即時ジャンプ | PASS |
| `profile_enable=true`で中間値を経るランプ | PASS |
| 符号・方向の正しさ（+0.1/-0.1 rad） | PASS |
| disable時の即時0A | PASS |
| 位置決め精度・定常偏差なしでの目標到達 | 未完了（P制御のみの定常偏差を確認、Ki込みの最終チューニングは未実施） |

### Bumpless transition

| 遷移 | 結果 |
|---|---|
| CURRENT→VELOCITY | PASS（~100ms blend、不連続ジャンプなし） |
| VELOCITY→POSITION（position_refがactual_position近傍から開始） | PASS |
| POSITION→VELOCITY | PASS |
| VELOCITY→CURRENT | PASS |

### Override

| 項目 | 結果 |
|---|---|
| CURRENT override中、POSITION側のrefが凍結・競合しない | PASS |
| VELOCITY override中、同上 | PASS |
| override解除時、position_refがactual_position近傍から再開 | PASS |
| override進入・解除時のcurrent_cmd不連続ジャンプなし | PASS |

### ZERO_POSITION / limits

| 項目 | 結果 |
|---|---|
| ZERO_POSITION実行でactual_positionが0になる | PASS |
| 範囲外target_positionのCONTROL_COMMANDが`PARAM_ERR_RANGE`で拒否され、内部targetが不変 | PASS |
| 実位置がlimit外でenable時、次tickで`FAULT_BIT_POS_LIMIT`発生 | PASS |
| disable→clear_fault→範囲外のまま再enableで再発 | PASS（仕様通り、バグではない） |
| 範囲内に戻してから再enableでfaultなし | PASS |

### Host watchdog / HOST_TIMEOUT_DISABLE

| 項目 | 結果 |
|---|---|
| CONTROL_COMMANDのみ停止・HEARTBEAT継続でもper-motor timeout発火 | PASS |
| `host_rx_seen_since_armed`の罠（binary mode突入直後は先にHEARTBEAT/CONTROL_COMMANDが必要） | PASS（踏んで確認） |
| HOST_TIMEOUT_DISABLE: timeout後`enable=0`,`state=IDLE`,fault化しない | PASS |
| SIGKILL試験: プロセス異常終了→300ms後enable=0→再起動後set_enableなしでは動かない | PASS |
| 真のUSB抜線試験: `/dev/ttyACM0`消失・再enumeration→再接続→reconnect_count増加→set_enableなしでは動かない | PASS |
| 再接続handshakeでの全軸force-disable write | PASS |

## 未検証事項

- 4台同時接続時のCAN bus負荷・500Hz/axis実測jitter（コード上は確認済み、実測は1台構成のみ）。
- 数時間規模の長時間soak（約9分の部分試験のみで中断、中断までは全エラーカウンタ増加なし）。
- POSITIONモードのKi込み最終チューニング、オーバーシュート/振動の定量評価。
- 複数VESC構成でのfault escalation（bystander復帰）の実機確認（単軸構成のため、system fault自体は発生させていない）。
- flash_store.cの内部実装（schema/CRC/wear leveling/atomicity）。
- `PARAM_FACTORY_RESET`/`PARAM_LOAD_DEFAULTS`の実機動作。

## 未実装（試験対象外）

- Motor Identity / VESC profile自動適用。

## 無関係のため対象外

- `candleLight_fw`（upstream汎用gs_usb firmware、STM32F042向け、本システムと無関係）。
