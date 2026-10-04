# G431モーションコントローラ + ROS2 bridge 未解決課題

最終更新: 2026-10-05

## この文書の位置づけ

**前回版からの修正**: 前バージョンの最優先課題「`~/canable2-fw-src`のソースが参照できない」は解消した（このセッションで直接読み、実機試験も行った）。cascade/profile/bumpless/override/fault/watchdog/flash persistenceの大部分は`design_summary.md`に確定事項として記載済み。本書は**実際に残っている**課題のみを記録する。

## 1. 多軸（4台同時）構成の未検証

このセッションはmotor0に1台のVESCのみ接続して試験した。以下は**コード上は実装済みだが実測していない**:

- 4台のVESCを接続した際の実際のCAN bus負荷、500Hz/axis（`main.c`の2軸交互500Hzスケジューリング、`design_summary.md`参照）の実測jitter。
- 複数motorが同時にfaultを起こした場合のsystem fault集約・bystander自動復帰の実機確認（コードロジックは確認済み、単軸では再現できない）。
- 4軸間でのUSB/CAN帯域競合、`CAN_TX_CURRENT_OVERWRITE_COUNT`の実運用閾値。

対応: 2台目以降のVESCが用意できた時点で実施する。

## 2. 長時間soak試験の未完了

約9分の部分soak（+5/-5 rad/s、ポーズ含む）のみ実施し、`+10/-10 rad/s`フェーズの途中でユーザー指示により中断した。中断までの区間ではエラーカウンタ（CRC/COBS/sequence gap/reconnect/telemetry loss）の増加なし。

対応: 数時間規模の連続運転試験を別途実施する。

## 3. POSITIONモードの最終チューニング未実施

`design_summary.md`に記載の通り、`pos_kp=10.0`等はカスケード機能確認用の一時的な値であり、実負荷でのチューニング対象ではない。現在のVELOCITY段ゲイン（`vel_kp=0.30, vel_ki=0`）もP制御のみで定常偏差が残る状態。

対応:
- 実際の機構・負荷を載せた段階で、VELOCITY段にKiを少量加えて定常偏差を詰める（セッション後半で合意していた方針）。
- POSITION側のpos_kp/ki、オーバーシュート・振動の評価。
- 低速域の静止摩擦補償（最低速度強制等）が必要か、VELOCITY側の追加試験で判断する。

## 4. flash_store.cの内部実装が未確認

`PARAM_SAVE`で値が永続化されること自体は実機で繰り返し確認したが、以下は未確認:

- flash layout、schema/version、CRC。
- 書込み粒度、wear leveling、書込み回数制限。
- 電源断中の書込み中断に対するatomicity/rollback。
- `PARAM_FACTORY_RESET`/`PARAM_LOAD_DEFAULTS`の正確な動作範囲。

対応: `~/canable2-fw-src/src/flash_store.c`を直接調査する。

## 5. Motor Identity / VESC profile自動適用（未実装）

将来機能。現時点で実装も確定schemaもない。設計タスク（優先度は未定）:

1. Motor Identityの定義: motor_index、`vesc_can_id`、motor型式、`pole_pairs`、`gear_ratio`等の候補フィールド（未確定）。
2. VESC profileの定義: 電流/速度/温度/電圧limit、CAN status rate等、どこまでをG431が所有するか。
3. 期待値/観測値/適用値を識別し差分表示するprotocol設計。
4. デフォルトは照合のみとし、自動書込みを許す条件（operator承認、全軸disable、verify、rollback）。
5. identity/profile不一致時のenable interlock方針。
6. ROS APIとbinprotoのregister map同時設計、version negotiation。

## 6. ROS2ノード側の未確認事項

- `set_enable`/`clear_fault`サービスのmotor index事前range check（コード上見当たらず、firmware側のreplyに依存している可能性）。
- `send_all_motor_commands()`（4軸分のCONTROL_COMMAND送信）で、個々の`write_raw()`失敗を即時検出せず次回readでdisconnect判定している設計が十分か。
- sequence counter / telemetry_cycle_sequenceのwrap時のgap判定。
- 自動テストが存在しない（binproto codec、telemetry regrouping、reconnect state machineの回帰検証ができない）。host-side unit/integration testの追加。

## 無関係として整理済み

- `candleLight_fw`（upstream汎用gs_usb firmware、STM32F042向け）は本システムと無関係。本システムの設計判断に影響しない。
