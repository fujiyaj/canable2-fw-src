# G431モーションコントローラ + ROS2 bridge 設計要約

最終更新: 2026-10-05

## この文書の位置づけ

CatchRobo-r09の`vesc_controller_node`と、専用G431ファームウェア（`~/canable2-fw-src`、`atcan-firmware`ブランチ、このファームウェア側の最新commitは`afe4754`）の設計基準。今後の変更は実装前に本書・`test_status.md`・`open_issues.md`を読み、変更後に更新すること。

**重要な前回版からの修正**: 本書の前バージョンは`~/canable2-fw-src`を読まずに作成されたため、G431内部のcascade/fault/watchdog/flash persistenceをすべて「未確認」としていた。これは誤り。`~/canable2-fw-src`のソースコードと、このリポジトリでの実機検証（1台のVESCをmotor0に接続した単軸試験）により、以下は確定事項として確認済みである。確認できなかった事項のみ「未確認」とする。

この文書内で「〜確認済み」とする根拠は次の2種類のいずれかである。
- **コード根拠**: `~/canable2-fw-src`または本リポジトリのソースコードを直接参照した（ファイル名を明記）。
- **実機根拠**: このセッション内で実際のG431 + 1台のVESC（motor0接続）でテストした結果。

また、本リポジトリには`candleLight_fw/`という別ディレクトリがあるが、これは upstream の汎用`gs_usb`互換CAN-USBアダプタファームウェア（STM32F042向け、README上STM32G431は未対応）であり、**専用G431モーションコントローラファームウェアとは無関係**。このワークスペースに置かれているだけで、本システムの設計とは関係しない。以下では扱わない。

## 対象システム

```
ROS2 (vesc_controller_node)
    | USB CDC, binproto v2 (COBS + CRC16/CCITT-FALSE)
G431 (canable2-fw-src, atcan-firmware branch)
    | CAN bus, VESC標準プロトコル (SET_CURRENT, STATUS/STATUS_4/STATUS_5)
VESC (最大4台、motor_index 0..3)
```

## 責務分担

| 層 | 責務 |
|---|---|
| ROS2 application | target position/velocity/current、profile/override指定、enable/disable、fault clear |
| `vesc_controller_node` | serial所有、ASCII→binary handshake、protocol version確認、host watchdog/telemetry rate設定、heartbeat、4軸command再送（50Hz）、telemetry集約・regroup、診断poll、再接続、**PIDゲインは一切送らない** |
| G431 firmware | position/velocity/current cascade、profile(台形)生成、CAN I/O（500Hz/motor）、host/per-motor watchdog評価、fault評価・system fault集約、flash persistence |
| VESC | SET_CURRENT実行、STATUS/STATUS_4/STATUS_5でfeedback（erpm/current/duty、temp/pid_pos、tachometer/voltage）を返す |

根拠: `~/canable2-fw-src/src/control.c`冒頭コメント、`src/cr09/src/vesc_controller_node.cpp`冒頭コメント（"G431側がcascade全体を担当するのでROS2側はPIDを実装しない"という本プロジェクトの明示的設計判断）。

## 4軸構成

`MOTOR_COUNT=4`（`~/canable2-fw-src/inc/control.h`等でハードコード）。

- `param_scope_t`はSYSTEM/MOTORの2分類。SYSTEM-scopeは明示列挙（`PARAM_DEVICE_ID`, `PARAM_FW_VERSION`, `PARAM_PROTOCOL_VERSION`, `PARAM_MOTOR_COUNT`, `PARAM_HOST_TIMEOUT_MS`, `PARAM_HOST_TIMEOUT_ACTION`, `PARAM_CONTROL_OVERRUN_COUNT`, `PARAM_CONTROL_DT_MAX_US`, `PARAM_SYSTEM_FAULT_ACTIVE/LATCHED`, `PARAM_CAN_BITRATE`, `PARAM_SAVE`, `PARAM_LOAD_DEFAULTS`, `PARAM_FACTORY_RESET`, `PARAM_CDC_TX_*`）。それ以外は既定でMOTOR-scope（`params.c`の`param_scope()`）。
- **既存レジスタIDは4軸化時も変更していない**。motor_index引数の追加で対応（このプロジェクトの明示的設計判断、本書冒頭の経緯）。
- `vesc_can_id`（0xFF=未設定、重複チェック対象外）でCAN送信先IDとmotor_indexを対応付け。受信側は`motor_index_from_vesc_id(sender_id)`で逆引き（`vesc_can.c`）。
- `vesc_can_set_current()`は`vesc_can_id==0xFF`の未設定motorに対してno-op（CAN帯域の浪費防止、このセッション内で発見・修正済みの実バグ）。

### CAN送信周期（確認済み・コード根拠）

`~/canable2-fw-src/src/main.c`:
- 制御ループは1kHz（`HAL_GetTick()`由来のSysTick、コメントに明記）。毎tickで4軸すべての`control_step()`を実行し、その後`control_recompute_system_fault()`を1回実行。
- VESCへの`vesc_can_set_current()`送信は**1kHzそのままではなく、2軸ずつ交互に1ms tickへ分散**：偶数tickでmotor0・motor2、奇数tickでmotor1・motor3。結果として**各motorは500Hzで新しいCAN mailbox値を得る**（4-VESC/1Mbpsバス設計の想定値、`params.h`の`PARAM_ANGLE_SPAN_MOTOR_REV`コメント参照とmain.cコメントに明記）。
- fault/disabled状態のmotorも同じ500Hzで0Aを再送する（one-shotではない）。
- CAN TX自体はハードウェアmailbox 3段（`SRAMCAN_TFQ_NBR`、`stm32g4xx_hal_fdcan.c`にハードコード）＋ソフトウェア側1段のsingle-slot mailbox/motor。前回値がまだ送信されていないうちに新しい値で上書きされた回数を`CAN_TX_CURRENT_OVERWRITE_COUNT`としてカウント（CANハードウェア送信失敗ではなく、mailbox上書きの意味。`vesc_can.c`冒頭コメント）。

4台同時接続時の実測jitter/round-robin挙動自体は未検証（motor0に1台のVESCのみ接続して試験、下記「未検証事項」参照）。

## 制御モードとcascade（確認済み・コード根拠 + 実機根拠）

`control_source_t`: `NONE=0`, `CURRENT=1`, `VELOCITY=2`, `POSITION=3`（`MotorCommand.msg`とbinprotoで共通）。

`compute_source_output()`（`control.c`）の演算：

```
CURRENT:   output = target_current + current_ff

VELOCITY:  output = pi_stage_step(velocity_ref, actual_velocity,
                                   vel_kp, vel_ki, vel_kd, vel_integral_limit)
                     + current_ff

POSITION:  v_ref = pi_stage_step(position_ref, actual_position,
                                  pos_kp, pos_ki, pos_kd, pos_integral_limit)
           v_ref += velocity_ff
           if velocity_limit > 0: v_ref = clamp(v_ref, -velocity_limit, velocity_limit)
           velocity_ref = v_ref   # 以降は毎tickVELOCITY段の入力として使われる
           output = pi_stage_step(velocity_ref, actual_velocity,
                                   vel_kp, vel_ki, vel_kd, vel_integral_limit)
                     + current_ff
```

`pi_stage_step()`:
```
error = ref - actual
d = (error - prev_error) / dt
if ki > 0 and integral_limit > 0:
    integral += ki * error * dt
    integral = clamp(integral, -integral_limit, integral_limit)
else:
    integral = 0   # ki/limitが0なら隠れたfeedforwardを残さない
return kp * error + integral + kd * d
```

**重要な確認済み事実（実機）**: `velocity_limit`パラメータは二重の意味を持つ。(1) POSITION段のv_ref出力クランプ、(2) **全control_sourceで有効な`actual_velocity`に対するハード過速度fault閾値**（`update_faults()`: `if velocity_limit>0 and |actual_velocity|>velocity_limit: FAULT_BIT_OVER_SPEED`）。VELOCITY試験時に「安全マージン」として設定した値がそのまま過速度フォルト閾値になり、試験レンジと衝突して自爆した事例をこのセッションで確認済み。

実機で確認した具体的な挙動（motor0、1台のVESC、設定`vel_kp=0.30, vel_ki=0`固定で機能検証のみ実施。これはチューニングではなく「カスケードが実装されパラメータ変更が正しく反映される」ことの確認が目的）：
- CURRENTモード：`effective_current_cmd`は常にROS2側の指令値と正確に一致。`actual_current`（VESC側計測）との符号・大きさの不一致は低速/停止付近のVESC側センシング特性によるものと判断（ブリッジの不具合ではない）。0.5A指令で実回転（41.29 rad/s）を確認。
- VELOCITYモード：`vel_kp=0.10〜0.25`では4〜10秒与えても静止摩擦を超えられず発進せず（`effective_current_cmd`はkp×誤差で一定値に収束するのみ）。`vel_kp=0.30`で発進（0.06〜0.6秒、スティックスリップを伴う）、定常はP制御のみのため目標の6〜7割程度に留まる定常偏差あり。`vel_kp=0.35`ではスティックスリップが明らかに悪化。
- POSITIONモード：誤差→`pos_kp`→`velocity_ref`→`vel_kp/ki`→`effective_current_cmd`の計算経路が実測値と一致することを確認。`profile_enable=false`では`effective_position_ref`が即座に目標値へジャンプ、`profile_enable=true`では中間値を経て滑らかにランプすることを確認（詳細は次節）。

ゲイン・カスケードの最終チューニング値は本書の確定仕様に含めない（実機・実負荷での再調整が前提、`test_status.md`参照）。

## Profile（台形プロファイル、確認済み・コード根拠 + 実機根拠）

`control.c`の`trapezoidal_step(ref, target, vel_max, acc_max, dec_max, dt)`:

```
err = target - ref
dist = |err|
stop_dist = vel_max^2 / (2 * dec_max)
step_vel = vel_max if dist > stop_dist else sqrt(2 * dec_max * dist)
step = sign(err) * step_vel * dt
return target if |step| >= dist else ref + step
```

**既知の未実装事項（コード内に明記されたTODO）**: `profile_acc_max`は実装されていない。`step_vel`は`vel_max`（または終端減速で制限された速度）へ直接ジャンプし、加速度制限によるランプアップはない。つまり現状は「減速のみ制限された台形」であり、真の台形プロファイルではない。無負荷/低電流でのCAN・角度デコード確認には問題ないが、実慣性負荷では移動開始時に速度コマンドのステップとして感じられる、とコード自身が注記している。

`profile_enable`の有無による分岐（`update_internal_reference()`）:
- `profile_enable=false`かつ`pos_ref_catching_up=false`の場合: `position_ref = clamp(target_position, position_min, position_max)`（即座にジャンプ）。**実機確認済み**：target_position=+0.1指令の最初のテレメトリサンプルで既に`effective_position_ref=0.1`（中間値なし）。
- `profile_enable=true`、または`pos_ref_catching_up=true`（後述）の場合: 上記`trapezoidal_step()`で毎tickランプ。**実機確認済み**：target_position=±0.1指令で中間値（例: 0.0→0.0912→0.1000）を経て到達。

`pos_ref_catching_up`: POSITION以外からPOSITIONへ遷移した瞬間に`true`になり、`profile_enable`の値に関わらず強制的にプロファイル整形を有効化する（bumpless transitionの一部、次節参照）。目標に到達したら`false`に戻る。

## Bumpless transition（確認済み・コード根拠 + 実機根拠）

`begin_transition(old_src, new_src)`（`control.c`）:
- 出力は`transition_alpha`を0→1へ`transition_blend_ms`かけて線形補間し、旧sourceの出力と新sourceの出力をブレンドする。
- VELOCITYへ新規に入る場合: `velocity_ref = actual_velocity`（誤差0からスタート）、ki>0かつintegral_limit>0なら`vel_integral`を現在の`output_current`から逆算してseed、そうでなければ0。
- POSITIONへ新規に入る場合: `position_ref = actual_position`（誤差0からスタート）、`pos_ref_catching_up = true`（前節）、ki>0かつintegral_limit>0なら`pos_integral`を`actual_velocity`から逆算してseed、そうでなければ0。
- CURRENTは内部状態を持たないため、出力ブレンドのみでbumplessになる。

**実機確認済み**（50Hz生テレメトリで検証）:
- CURRENT→VELOCITY→POSITION→VELOCITY→CURRENTの全遷移で、`effective_current_cmd`の不連続ジャンプなし。約100〜170msで新カスケードの出力へ滑らかに収束（`transition_blend_ms`既定値相当）。
- POSITION進入時、`effective_position_ref`は目標値や古い値へ飛ばず、**その時点の`actual_position`の直近値から開始**し、そこから目標へランプする。

## Override（確認済み・コード根拠 + 実機根拠）

`resolve_active_source()`（`control.c`）:
```
if override_enable and override_source != NONE:
    if override_source == POSITION: return control_source  # 防御的拒否
    return override_source
return control_source
```
POSITION overrideは禁止（`params_write()`側でも拒否される想定とコード注記）。

**実機確認済み**: CURRENT/VELOCITY overrideそれぞれで、override中は元のcontrol_source（POSITION）の内部状態（`position_ref`等）が完全に凍結され競合しない。override解除時は前節のbumpless機構がそのまま使われ、`position_ref`は`actual_position`の近傍から再開して元の目標へ滑らかに復帰する。`effective_current_cmd`もoverride進入・解除の両方で不連続ジャンプなし。

## Enable / disable（確認済み・コード根拠 + 実機根拠）

`PARAM_ENABLE (0x1000)`はWRITE_PARAM専用、`control_request_enable()`/`control_request_disable()`へ転送（`params.c`で直接ハンドルしない設計、"state machine invariantsを直接registerで迂回できないように"という明記コメント）。

`control_request_enable()`の拒否条件（`PARAM_ERR_CONFIG`、`FAULT_BIT_CONFIG_ERROR`をlatch）:
- `current_limit <= 0`
- `vesc_can_id == 0xFF`（未設定）
- `!vesc_alive`（VESCからのSTATUS未受信）
- 他motorとの`vesc_can_id`重複
- `host_watchdog_required && (host_timeout_ms==0 || !host_rx_seen_since_armed)`
- 要求sourceに必要なfeedbackが未到達（CURRENT/VELOCITYは`velocity_feedback_valid`、POSITIONは`position_feedback_valid`）

既にSTATE_FAULTの場合は`PARAM_ERR_STATE`（"must clear_fault first"）。

`control_request_disable()`は`pr->enable = 0`のみ。

**`FAULT_BIT_CONFIG_ERROR`の重要な契約**: enable拒否時にだけ立つvisibility用のbitであり、**単独ではRUNNING→FAULTへのstate遷移を絶対に発生させない**（state=IDLEのまま残る）。このセッション中に発見・修正した実バグ: 以前の「bystander fault」修正が`fault_latched != 0`で単純にFAULT判定していたため、current_limit未設定による拒否だけでFAULTへ落ちてしまっていた。修正は`(fault_latched & ~FAULT_BIT_CONFIG_ERROR) != 0`へのマスク（`control.c`、commit`afe4754`）。

ROS2側（`vesc_controller_node`）: enableを連続commandに含めず、`motor_controller/set_enable`サービスから明示操作。node起動直後のcache済みcommandは全軸`NONE/0`。

## Fault model（確認済み・コード根拠 + 実機根拠）

### Local fault（motor自身）

`update_faults()`は`pr->enable != 0`（armed）の時のみ評価する。disarmed時は`fault_active = 0`を即座にセットし、**`fault_latched`は変更しない**（sticky history、"live readingではない"と明記）。

評価されるbit（`FAULT_BIT_*`、`params.h`）:

| bit | 値 | 意味 |
|---|---:|---|
| CAN_TIMEOUT | 1<<0 | VESCからのSTATUSが`can_timeout_ms`以内に来ない |
| COMMAND_TIMEOUT | 1<<1 | 予約（host_timeout_actionに統合、bit互換のため維持） |
| OVER_CURRENT | 1<<2 | `actual_current`が`current_limit * OVER_CURRENT_TRIP_RATIO`超、debounce付き |
| OVER_SPEED | 1<<3 | `velocity_limit>0`かつ`\|actual_velocity\|>velocity_limit`（**全control_sourceで評価、実機確認済み**） |
| POS_LIMIT | 1<<4 | `actual_position`が`position_min`/`position_max`の外（**全control_sourceで評価、command自体とは独立。実機確認済み**） |
| VESC_FAULT | 1<<5 | VESCがCAN経由で自身のfault codeを報告 |
| OVER_TEMP | 1<<6 | `max_temperature>0`かつ`actual_temperature`超過 |
| UNDER_VOLTAGE | 1<<7 | `min_bus_voltage>0`かつ`actual_voltage`未達 |
| OVER_VOLTAGE | 1<<8 | `max_bus_voltage>0`かつ`actual_voltage`超過 |
| CONFIG_ERROR | 1<<9 | enable拒否専用。**RUNNING→FAULTを強制しない**（前節） |
| FEEDBACK_INVALID | 1<<10 | 要求sourceが必要とするfeedbackが無効（limitが有効なのに該当feedbackが未到達の場合も含む） |
| RUNTIME_CONFIG | 1<<11 | host watchdog必須なのに`host_timeout_ms==0`（defense-in-depth、通常到達しない想定） |

`control_clear_fault()`: `fault_latched = fault_active`（現在activeなbitだけ残す）。**根本原因が解消していなければ、clear_fault呼び出し自体は成功してもfault_latchedは即座に再セットされる**（disarmed状態でfault_active=0になっていれば本当に0クリアされる）。POS_LIMITで実機確認済み: 範囲外のままclear_fault→disabled中はfault_active=0に落ちるためclear成功→範囲外のまま再enable→次tickで即再fault。

### System fault

`control_recompute_system_fault()`（main.cから毎tick1回呼ばれる）が、各motorの`fault_active`/`fault_latched`の非zeroをbit iとして`sys_status.system_fault_active/latched`へ集約。

`control_step()`の`interlocked`判定: `(st->fault_active != 0) || (sys_status.system_fault_active != 0)` → このmotorを問答無用で`state=FAULT`、`active_source=NONE`、`output=0`にする（原因が自分でなくても、= bystander）。

**bystanderは自動復帰する**: `fault_latched`はTHIS MOTORの原因でしか立たないため、原因motor自身の`clear_fault`だけで`interlocked`がfalseに戻った瞬間、bystanderは自分の`clear_fault`なしで自動的に復帰する（コードコメントに明記、このセッションの「bystander fault」バグ修正の経緯そのもの）。

ROS2側は`PARAM_SYSTEM_FAULT_ACTIVE (0x7030)`/`PARAM_SYSTEM_FAULT_LATCHED (0x7031)`をpollして`SystemStatus`に出す。

## Host watchdog（確認済み・コード根拠 + 実機根拠）

二段構成（`params.h`「Two-tier host watchdog」コメント）:

- **system-wide**: `host_last_rx_ms`。`HEARTBEAT`と`CONTROL_COMMAND`の**両方**が`control_notify_host_rx()`を呼ぶ（`binproto.c`: HEARTBEAT処理内と、`control_apply_command()`成功後の両方で呼ばれる）。HEARTBEATは独立したsupervisor信号ではない。
- **per-motor**: `motor_control_t::last_command_ms`/`command_seen`。**該当motor宛の`CONTROL_COMMAND`成功時のみ**更新（HEARTBEATでは更新されない）。

`update_faults()`: `host_timeout_active = (host_age > host_timeout_ms) OR (command_seen AND cmd_age > host_timeout_ms)`。両チェックは**同じ**`host_timeout_ms`/`host_timeout_action`を使う（per-motor専用の別レジスタはない）。

**実機確認済み**（生プロトコルで厳密タイミング制御して検証）: 特定motorへの`CONTROL_COMMAND`だけを止め、`HEARTBEAT`を50ms間隔で継続送信し続けても、約300ms後にそのmotorの出力は強制的に0へ落ちた。すなわちHEARTBEATの継続だけではper-motor watchdogは救済されない。

**`host_rx_seen_since_armed`の罠（実機で踏んで確認済み）**: `binproto_enter_binary_mode()`は`control_require_host_watchdog(true)`を呼び、これは`host_rx_seen_since_armed = false`にリセットする（"must prove liveness again"）。つまりbinary mode突入直後は、`HEARTBEAT`または`CONTROL_COMMAND`を一度も送らずに`enable`しようとすると`PARAM_ERR_CONFIG`で拒否される。`binproto_force_ascii_mode()`は`control_require_host_watchdog(false)`（ASCII bench testは`host_timeout_ms=0`を許容）。

`host_timeout_action_t`（`params.h`）:

| 値 | 名前 | 挙動 |
|---:|---|---|
| 0 | HOLD | velocity/current sourceはtarget 0相当にフォールバック。position targetはそのまま（position holdは変更不要） |
| 1 | CURRENT_ZERO | `active_source=NONE`、出力0。**`enable`はクリアしない** |
| 2 | FAULT | `FAULT_BIT_COMMAND_TIMEOUT`を立ててFAULT状態へ |
| 3 | DISABLE | `pr->enable=0`、`active_source=NONE`、`state=IDLE`、出力0。**faultにはしない**（`fault_latched`は変更しない） |

## `HOST_TIMEOUT_DISABLE`（このセッションで追加・実機検証済み）

**追加の経緯**: `CURRENT_ZERO`は出力だけを0にし`enable`flagを残すため、通信断からの復帰時に新しい`set_enable`呼び出しなしで`CONTROL_COMMAND`の再送が再開するだけでmotorが再駆動してしまうことを実機（SIGKILLテストと実USB抜線テストの両方）で確認した。これを塞ぐために`host_timeout_action_t`へ`DISABLE=3`を追加（`control.c`/`params.h`/`params.c`、`write_u8`の許容範囲を`[HOLD, FAULT]`から`[HOLD, DISABLE]`へ拡張、commit`afe4754`）。既存のHOLD/CURRENT_ZERO/FAULTは変更していない（互換）。

自己持続的な実装: `pr->enable=0`にした後は、次tick以降`control_step()`冒頭の`if (!pr->enable)`分岐がそのまま効き、`host_timeout_active`自体が後でfalseに戻っても状態はIDLE/NONE/0を維持し続ける。再駆動には明示的な`PARAM_ENABLE=1`（=ROS2側`set_enable(true)`）が必須。

**実機検証済み（2系統）**:
1. **SIGKILL試験**（ホストプロセスの異常終了→FDクローズ→ノード再起動、USB bus resetは伴わない）: 300ms超待機後、生プロトコルで`PARAM_ENABLE=0`, `PARAM_STATE=1(IDLE)`, `fault_active=0`, `fault_latched=0`を確認。ノード再起動後、`set_enable`を一切呼ばずに`CONTROL_COMMAND`(0.1A)を送っても出力0のまま。`set_enable=true`で初めて動作。
2. **真のUSB抜線試験**（`/dev/ttyACM0`の消失・再enumeration込み）: ノードログで`write failed: Input/output error`→`link lost, will reconnect`→`open(/dev/ttyACM0) failed: No such file or directory`→再接続後`connected and configured`の完全なhandshake再実行を確認。`reconnect_count`は1増加。再接続直後、`connected=true`, `state=1(IDLE)`, `active_source=0(NONE)`, `effective_current_cmd=0.0`, `fault_active=0`。`set_enable`なしで`CONTROL_COMMAND`(0.1A)を送っても出力0のまま、`set_enable=true`で初めて動作。

**付随して確認した事実**: 実USB抜線はこのUSB給電ボードのMCU電源断（リセット）も伴うらしく、未保存（PARAM_SAVEしていない）のRAM値（例: 一時的に広げた`position_min/max`）は電源断で最後に保存した値へ戻った。保存済みの値（`current_limit`, `vel_kp`等）は正しく保持された。

## USB接続・再接続時の安全設計（確認済み・コード根拠 + 実機根拠）

`vesc_controller_node`のhandshake（`open_and_handshake()`）順序:

1. port open、DTR=true、RTS=false。
2. USB CDC settle待ち。
3. binary modeのままか、短い`GET_INFO`でprobe。失敗ならASCII `binary\n`送信→`OK BINARY`確認（フォールバック）。
4. `GET_INFO`でprotocol_version==2を確認。不一致ならport close、接続失敗。
5. `PARAM_HOST_TIMEOUT_MS`書込み。
6. `PARAM_HOST_TIMEOUT_ACTION=HOST_TIMEOUT_DISABLE(3)`書込み。
7. **（このセッションで追加した二次防御）4軸すべてへ`PARAM_ENABLE=0`を明示write**。1軸でもACK失敗ならport close、通常通信へ移行しない。
8. `SET_TELEMETRY_RATE`書込み。
9. 全て成功後だけ`connected_=true`、steady stateへ移行。

read/write失敗時: `connected_=false`、port close、`reconnect_count++`、`reconnect_period_s`（既定1秒）ごとに上記handshakeを再試行。USB blipとG431 rebootを区別しない。

**二層防御の理由**（ファイルヘッダ"Safety notes"、このセッションで追記）: firmware側`HOST_TIMEOUT_DISABLE`が本命、ROS2側の毎接続force-disableは「host timeoutを一度も踏まずに通信が切れたケース」（例: 電源が入ったまま通信だけ止まった場合にfirmware側watchdogがまだ発火していない間に再接続が先に来た、等の想定）への保険。

キャッシュされたcommand（target値）自体は再接続でクリアされない。再enableする上位側は、その時点のtarget値が安全かを確認する責任を持つ、というのがROS2側コメントの明示事項（このnodeは`NONE/0`へのリセットを行わない）。

## Flash persistence（確認済み・コード根拠 + 実機根拠、内部実装は一部未確認）

`params.c`: `motor_config_t`は`motor_flash[MOTOR_COUNT]`、`bus_config_t`は`bus_flash`という変数名だが、**これはRAM上の構造体**であり、「flash保存の対象になり得る」という命名。明示的に`PARAM_SAVE`（SYSTEM-scope、trigger）を書き込むまでは**RAM上の値のみ**で、MCUリセット（リフラッシュ、または実USB抜線に伴う電源断）で compiled-in defaultへ戻る。

**実機で繰り返し確認した事実**: `current_limit`, `vel_kp/ki`, `pos_kp/ki`, `position_min/max`, `velocity_limit`等は、一度`PARAM_SAVE`した後はリフラッシュ・電源断を跨いで正しく復元される。`PARAM_SAVE`せずRAM書込みだけだった値は、リフラッシュ・電源断で default値（`DEFAULT_*`、`params.h`）に戻る。このセッション中、この挙動により`current_limit`等を複数回再設定する手間が生じた。

`vesc_controller_node`がhandshakeごとに書く`host_timeout_ms`/`host_timeout_action`/`telemetry_rate`は、ノード側では`PARAM_SAVE`を呼ばない（毎接続時にRAM上へ書き直す設計、永続化に依存しない）。

**未確認（flash_store.cの内部実装）**: flash layout、schema/version、CRC、wear leveling、書込み中断時のatomicity、`PARAM_FACTORY_RESET`/`PARAM_LOAD_DEFAULTS`の正確な動作範囲。`flash_store_save()`のエラーパス（`PARAM_ERR_IO`、flash erase/program失敗）が存在することはコードで確認したが、内部詳細は未調査。

## Binary USB protocol v2（確認済み・コード根拠、既存README/binproto.hppと一致）

- raw header: version(u8) + type(u8) + sequence(u16 LE) + payload_len(u16 LE)、末尾にCRC-16/CCITT-FALSE（poly 0x1021, init 0xFFFF, non-reflected, LE付加）。
- 全体をCOBS encodeし`0x00`区切り。
- raw上限128byte。整数/floatは明示little-endian pack（struct memcpyはwire境界で使わない）。

| Type | 値 | 用途 |
|---|---:|---|
| GET_INFO | 0x01 | protocol/fw情報 |
| HEARTBEAT | 0x02 | host activity、RTT計測 |
| READ_PARAM/WRITE_PARAM/PARAM_RESPONSE | 0x10/0x11/0x12 | parameter access |
| CONTROL_COMMAND | 0x20 | 1motor分の連続command、**no ACK on success**。拒否時のみ`ERROR`応答あり（`control_apply_command()`の戻り値） |
| SET_TELEMETRY_RATE/TELEMETRY | 0x30/0x31 | telemetry設定/受信 |
| ACK/ERROR | 0x7E/0x7F | 応答 |

**実機確認済み**: `target_position`が`position_min/max`外のCONTROL_COMMANDを送ると、`vesc_controller_node`は読み取らない（CONTROL_COMMANDの戻りエラーを意図的に無視する設計、`dispatch_frame()`コメント"a CONTROL_COMMAND ERROR is not acted on further here"）が、ファームウェア側は`ERROR(0x7F)`応答+`error_code=5(PARAM_ERR_RANGE)`を返し、該当motorの内部target値は書き換えない（生プロトコルスクリプトで直接確認）。

## Telemetryと診断（確認済み・コード根拠、既存設計どおり）

TELEMETRYは1周期につき1motor1frame、計4frame。同一周期の4frameは共通`telemetry_cycle_sequence`を持つ。`vesc_controller_node`の`TelemetryCycleBuffer`が`group_deadline_ms`（既定20ms）まで待って4frameを1つの`MotorTelemetryArray`へ再構成し、間に合わない軸は`valid=false`でpublish（他軸をブロックしない）。

ROS2側が収集する診断: CRC/COBSエラー数、cycle sequence gap、telemetry age、reconnect_count、heartbeat RTT、G431自身の`cdc_tx_drop_count`/`cdc_tx_bytes_pending`（2Hzでpoll）、4軸分の`can_tx_current_overwrite_count`、`system_fault_active/latched`。

## ROS2 bridge interface（確認済み・コード根拠）

`vesc_controller_node`は通常のROS2 node（`ros2_control::hardware_interface`ではない）。

| 名前 | 型 | 方向 |
|---|---|---|
| `motor_controller/command` | `MotorCommandArray` | sub |
| `motor_controller/telemetry` | `MotorTelemetryArray` | pub |
| `motor_controller/status` | `SystemStatus` | pub |
| `motor_controller/set_enable` | `SetMotorEnable` | service |
| `motor_controller/clear_fault` | `ClearMotorFault` | service |

既定parameter: `/dev/ttyACM0`, 115200baud, heartbeat 20Hz, command 50Hz, telemetry 50Hz, host_timeout_ms 300, group_deadline_ms 20, reconnect_period_s 1.0, handshake_timeout_ms 500, status 2Hz, diag_poll 2Hz（`config/vesc_controller.yaml`）。

同じCatchRobo-r09内に、別経路として`atcan_bridge_node`/`socketcan_bridge_node`やVESCへ直接CAN commandを送るteleopノード群が存在するが、これらは本bridgeとは別の制御経路であり、本書の対象外（運用時に二重送信しないこと）。

## Motor Identity / VESC profile自動適用（未実装・将来機能）

現時点で`vesc_controller_node`はMotor Identity/VESC profileの読出し・検証・適用を一切行わない。対応するmessage/service/parameter/schemaも存在しない。少なくとも次を設計確定する必要がある（本書では未確定の候補として記録するのみ）:

- motor_index・`vesc_can_id`・motor型式・`pole_pairs`・`gear_ratio`のidentity schema。
- VESCごとの制限値・feedback設定schema。
- 照合のみ/明示適用/自動適用のモードとデフォルト。
- 不一致時のenable拒否方針。
- 適用中の全軸disable維持と再起動時の非再開手順。

## 未検証事項（詳細は`test_status.md`）

- 4台同時接続時のCAN bus負荷・500Hz/axis実測jitter（motor0の1台のみで試験）。
- 数時間規模の長時間soak（約9分の部分試験のみ、ユーザー指示で中断）。
- flash_store.cの内部実装（schema/CRC/wear leveling/atomicity）。
- Motor Identity / VESC profile自動適用（未実装）。

## 互換性上の制約

- binproto protocol version 2固定。不一致では通信継続しない。
- register IDは`~/canable2-fw-src`の`params.h`と数値同期必須。
- motor数4、payload layout、telemetry layoutの変更はfirmware/ROS message/nodeを同時更新する。
- enableを連続commandへ統合しない。
- PID gain/trajectory生成をROS側へ移さない。
- `candleLight_fw`は無関係（STM32F042向け汎用ファーム、本システムとは別物）。

## Decision History

- ROS2側は薄いbridgeに保ち、制御loopとtrajectoryをG431へ集約（USB/ROS jitterから制御周期を切り離す）。
- 4軸化にあたり既存レジスタIDを変更せず、motor_index引数追加で対応（変更量抑制）。
- システムfault escalationはbystanderを自動復帰させる設計とし、原因motor自身のclear_fault一回で済むようにした。
- `FAULT_BIT_CONFIG_ERROR`はenable拒否専用とし、RUNNING→FAULTを強制しない契約を明文化・バグ修正（このセッション）。
- HEARTBEATを独立supervisorにせず、CONTROL_COMMANDと同じhost活動信号として扱う一方、CONTROL_COMMANDにはper-motor freshness watchdogを別途持たせた。
- host timeout actionを`CURRENT_ZERO`のままにせず`HOST_TIMEOUT_DISABLE`を追加し、ROS2側の再接続時force-disableと二重化した（このセッションで発見した設計ギャップの修正）。
- telemetry欠落時は全軸待ちで止めず、deadline後に欠落軸のみinvalidにしてpublish。
- Motor Identity / VESC profile自動適用は意図的に未実装のまま保留（探索段階）。

## 根拠ファイル

- `~/canable2-fw-src/src/control.c`, `src/binproto.c`, `src/vesc_can.c`, `src/params.c`, `src/main.c`, `inc/params.h`, `inc/control.h`, `inc/vesc_can.h`, `inc/binproto.h`
- `src/cr09/src/vesc_controller_node.cpp`
- `src/cr09/include/cr09/binproto.hpp`
- `src/cr09/msg/*.msg`, `src/cr09/srv/*.srv`
- `src/cr09/config/vesc_controller.yaml`
- `readme.md`
- このセッション内の実機試験ログ（motor0 + VESC 1台）
