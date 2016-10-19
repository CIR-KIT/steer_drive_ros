#step_back_and_max_steer_recovery
## 仕様
- ざっくりしたものはissue参照→[Recovery の挙動について #10](https://github.com/CIR-KIT/third_robot_pkg/issues/10)
- 実際には，これよりもうちょっと色々やっている．
  - 左に障害物があったら，「右旋回」→「直進」→「左旋回」を行うことで進路クリアを図っている
- 移動量、回転量で制御．制御量はパラメータで設定可能．
- 移動中リアルタイムにcostmapを参照し，衝突前に停止する．許容距離はパラメータで設定可能．
- 「後退→旋回→直進→反対旋回」or「後退→旋回」をパラメータで選択可能.

## 挙動
- 動画サンプル→https://www.youtube.com/watch?v=j6GID9XuiiU

## 使い方
### 事前設定
- まずは`catkin_make`する．

- navigation の `planner.yaml`(NavfanROSやTrajectoryPlannerROSの設定が入っているファイル)に，以下のパラメータを追加する．

``` yaml:
recovery_behaviour_enabled: true

recovery_behaviors:
  - {name: conservative_reset, type: clear_costmap_recovery/ClearCostmapRecovery}
  - {name: step_back_and_max_steer_recovery, type: step_back_and_max_steer_recovery/StepBackAndMaxSteerRecovery}
  - {name: aggressive_reset, type: clear_costmap_recovery/ClearCostmapRecovery}
  
step_back_and_max_steer_recovery:
    # 最初の一回だけ旋回したい場合にtrue
    only_single_steering: true
    # リカバリ行動の試行回数[回]
    trial_times         : 3
    # 障害物までの許容距離[m]．
    #-- 移動中に，移動方向に対して最も近い障害物がこの距離以内に出現したら停止する．
    obstacle_patience   : 0.3
    #-- 移動中に，障害物を確認する頻度[回/sec]
    obstacle_check_frequency: 5.0
    # back(初回後退時の速度[m/s]と移動距離[m])
    linear_vel_back     : -0.3
    step_back_length    : 1.0
    # steer(旋回時の直進速度[m/s]，回転速さ(環境に寄って±が変わる)[rad/s]と目標回転角度[rad])
    linear_vel_steer    : 0.3
    angular_speed_steer : 0.5
    turn_angle          : 1.5
    # forward(旋回→直進→旋回の直進時の速度[m/s]と目標移動距離[m])
    linear_vel_forward  : 0.3
    step_forward_length : 1.0
```

### 使用時挙動
- `move_base`立ち上げ時に以下のメッセージが出ていれば初期化成功
```bash
[ INFO] [1476868343.792576518, 2448.560000000]: Initialized with only_single_steering = false
[ INFO] [1476868343.792646641, 2448.560000000]: Initialized with recovery_trial_times = 3
[ INFO] [1476868343.792669900, 2448.560000000]: Initialized with obstacle_patience = 0.50
[ INFO] [1476868343.792698568, 2448.560000000]: Initialized with obstacle_check_frequency = 6.00
[ INFO] [1476868343.792745858, 2448.560000000]: Initialized with linear_vel_back = -0.30, step_back_length = 8.00
[ INFO] [1476868343.792777209, 2448.560000000]: Initialized with linear_vel_steer = 0.90, angular_speed_steer = 0.50, turn_angle = 1.00
[ INFO] [1476868343.792811151, 2448.560000000]: Initialized with linear_vel_forward = 0.30, step_forward_length = 1.00
```

- パラメータサーバに次のように登録されていればOK
```bash
rosparam list | grep step_back_and_max_steer_recovery
/move_base/step_back_and_max_steer_recovery/angular_speed_steer
/move_base/step_back_and_max_steer_recovery/linear_vel_back
/move_base/step_back_and_max_steer_recovery/linear_vel_forward
/move_base/step_back_and_max_steer_recovery/linear_vel_steer
/move_base/step_back_and_max_steer_recovery/obstacle_check_frequency
/move_base/step_back_and_max_steer_recovery/obstacle_patience
/move_base/step_back_and_max_steer_recovery/only_single_steering
/move_base/step_back_and_max_steer_recovery/step_back_length
/move_base/step_back_and_max_steer_recovery/step_forward_length
/move_base/step_back_and_max_steer_recovery/trial_times
/move_base/step_back_and_max_steer_recovery/turn_angle

```

- リカバリ行動に入ったら以下のようなメッセージが出る．
  - 1秒に1回，移動横行と障害物までの最短距離を出力する．
  - 障害物発見時はWARNINGで表示．
```bash
[ INFO] [1476868375.829523527, 2480.590000000]: *****************************************************
[ INFO] [1476868375.829690887, 2480.590000000]: **********Start StepBackAndSteerRecovery!!!**********
[ INFO] [1476868375.829753797, 2480.590000000]: *****************************************************
[ INFO] [1476868375.829794323, 2480.590000000]: ==== 1 th recovery trial ====
[ INFO] [1476868383.643012049, 2488.400000000]: min dist to obstacle = 1.59 [m] in BACKWARD
[ INFO] [1476868384.840226745, 2489.590000000]: min dist to obstacle = 0.92 [m] in BACKWARD
[ WARN] [1476868386.036249305, 2490.790000000]: obstacle detected at BACKWARD
[ WARN] [1476868386.036298028, 2490.790000000]: min dist to obstacle = 0.48 [m] in BACKWARD
[ INFO] [1476868386.036321913, 2490.790000000]: complete step back
[ INFO] [1476868387.007138030, 2491.760000000]: min_l = 12.22 [m], min_r = 5.98 [m]
[ INFO] [1476868387.007252925, 2491.760000000]: attempting to turn left at the 1st turn
[ INFO] [1476868387.610307113, 2492.360000000]: min dist to obstacle = 11.09 [m] in FORWARD_LEFT
[ INFO] [1476868388.005182918, 2492.760000000]: complete the 1st turn
[ INFO] [1476868388.005297723, 2492.760000000]: attemping step forward
[ INFO] [1476868388.611079357, 2493.360000000]: min dist to obstacle = 8.73 [m] in FORWARD
[ INFO] [1476868389.212381216, 2493.960000000]: min dist to obstacle = 7.70 [m] in FORWARD
[ INFO] [1476868390.413571989, 2495.160000000]: min dist to obstacle = 5.98 [m] in FORWARD
[ INFO] [1476868391.010836290, 2495.760000000]: complete step forward
[ INFO] [1476868391.010889155, 2495.760000000]: attempting second turn
[ INFO] [1476868391.013244061, 2495.760000000]: min dist to obstacle = 25.86 [m] in FORWARD_RIGHT
[ INFO] [1476868392.008923908, 2496.760000000]: complete second turn
[ INFO] [1476868392.993798006, 2497.740000000]: break recovery because the robot got clearance
[ INFO] [1476868392.993863847, 2497.740000000]: *****************************************************
[ INFO] [1476868392.993893442, 2497.740000000]: **********Finish StepBackAndSteerRecovery!!**********
[ INFO] [1476868392.993917387, 2497.740000000]: *****************************************************

```
