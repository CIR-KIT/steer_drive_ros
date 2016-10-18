#step_back_and_max_steer_recovery
## 仕様
- ざっくりしたものはissue参照→[Recovery の挙動について #10](https://github.com/CIR-KIT/third_robot_pkg/issues/10)
- 実際には，これよりもうちょっと色々やっている．
  - 左に障害物があったら，「右旋回」→「直進」→「左旋回」を行うことで進路クリアを図っている
  - とか
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
    # back(初回後退時の速度[m/s]と移動距離[m])
    linear_vel_back     : -0.3
    step_back_length    : 1.0
    # steer(旋回時の速度[rad/s]と目標回転角度[rad])
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
[ INFO] [1476805022.459020448, 15000.030000000]: Recovery behavior will clear layer obstacles
[ INFO] [1476805022.517777693, 15000.090000000]: Initialized with only_single_steering = false
[ INFO] [1476805022.517814034, 15000.090000000]: Initialized with recovery_trial_times = 3
[ INFO] [1476805022.517844810, 15000.090000000]: Initialized with obstacle_patience = 0.30
[ INFO] [1476805022.517860301, 15000.090000000]: Initialized with linear_vel_back = -0.30, step_back_length = 1.00
[ INFO] [1476805022.517876789, 15000.090000000]: Initialized with linear_vel_steer = 0.30, angular_vel_steer = 0.50, turn_angle = 1.50
[ INFO] [1476805022.517908477, 15000.090000000]: Initialized with linear_vel_forward = 0.30, step_forward_length = 1.00
```

- パラメータサーバに次のように登録されていればOK
```bash
/move_base/step_back_and_max_steer_recovery/angular_speed_steer
/move_base/step_back_and_max_steer_recovery/linear_vel_back
/move_base/step_back_and_max_steer_recovery/linear_vel_forward
/move_base/step_back_and_max_steer_recovery/linear_vel_steer
/move_base/step_back_and_max_steer_recovery/obstacle_patience
/move_base/step_back_and_max_steer_recovery/only_single_steering
/move_base/step_back_and_max_steer_recovery/step_back_length
/move_base/step_back_and_max_steer_recovery/step_forward_length
/move_base/step_back_and_max_steer_recovery/trial_times
/move_base/step_back_and_max_steer_recovery/turn_angle
```

- リカバリ行動に入ったら以下のようなメッセージが出る．
```bash
[ INFO] [1476805029.597262999, 15007.170000000]: *****************************************************
[ INFO] [1476805029.597325668, 15007.170000000]: **********Start StepBackAndSteerRecovery!!!**********
[ INFO] [1476805029.597349570, 15007.170000000]: *****************************************************
[ INFO] [1476805061.093148562, 15038.650000000]: attempting step back
[ INFO] [1476805064.545240776, 15042.100000000]: complete step back
[ INFO] [1476805065.515806324, 15043.070000000]: min_l = 11.25, min_r = 4.60
[ INFO] [1476805065.515948508, 15043.070000000]: attempting to turn left at the 1st turn
[ INFO] [1476805068.216827905, 15045.770000000]: complete the 1st turn
[ INFO] [1476805068.216925820, 15045.770000000]: attemping step forward
[ INFO] [1476805071.568092876, 15049.120000000]: complete step forward
[ INFO] [1476805071.568256384, 15049.120000000]: attempting second turn
[ INFO] [1476805074.719016710, 15052.270000000]: complete second turn
[ INFO] [1476805075.669610158, 15053.220000000]: continue recovery because the robot couldn't get clearance
[ INFO] [1476805075.671413290, 15053.220000000]: attempting step back
[ INFO] [1476805079.121436749, 15056.670000000]: complete step back
[ INFO] [1476805080.087343376, 15057.630000000]: min_l = 11.80, min_r = 11.30
[ INFO] [1476805080.087466251, 15057.630000000]: attempting to turn left at the 1st turn
[ INFO] [1476805082.783008088, 15060.330000000]: complete the 1st turn
[ INFO] [1476805082.783089460, 15060.330000000]: attemping step forward
[ INFO] [1476805086.134868653, 15063.680000000]: complete step forward
[ INFO] [1476805086.134918763, 15063.680000000]: attempting second turn
[ INFO] [1476805088.986284833, 15066.530000000]: complete second turn
[ INFO] [1476805089.973853268, 15067.510000000]: continue recovery because the robot got clearance
[ INFO] [1476805089.973912937, 15067.510000000]: *****************************************************
[ INFO] [1476805089.973933753, 15067.510000000]: **********Finish StepBackAndSteerRecovery!!**********
[ INFO] [1476805089.973950421, 15067.510000000]: *****************************************************

```
