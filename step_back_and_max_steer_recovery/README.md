#step_back_and_max_steer_recovery
## 仕様
- ざっくりしたものはissue参照→[Recovery の挙動について #10](https://github.com/CIR-KIT/third_robot_pkg/issues/10)
- 実際には，これよりもうちょっと色々やっている．
  - 左に障害物があったら，「右旋回」→「直進」→「左旋回」を行うことで進路クリアを図っている
  - とか

## 挙動
- 動画サンプル→https://www.youtube.com/watch?v=j6GID9XuiiU

## 使い方
### 事前設定
- navigation の `planner.yaml`(NavfanROSやTrajectoryPlannerROSの設定が入っているファイル)に，以下のパラメータを追加する．

``` yaml:
step_back_and_max_steer_recovery:
    # リカバリ行動の試行回数
    trial_times         : 3
    # back(初回後退時の速度と移動時間)
    linear_vel_back     : -0.3
    duration_back       : 3.0
    # steer(旋回時の速度と移動時間)
    linear_vel_steer   : 0.3
    angular_vel_steer  : 0.5
    duration_steer     : 2.0
    # forward(旋回→直進→旋回の直進時の速度と移動時間)
    linear_vel_forward   : 0.3
    duration_forward     : 3.0
```

### 使用時挙動
- `move_base`立ち上げ時に以下のメッセージが出ていれば初期化成功
```bash
[ INFO] [1476491523.796292048, 1635.850000000]: Initialized with trial_times = 3
[ INFO] [1476491523.796437138, 1635.850000000]: Initialized with linear_vel_back = -0.30, duration_back = 3.00
[ INFO] [1476491523.796468232, 1635.850000000]: Initialized with linear_vel_steer = 0.30, angular_vel_steer = 0.50, duration_steer = 2.00
[ INFO] [1476491523.796495523, 1635.850000000]: Initialized with linear_vel_forward = 0.30, duration_forward = 3.00

```

- リカバリ行動に入ったら以下のようなメッセージが出る．
```bash
[ INFO] [1476491565.790291005, 1677.830000000]: *****************************************************
[ INFO] [1476491565.790390087, 1677.830000000]: **********Start StepBackAndSteerRecovery!!!**********
[ INFO] [1476491565.790435929, 1677.830000000]: *****************************************************
[ INFO] [1476491569.854152514, 1681.890000000]: min_l = 20.70, min_r = 5.55
[ INFO] [1476491577.896652466, 1689.930000000]: continue at (3.00, 0.00, 0.10) for max_time 1.00 seconds
[ INFO] [1476491581.956789414, 1693.990000000]: min_l = 13.40, min_r = 13.10
[ INFO] [1476491590.017833711, 1702.040000000]: break at (3.00, 0.00, 0.10) for max_time 1001.00 seconds
[ INFO] [1476491590.017891626, 1702.040000000]: *****************************************************
[ INFO] [1476491590.017917844, 1702.040000000]: **********Finish StepBackAndSteerRecovery!!**********
[ INFO] [1476491590.017941666, 1702.040000000]: *****************************************************

```
