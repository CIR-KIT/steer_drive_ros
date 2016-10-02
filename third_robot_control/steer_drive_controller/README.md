## Rocker Bogie Controller ##

Controller for a rocker bogie drive mobile base.

Detailed user documentation can be found in the controller's [forest_robot_project wiki page](https://github.com/Nishida-Lab/forest_robot_project/wiki/rocker_bogie_controller).

- 左方向
![左方向](https://github.com/Nishida-Lab/forest_robot_project/blob/add_fr01_rocker_bogie_controller/fr01_control/rocker_bogie_controller/.fig/rocker_bogie_right.png)

- 右方向
![右方向](https://github.com/Nishida-Lab/forest_robot_project/blob/add_fr01_rocker_bogie_controller/fr01_control/rocker_bogie_controller/.fig/rocker_bogie_left.png)

## デバッグ開発の手順
- パスを通す

```bash
source path.bash
```

- パスを通したコンソールでQtCreatorを起動する
```bash
qtcreator
```

- [QtCreatorでROSのパッケージをビルド&デバッグ実行する](http://qiita.com/MoriKen/items/ea41e485929e0724d15e)にしたがってビルドする．実行オプションで`steer_drive_test`を選択する．

- gazeboを実行する
```bash
roslaunch third_robot_gazebo third_robot_world.launch 
```

- コントローラ動作用の最低限の設定を行うlaunchファイルを実行する
```bash

```
