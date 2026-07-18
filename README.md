# jetpilot_vesc_interface

JetPilot の正規化 control command を VESC driver 用 topic に変換する vehicle interface package です。上位の planning/control/operation は `jetpilot_msgs/msg/ControlCommand` だけを扱い、この package が eRPM、brake current、servo position へ変換します。

## Node

| Node | 役割 |
| --- | --- |
| `control_cmd_to_vesc_node` | `/control_cmd` を VESC driver の motor/servo command topic に変換する |

## Topic契約

| 方向 | Topic | 型 | 用途 |
| --- | --- | --- | --- |
| input | `/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | mux 後の正規化指令 |
| output | `commands/motor/speed` | `std_msgs/msg/Float64` | throttle/reverse を eRPM に変換した速度指令 |
| output | `commands/motor/brake` | `std_msgs/msg/Float64` | brake を current [A] に変換した制動指令 |
| output | `commands/servo/position` | `std_msgs/msg/Float64` | steering を servo position に変換した操舵指令 |

`/control_cmd` は best-effort `KeepLast(1)` で購読します。VESC driver へ向けた topic は相対名なので、launch namespace に応じて配置できます。

## Conversion algorithm

1. 新しい `/control_cmd` を保存し、受信時刻を記録する
2. `max_command_age_s` を超えた場合は speed 0、servo neutral を publish する
3. steering は `servo_offset + steering * servo_gain` で servo position に変換し、`servo_min` から `servo_max` に clamp する
4. brake が deadband を超える場合は speed 0 と `brake * max_brake_current_amps` を publish する
5. brake が無い場合は throttle/reverse の大きい方を採用し、forward/reverse eRPM へ変換する

throttle、reverse、brake には個別 deadband があります。servo の符号は車体組付けで変わるため、最初はタイヤを浮かせて `servo_gain` と `servo_offset` を確認してください。

## 起動

```bash
ros2 launch jetpilot_vesc_interface vesc_interface.launch.xml
```

`jetpilot_system_launch` から使う場合は vehicle interface をこの package に差し替えます。

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  vehicle_interface_pkg:=jetpilot_vesc_interface \
  vehicle_interface_launch:=launch/vesc_interface.launch.xml
```
