# jetpilot_vesc_interface

## Purpose

JetPilot の正規化 control command を VESC driver 用 topic に変換する vehicle interface package です。上位の planning/control/operation は `jetpilot_msgs/msg/ControlCommand` だけを扱い、この package が eRPM、brake current、servo position へ変換します。

`publish_description:=true`では、`base_link`を親とするcamera、EVS、thremoの固定TFも公開します。各センサーの取付位置はlaunch引数で変更できます。

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `control_cmd_to_vesc_node` | `control_cmd_to_vesc_node` | 正規化指令をVESC motor/servo commandへ変換する |
| `vesc_driver_node` | `vesc_driver_node` | VESC hardwareと通信するexternal driver |
| `robot_state_publisher` | `robot_state_publisher` | 任意でvehicle mountのstatic TFをpublishする |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `control_cmd_to_vesc_node` | `/vehicle/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | node内の`/control_cmd`をlaunchで標準remap |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `control_cmd_to_vesc_node` | `/commands/motor/speed` | `std_msgs/msg/Float64` | Reliable / Volatile | throttle/reverseをeRPMへ変換した指令 |
| `control_cmd_to_vesc_node` | `/commands/motor/brake` | `std_msgs/msg/Float64` | Reliable / Volatile | brake current [A] |
| `control_cmd_to_vesc_node` | `/commands/servo/position` | `std_msgs/msg/Float64` | Reliable / Volatile | steering servo position |

### TF

| Node | Parent | Child | Mode | Description |
| --- | --- | --- | --- | --- |
| `robot_state_publisher` | `base_link` | configured sensor frames | Static publish | `publish_description:=true`時のcamera/EVS/thermal mount |

## Parameters

eRPM、brake current、servo変換、deadband、watchdogの標準値は
[`config/vesc_interface.param.yaml`](config/vesc_interface.param.yaml)を参照してください。

## Assumptions / Known limits

- VESC driverのtopic namespaceが標準`/commands/*`と一致している必要があります。
- servoの符号・offset、eRPM上限、brake currentは実車ごとに校正します。
- command timeout時はspeed 0とneutral servoを出しますが、hardware側watchdogも独立して必要です。

`/control_cmd` は best-effort `KeepLast(1)` で購読します。VESC driver へ向けた topic は相対名なので、launch namespace に応じて配置できます。

## Conversion algorithm

1. 新しい `/control_cmd` を保存し、受信時刻を記録する
2. `max_command_age_s` を超えた場合は speed 0、servo neutral を publish する
3. steering は `servo_offset + steering * servo_gain` で servo position に変換し、`servo_min` から `servo_max` に clamp する
4. brake が deadband を超える場合は speed 0 と `brake * max_brake_current_amps` を publish する
5. brake が無い場合は throttle/reverse の大きい方を採用し、forward/reverse eRPM へ変換する

throttle、reverse、brake には個別 deadband があります。servo の符号は車体組付けで変わるため、最初はタイヤを浮かせて `servo_gain` と `servo_offset` を確認してください。

## How to launch

```bash
ros2 launch jetpilot_vesc_interface vesc_interface.launch.xml
```

固定TFも同時に起動する場合:

```bash
ros2 launch jetpilot_vesc_interface vesc_interface.launch.xml \
  publish_description:=true \
  description_camera_frame:=camera_link \
  description_evs_frame:=evs_link \
  description_thremo_frame:=thremo_link
```

`jetpilot_system_launch` から使う場合は vehicle interface をこの package に差し替えます。

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  vehicle_interface_pkg:=jetpilot_vesc_interface \
  vehicle_interface_launch:=launch/vesc_interface.launch.xml \
  publish_vehicle_evs_description:=true \
  publish_vehicle_thremo_description:=true
```

EVSとthremoの初期姿勢はどちらも`base_link`に対して`xyz=0 0 0`、`rpy=0 0 0`です。実機の取付位置を測定したら、`vehicle_description_evs_*`と`vehicle_description_thremo_*`を更新してください。`scripts/bringup.sh`のVESCモードでは両TFを自動的に有効化します。
