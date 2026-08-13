# rebot_msg_center — reBot 单臂采集栈的 ROS2 消息中心

把原来"单进程直连硬件"的 lerobot 采集链路拆成**设备驱动节点 + topic**,schema 模仿
AgilexCobotMagic 的 msg_center JSON。控制回路(主臂→映射→从臂)由节点自持 100Hz,
与录制循环彻底解耦;录制只是被动订阅。

## 拓扑

```
starai_leader_node ──/rebot/leader/joint_state(100Hz)──┐
                                                       ↓
                                   teleop_map_node(映射+ramp+闸门,100Hz)
                                                       ↓
                                   /rebot/follower/joint_cmd
                                                       ↓
                                          rebot_arm_node ──→ CAN(RobStride)
                                                       ↓
                                   /rebot/follower/joint_state(200Hz)─┐
orbbec_node ──/rebot/wrist/{color,depth}/compressed(30Hz)─────────────┤
uvc_node    ──/rebot/front/color/compressed(30Hz)─────────────────────┤
                                                                      ↓
                                        lerobot bridge:ros2_rebot_follower
                                        + ros2_rebot_teleop → lerobot-record
```

- **话题注册表**:`src/rebot_msg_center/config/rebot_msg_center_single_arm.json`
  (照 Agilex schema:`enable / topic_name / type_name / default_fps`)。新增设备 = 改 JSON + 写节点。
- **精调行为归属**:限位/夹爪直驱/退出回零在 arm 节点(贴硬件);绝对映射/启动 ramp 在 map
  节点;StarAI 4ms 单发在 leader 节点。全部复用 lerobot 插件类,一行没重写。

## 跑法

```bash
# 终端 1:消息中心(Ctrl-C 全停,arm 会平滑回零再卸力矩)
~/rebot_ros2_ws/start_msg_center.sh

# 终端 2:录制(观测走 topic,数据集格式与 rebot_follower 完全一致)
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$HOME/rebot_ros2_ws/src/rebot_msg_center
conda activate data_collect
lerobot-record \
  --robot.type=ros2_rebot_follower --robot.id=follower1 \
  --teleop.type=ros2_rebot_teleop --teleop.id=rebot_leader \
  --dataset.repo_id=<repo> --dataset.fps=30 --dataset.rgb_encoder.vcodec=h264 ...
```

相机掉线/重插:重启对应节点即可,不用动录制进程——这就是消息中心的初衷。

## 运维

```bash
ros2 topic hz /rebot/follower/joint_state     # 验 200Hz
ros2 bag record -a -o session_xxx             # 原始流留底(含图像)
~/rebot_ros2_ws/start_msg_center.sh stop      # 只停
```

性能基线(2026-08-13,Jetson Thor):topic 单跳 p99 0.96ms;leader 100Hz std 0.18ms;
joint_state 200Hz std 0.10ms;record 循环观测+动作 0.07ms。

## 注意

- **UVC 节点号会漂**:前视相机这次是 /dev/video0(旧脚本写死 /dev/video10)。用
  `FRONT_CAM=/dev/v4l/by-path/...` 环境变量指定稳定路径更靠谱。
- **QoS**:所有 topic 是 BEST_EFFORT keep-last 1,订阅方必须匹配(RELIABLE 订阅会
  被 DDS 判不兼容静默收不到)。
- **SIGTERM 安全**:arm 节点注册了 SIGTERM 优雅退出(回零→卸力矩),`kill`/`timeout`
  都安全;但 `kill -9` 依然会砸臂,别用。
- **回退**:旧直连链路(rebot_follower + starai_to_rebot_leader)原样保留,
  不起消息中心就能用。
- 已知差异:闸门录制"回车等待"期间,原链路从臂冻结,消息中心模式下从臂**继续跟随
  主臂**(map 节点自持)。下一条开录时 record_rebot_gated 的 rearm_ramp hook 仍然有效。
  要冻结可以往 /rebot/teleop/enable 发 false。
