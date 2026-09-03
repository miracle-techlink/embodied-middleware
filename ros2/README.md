# middleware — reBot 单臂采集栈的 ROS2 消息中心

把原来"单进程直连硬件"的 lerobot 采集链路拆成**设备驱动节点 + topic**,schema 模仿
AgilexCobotMagic 的 msg_center JSON。控制回路(主臂→映射→从臂)由节点自持 100Hz,
与录制循环彻底解耦;录制只是被动订阅。

> **目录分类(2026-09-03 起)**:脚本按 `launch/`(启动)/`runtime/`(录制与守护)/
> `admin/`(体检运维)分层;Python 包按 `core/`、`backends/`、`nodes/{arms,leaders,
> cameras,control,benchmarks}`、`maintenance/` 分层。根目录的 start_*/record_*/
> rebot_* 全部是兼容 wrapper,旧用法不破。详见 ../docs/architecture/DIRECTORY_LAYOUT.md
> 与 ../docs/architecture/COMPATIBILITY.md。

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

- **话题注册表**:`src/middleware/config/rebot_single_arm.json`
  (照 Agilex schema:`enable / topic_name / type_name / default_fps`)。新增设备 = 改 JSON + 写节点。
- **精调行为归属**:限位/夹爪直驱/退出回零在 arm 节点(贴硬件);绝对映射/启动 ramp 在 map
  节点;StarAI 4ms 单发在 leader 节点。全部复用 lerobot 插件类,一行没重写。

## 跑法

```bash
# 终端 1:消息中心(Ctrl-C 全停,arm 会平滑回零再卸力矩)
~/middleware/start_teleop.sh               # 采集/遥操作模式:五节点全起
~/middleware/start_infer.sh                # 推理模式:只起 arm+双相机
                                               # 不起 leader/teleop_map —— follower 指令源只有策略
# (两者都支持 stop 子参数;引擎仍是 start_msg_center.sh teleop|infer|stop,wrapper 不复制逻辑)

# 终端 2:录制(观测走 topic,数据集格式与 rebot_follower 完全一致)
REPO_ID=用户名/数据集名 TASK="任务描述" \
  bash ~/middleware/record_ros2.sh
# 闸门式(每条固定时长→当场选留/丢→回车下一条;直连版 record_rebot_gated 的消息中心等价物,
# 回零/对齐/存活探测换成 topic 语义,见 record_rebot_gated_ros2.py 头注释。
# 0 位确认仅整轮开头一次;倒计时期间从臂冻结,结束那一刻才恢复跟随并开录,无对齐等待):
REPO_ID=用户名/数据集名 TASK="任务描述" \
  bash ~/middleware/record_rebot_gated_ros2.sh
# (脚本内补齐 source/PYTHONPATH/conda env;发现 joint_cmd 无发布者 = 没起消息中心或在
#  infer 模式 → 自动优雅停当前布局(arm 回零)→ 后台拉起 teleop → 等就绪再录。AUTO_SWITCH=0 关。
#  EPISODES/EP_TIME/NO_DISPLAY/RESUME 见脚本头)
# 手打等价命令:
#   source /opt/ros/jazzy/setup.bash
#   export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$HOME/middleware/src/middleware
#   conda activate data_collect
#   lerobot-record \
#     --robot.type=ros2_rebot_follower --robot.id=follower1 \
#     --teleop.type=ros2_rebot_teleop --teleop.id=rebot_leader \
# --dataset.repo_id=<repo> --dataset.fps=30 --dataset.rgb_encoder.vcodec=h264 ...
# ⚠ flag 用下划线(--dataset.single_task),连字符版 draccus 不认(踩过)
```

相机掉线/重插:重启对应节点即可,不用动录制进程——这就是消息中心的初衷。

## 日志与体检

```bash
~/middleware/rebot_doctor.sh          # 一键体检(只读):五节点/发布者数/实测频率/近期日志错误/CAN/磁盘
~/middleware/rebot_doctor.sh infer    # 按 infer 模式体检
~/middleware/rebot_watch.sh           # 实时日志窗:latest 会话全聚合,错误红/警告黄(Ctrl-C 退)
~/middleware/rebot_watch.sh -a        # 连历史录制留档一起跟踪
```

- 消息中心每次启动建 `logs/sess_<时间>_<模式>/`,`logs/latest` 软链指向当前会话;
  `/tmp/<节点>.log` 保留为兼容软链(旧习惯不破)
- 两个 record 脚本输出 tee 到 `logs/record[_gated]_<时间>.log`(`PYTHONUNBUFFERED`
  保证交互提示实时落盘;终端显示不受影响)——终端回滚丢了也有全量留档
- 频率探针 `rebot_rate.py`:这版 `ros2 topic hz` 没有 QoS 参数,RELIABLE 默认订阅
  收不到 BEST_EFFORT 流;探针 QoS 匹配,先等首帧再开窗计数
- rerun:record 脚本默认 `--display_data=true`——这版 lerobot 的默认是 False,
  不显式传 true 永远不弹(踩过 2026-08-20);NO_DISPLAY=1 关

## 运维

```bash
ros2 topic hz /rebot/follower/joint_state     # 验 200Hz
ros2 bag record -a -o session_xxx             # 原始流留底(含图像)
~/middleware/start_msg_center.sh stop      # 只停
```

性能基线(2026-08-13,Jetson Thor):topic 单跳 p99 0.96ms;leader 100Hz std 0.18ms;
joint_state 200Hz std 0.10ms;record 循环观测+动作 0.07ms。

## 注意

- **USB 拓扑(2026-08-13 已整改)**:Orbbec 已单走一路直插(Bus1 根口,但协商仍
  480M——线材是 USB2 的,换 USB3 线可上 Bus2 5000M);USB2 hub 只剩前视/PCAN/CH341
  三个轻载。母仓 SETUP_LOG 明确"适配器直插 Jetson,别经拓展坞"。
- **autosuspend 已持久关闭**:`/etc/udev/rules.d/99-rebot-usb-noautosuspend.rules`
  覆盖 Orbbec/前视/PCAN/CH341 四设备,重插重启都生效。
- **USB 软复位**:`src/middleware/scripts/usb_reset.py [orbbec|pcan|ch341|frontcam]`
  (需 sudo),USBDEVFS_RESET 免拔插;复位后重启对应节点即可。
- **UVC 节点号会漂**:前视相机这次是 /dev/video0(旧脚本写死 /dev/video10)。用
  `FRONT_CAM=/dev/v4l/by-path/...` 环境变量指定稳定路径更靠谱。
- **QoS**:所有 topic 是 BEST_EFFORT keep-last 1,订阅方必须匹配(RELIABLE 订阅会
  被 DDS 判不兼容静默收不到)。
- **SIGTERM 安全**:arm 节点注册了 SIGTERM 优雅退出(回零→卸力矩),`kill`/`timeout`
  都安全;但 `kill -9` 依然会砸臂,别用。
- **回退**:旧直连链路(rebot_follower + starai_to_rebot_leader)原样保留,
  不起消息中心就能用。
- 已知差异:闸门录制"回车等待"期间,原链路从臂冻结,消息中心模式下从臂**继续跟随
  主臂**(map 节点自持;record_rebot_gated_ros2 会话起手会先冻结)。
  **录完自动回零(2026-08-20)**:往 `/rebot/teleop/go_home` 发 Empty,map 节点把 leader
  视作零、以专用慢速(参数 `go_home_ramp_deg_per_step`,默认 0.5°/步@100Hz≈50°/s)从当前
  位滑回 home;rearm / enable 恢复自动退出该模式。手动回零:`rebot_go_home.py [--wait]`。
  临时冻结:`rebot_enable.py false`。录制退出(q/^C/正常跑完):**冻结遥操作 →
  go_home 归零(等到位)→ 落盘**。record 脚本会检测 map 节点有无 go_home 订阅,
  旧版自动重启消息中心换新。
