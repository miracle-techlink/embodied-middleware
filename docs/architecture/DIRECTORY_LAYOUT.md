# 目录职责与 canonical 路径

本仓按职责分层:ROS2 消息中心(`ros2/`)、LeRobot 插件(`lerobot_plugins/`)、
直连采集工具(`tools/`)、文档(`docs/`)。每层内部再按"启动/运行/运维"
或"插件源/安装器/补丁"细分。旧入口保留为透传 wrapper,canonical 实现只有一份。

## ros2/(= live 仓 ~/middleware)

```text
ros2/
├── launch/       启动:start_msg_center(引擎)、start_teleop、start_infer、start_rig(profile)
├── runtime/      运行期:record_ros2、record_rebot_gated_ros2(.sh/.py)、watchdog、rig_watchdog(profile)
├── admin/        运维体检:rebot_doctor、rebot_watch、rebot_rate、rebot_enable、rebot_go_home、rig_doctor(profile)
├── src/middleware/
│   ├── middleware/
│   │   ├── core/             topic_registry(JSON 话题注册表读取)
│   │   ├── backends/         硬件后端层,节点只剩 ROS 逻辑;两支臂同协议对称
│   │   │   ├── rebot/        RebotFollower 薄壳(行为零重写,透传 lerobot 精调实现)
│   │   │   └── piper/        PiPER 后端抽象(SDK 真机 / MuJoCo 仿真),不含 ROS2 Node
│   │   ├── nodes/
│   │   │   ├── arms/         rebot_arm_node、piper_arm_node(从臂驱动节点)
│   │   │   ├── leaders/      starai_leader_node(主臂示教节点)
│   │   │   ├── cameras/      orbbec_node、uvc_node(相机节点)
│   │   │   ├── control/      teleop_map_node(主臂→从臂映射/闸门/回零)
│   │   │   └── benchmarks/   msg_center_bench(链路基准)
│   │   └── maintenance/      usb_reset 等节点外运维库(canonical)
│   ├── config/               rebot_single_arm.json(rebot 栈)、piper_msg_center.json(piper 栈)
│   ├── scripts/              外部兼容 wrapper(usb_reset.py → maintenance)
│   └── setup.py              find_packages + entry points(命令名不变)
└── 旧根入口 wrapper          start_*.sh、record_*.sh、watchdog.sh、rebot_*.sh/.py
```

### 判定规则

- 涉及 ROS2 Node 的 → `nodes/` 下按设备角色(arms/leaders/cameras/control)放;
- 硬件交互(不依赖 rclpy)的 → `backends/` 按厂商放;设备驱动来自 lerobot 插件的
  也在这里做薄壳(rebot),使节点层对所有臂只见统一协议(connect/disconnect/
  get_state/send_cmd);
- 注册表/QoS/schema 等节点公共依赖 → `core/`;
- 被 shell 脚本和节点共同调用的运维实现 → `maintenance/`(shell 入口在 `scripts/` 留 wrapper);
- 一次性起停编排 → `launch/`;录制/守护长跑 → `runtime/`;只读体检/手工干预 → `admin/`。
- 多机配置只放仓库根 `profiles/rigs/*.yaml`;本机序列号/设备路径放 git 忽略的
  `profiles/local/overrides.yaml`。新机器使用 `start_rig.sh`、`rig_doctor.sh`、
  `rig_watchdog.sh`,旧 reBot 固定入口继续兼容。

## lerobot_plugins/

```text
lerobot_plugins/
├── plugins/          待复制进 LeRobot 源码树的插件源码
│   ├── robots/rebot_follower/
│   ├── teleoperators/starai_violin_leader/
│   ├── teleoperators/starai_to_rebot_leader/
│   └── cameras/orbbec/
├── installers/       install_plugins.sh、install_orbbec.sh、install_depthfix.sh
├── patches/depthfix/ 补丁说明与版本约束
├── manifests/        plugin_manifest.yaml(插件→目标路径注册行清单)
└── install*.sh       旧根入口 wrapper
```

`plugins/` 只放 Python 源码;安装/注册逻辑只存在于 `installers/`,从仓库根计算
路径、读 `LEROBOT_SRC`、幂等复制。

## tools/(直连链路,不经消息中心)

```text
tools/
├── acquisition/    teleop_rebot、record_rebot、record_rebot_gated(.sh/.py)
├── environment/    setup_env(全新机器从零搭环境)
├── hardware/
│   ├── can/        setup_rebot_can、install_can_udev
│   ├── usb/        check_usb、usbreset_orbbec、install_usb_noautosuspend、install_no_camera_probe
│   └── power/      maxn_lock、estop_release
├── diagnostics/    probe_arm、profile_loop(.sh/.py)
└── (旧 scripts/ 根入口 wrapper 保留)
```

判定:采集动作 → `acquisition/`;装环境 → `environment/`;动硬件配置 →
`hardware/` 按总线再分;只读测量 → `diagnostics/`。
