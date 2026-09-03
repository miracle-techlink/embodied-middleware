# embodied-middleware 文档索引

## 入口

- [目录职责与 canonical 路径](architecture/DIRECTORY_LAYOUT.md)
- [兼容性与不变契约](architecture/COMPATIBILITY.md)
- [消息中心 README](../ros2/README.md)
- [直连采集环境搭建](ENV_SETUP.md)
- [LeRobot 采集笔记](LEROBOT_NOTES.md)
- [深度数据集说明](REBOT_SINGLE_ARM_DEPTH.md)

## 代码入口

- ROS2 启动：`ros2/launch/`
- ROS2 录制与看门狗：`ros2/runtime/`
- ROS2 体检和运维：`ros2/admin/`
- ROS2 Python 包：`ros2/src/middleware/middleware/`
- LeRobot 插件源：`lerobot_plugins/plugins/`
- LeRobot 插件安装器：`lerobot_plugins/installers/`
- 直连采集工具：`tools/acquisition/`
- 环境与硬件工具：`tools/environment/`、`tools/hardware/`
- 诊断工具：`tools/diagnostics/`

旧入口保留在 `ros2/` 根、`scripts/`、`lerobot_plugins/` 根，作为兼容 wrapper，
不承载业务实现。
