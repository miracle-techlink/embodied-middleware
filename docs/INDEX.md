# embodied-middleware 文档索引

## 入口

- [目录职责与 canonical 路径](architecture/DIRECTORY_LAYOUT.md)
- [兼容性与不变契约](architecture/COMPATIBILITY.md)
- [消息中心 README](../README.md)

## 代码入口

- ROS2 启动：`launch/`
- ROS2 录制与看门狗：`runtime/`
- ROS2 体检和运维：`admin/`
- ROS2 Python 包：`src/middleware/middleware/`
- 直连采集工具：`tools/acquisition/`
- 环境与硬件工具：`tools/environment/`、`tools/hardware/`
- 诊断工具：`tools/diagnostics/`

旧入口仍保留在仓库根目录，作为兼容 wrapper，不承载业务实现。
