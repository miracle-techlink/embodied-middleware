# depthfix 补丁说明

`installers/install_depthfix.sh` 对 LeRobot 核心打的 4 处幂等补丁,绕开 pyav
构建不支持 `gray12le` numpy 转换的 bug,保住 hevc/gray12le/mp4 的深度编码设计:

1. `depth_utils.py` 编码:`VideoFrame` 构造器 + `write_u16_plane` 建帧(替 `from_ndarray`)
2. `video_utils.py` 解码:手动读 u16 plane(替 `to_ndarray(gray12le)`)
3. `video_utils.py` `get_video_info`:Codec 无 `canonical_name` 时回退 `.name`
4. `video_utils.py` `concatenate_video_files`:无 `add_stream_from_template` 时用
   `add_stream(template=)`(第 2 条 episode 起拼接视频块时触发)

## 版本约束

- 验证过的上游:lerobot `v0.6.1`(见 `tools/environment/setup_env.sh` 的默认 tag)
- pyav 15.x 实测 round-trip 正常,深度误差 = 纯 12-bit 量化(~1mm),无编码损失
- **lerobot 升级会覆盖核心文件 → 升级后重跑 `install_depthfix.sh`**
- 若上游已合入等效修复,以 anchor 未命中提示为准,先核对再手动处理

补丁本体是脚本内的 Python here-doc(锚点替换式,幂等);本目录只放说明与约束,
不放游离于安装器之外的第三份补丁副本。
