# 单臂 reBot · 双视角带深度 · 遥操作 + 数据采集

一支 **StarAI Violin 主臂** → 一支 **Seeed reBot B601-RS 从臂**,原生跑在官方
`lerobot-teleoperate` / `lerobot-record` 上,双视角:
**腕部 Orbbec(彩色 + 对齐深度)** + **一路 USB 摄像头(第二视角,彩色)**。

## 这套东西由什么组成

| 组件 | 类型 | 说明 |
|---|---|---|
| `rebot_follower` (robot) | 继承官方 `seeed_b601_rs_follower` | 单臂 reBot 原生接口。**只加深度**:`use_depth` 的相机多出 `observation.images.<cam>_depth` (H,W,1 uint16 mm)。`joint_directions` 归一为 1 → action 与 observation 同帧。退出前平滑回零位(坐姿)再卸力矩。 |
| `starai_to_rebot_leader` (teleop) | 包 `starai_violin_leader` | 把 leader 映射成 **reBot 关节空间目标**(home 锚点增量 + 夹爪 remap)。所以录进数据集的 `action` = reBot 空间,与 `observation.state` 同帧,利于训练。 |
| `orbbec` (camera) | 已有插件 | 彩色 + 软件 D2C 对齐深度。 |

**动作/观测空间(录进数据集)**
- `observation.state` = reBot 7 关节:`shoulder_pan.pos … wrist_roll.pos, gripper.pos`(原始电机角,度)
- `observation.images.wrist` = 腕部 Orbbec 彩色 (480,640,3)
- `observation.images.wrist_depth` = 腕部对齐深度 (480,640,1) uint16 毫米 ← **带深度**
- `observation.images.front` = USB 摄像头第二视角 (480,640,3)
- `action` = reBot 关节空间目标(与 `observation.state` 同帧)

## 安装(一次)

```bash
cd ~/platform/10-acquisition/code/rebot_datacollect/lerobot_plugins
LEROBOT_SRC=~/lerobot-datacollect bash install_plugins.sh    # 注册 rebot_follower + starai_to_rebot_leader + ROS2 bridge
LEROBOT_SRC=~/lerobot-datacollect bash install_orbbec.sh     # 注册 orbbec 相机(带深度必需)
```
> lerobot 升级后插件会被覆盖,重跑上面两条即可。

## 现场准备

1. **CAN 起来**(PCAN = can5):
   ```bash
   sudo ip link set can5 up type can bitrate 1000000 restart-ms 100
   ```
2. **标定 reBot**(按 `rebot_follower` 类型,做一次,存到
   `.../calibration/robots/rebot_follower/follower1.json`;之后自动加载不再问):
   ```bash
   ~/miniconda3/envs/lerobot/bin/lerobot-calibrate \
     --robot.type=rebot_follower --robot.port=can5 \
     --robot.can_adapter=socketcan --robot.id=follower1
   ```
   零位 = 官方手册的坐姿(sit-down),夹爪闭合。**退出回零就是回到这个姿态**。
   (直接跑 teleop/record 时若没标定文件,也会自动引导你标一次。)
3. **leader 标定**:已有 `starai_violin_leader/leader1.json` 就不用再标。没有则:
   ```bash
   ~/miniconda3/envs/lerobot/bin/lerobot-calibrate \
     --teleop.type=starai_violin_leader --teleop.port=/dev/ttyCH341USB0 --teleop.id=leader1
   ```
4. **相机链路**:两个 Orbbec + USB 摄像头目前都在 **USB2**。USB2 上 Orbbec 必须 `mjpg`
   (脚本已带),且 `FULL_FRAME_REQUIRE + 深度 10fps` 会把整体拉到 ~10fps。**要 30fps 深度,
   把腕部 Orbbec 插到 USB3 口(Bus 002 的 hub)。** 卡死时 `python scripts/usbreset_orbbec.py`。

## 跑遥操作(先干这个,校方向)

```bash
PY=~/miniconda3/envs/data_collect/bin/python bash ~/platform/10-acquisition/code/rebot_datacollect/tools/acquisition/teleop_rebot.sh
```
- 起来会先**平滑 ramp 到 home** `[sp0, sl85, ef100, wf5, wy0, wr0]`(靠 `max_relative_target=8°/步`,无跳变)。
- rerun 里能看到 `wrist`(彩色)、`wrist_depth`(深度)、`front`(USB)三路 + 关节曲线。
- **方向校准**:某关节反了 → 加 `--teleop.flip="..."`(1–6,逗号分隔;默认 `3,4,5`)。
  夹爪开合反了 → 把 `--teleop.grip_close_deg` / `--teleop.grip_open_deg` 两个值对调。
- Ctrl-C:reBot 自动回零位(坐姿)再卸力矩。**手别挡。**

常用覆盖(环境变量):`WRIST_CAM`(Orbbec 序列号)、`FRONT_CAM`(USB 节点,默认 `/dev/video4`)、
`NO_DEPTH=1`(只彩色)、`NO_CAM=1`(不开相机)。也可直接追加 `--key=val` 传给 CLI。

## 采数据集

```bash
PY=~/miniconda3/envs/data_collect/bin/python \
REPO_ID=你的用户名/rebot_pick TASK="pick up the cube" EPISODES=20 \
bash ~/platform/10-acquisition/code/rebot_datacollect/tools/acquisition/record_rebot.sh
```
- 录完后**务必抽检深度**:数据集 meta 里应有 `observation.images.wrist_depth` 且被标为深度图。
  快速看一集:
  ```bash
  ~/miniconda3/envs/lerobot/bin/python -c "from lerobot.datasets.lerobot_dataset import LeRobotDataset as D; d=D('你的用户名/rebot_pick'); print([k for k in d.meta.features], '| depth_keys=', d.meta.depth_keys)"
  ```
- 上传:加 `PUSH=true`。

## 调参速查(全在 `--teleop.*` / `--robot.*`)

| 参数 | 默认 | 作用 |
|---|---|---|
| `--teleop.flip` | `3,4,5` | 翻转的臂关节(1–6) |
| `--teleop.scale` | `1.0` | 主→从 增量增益 |
| `--teleop.rebot_home_deg` | `[0,85,100,5,0,0]` | reBot home(各关节限位中点,避免单边关节死区) |
| `--teleop.grip_close_deg` / `grip_open_deg` | `20 / 250` | 夹爪闭/开端(度);反了就对调 |
| `--teleop.grip_clamp_deg` | `25` | 闭合端过冲(夹持力) |
| `--teleop.grip_ratio_min` | `0.62` | 主臂夹爪"捏到底"的 ratio |
| `--robot.max_relative_target` | `8.0` | 每步度数上限(起步 ramp + 限速);`null` 解除 |
| `--robot.return_home_on_exit` | `true` | 退出回零位再卸力矩 |
| `--robot.exit_home_deg` | `[0,0,0,0,0,0]` | 退出回到的姿态 |
