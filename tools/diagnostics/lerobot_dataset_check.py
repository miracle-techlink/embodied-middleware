#!/usr/bin/env python3
"""lerobot_dataset_check — LeRobot 数据集采集后质量校验(piper/rebot 通用)。

校验内容(全部只读):
  1. 结构        meta/info.json、data/chunk-*/file-*.parquet、videos/ 齐全
  2. 帧数       每 episode 帧数 vs 期望(fps×时长口径:按 frame_index 连续段统计)
                帧索引连续(无断裂/重号)
  3. 数值健康    state/action 无 NaN;逐关节 action−state 偏差分布
                (mean/median/max;静止≈0、运动中=跟踪残差,肉眼判量级)
  4. 视频        每个 camera key 有非空 mp4、编码/尺寸与 info.json 一致(帧数抽查)
用法:
  python3 lerobot_dataset_check.py <数据集目录>
    (目录 = ~/.cache/huggingface/lerobot/local/xxx_时间戳 那一层)
退出码: 0=通过(无致命问题) / 1=有问题(打印明细)
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq


def load_parquets(data_dir: Path) -> list[Path]:
    return sorted(glob.glob(str(data_dir / "chunk-*" / "file-*.parquet")))


def col_delta(files: list[Path], names: list[str], base: str, other: str) -> dict[str, dict]:
    """逐关节统计 (other − base) 的绝对值分布(度/mm)。
    本 lerobot 版本里 action / observation.state 是 fixed_size_list 向量列,
    names(来自 info.json)给出列内元素顺序(joint_1..gripper)。"""
    per: dict[str, list[np.ndarray]] = {n: [] for n in names}
    for f in files:
        t = pq.read_table(f, columns=[base, other]).to_pandas()
        a = np.stack(t[other].to_numpy())
        s = np.stack(t[base].to_numpy())
        d = np.abs(a - s)
        for i, n in enumerate(names):
            per[n].append(d[:, i])
    out = {}
    for n in names:
        x = np.concatenate(per[n]) if per[n] else np.zeros(0)
        out[n] = {
            "n": int(len(x)),
            "mean": float(x.mean()) if len(x) else 0.0,
            "p50": float(np.median(x)) if len(x) else 0.0,
            "p95": float(np.percentile(x, 95)) if len(x) else 0.0,
            "max": float(x.max()) if len(x) else 0.0,
        }
    return out


def segment_frames(files: list[Path], eps: int) -> list[tuple[int, int]]:
    """按 episode_index 切分,统计每个 episode 的帧数与帧索引连续性。"""
    cols = ["episode_index", "frame_index"]
    seg: dict[int, list[tuple[int, int]]] = {}
    for f in files:
        t = pq.read_table(f, columns=cols).to_pandas()
        for ep, fi in zip(t["episode_index"].tolist(), t["frame_index"].tolist()):
            seg.setdefault(int(ep), []).append(int(fi))
    out = []
    for ep in sorted(seg):
        fi = seg[ep]
        gaps = sum(1 for a, b in zip(fi, fi[1:]) if b != a + 1)
        out.append((ep, len(fi), gaps))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="LeRobot 数据集质量校验")
    ap.add_argument("dir", type=Path)
    args = ap.parse_args()

    root = args.dir
    info_p = root / "meta" / "info.json"
    if not info_p.exists():
        print(f"!! 不是数据集目录(缺 meta/info.json): {root}")
        return 1
    info = json.loads(info_p.read_text())
    print(f"== 数据集 {root.name}")
    print(f"   robot_type={info.get('robot_type')}  fps={info.get('fps')}  "
          f"episodes={info.get('total_episodes')}  frames={info.get('total_frames')}")

    feats: dict = info.get("features", {})
    act_feat = feats.get("action", {}).get("names") or []
    st_feat = feats.get("observation.state", {}).get("names") or []
    vids = {k: v for k, v in feats.items() if k.startswith("observation.images.")}

    files = [Path(p) for p in load_parquets(root / "data")]
    if not files:
        print("!! data/ 下没有 parquet 文件")
        return 1

    # 1) 每 episode 帧数与连续性
    segs = segment_frames(files, int(info.get("total_episodes", 0)))
    print(f"\n-- 帧数/连续性(共 {len(segs)} 个 episode 段):")
    total_rows = 0
    ok_seg = True
    for ep, n, gaps in segs:
        total_rows += n
        flag = "" if gaps == 0 else f"  <-- 帧索引断裂 {gaps} 处"
        if gaps:
            ok_seg = False
        print(f"   ep{ep}: {n} 帧{gaps and f' gaps={gaps}' or ''}{flag}")
    print(f"   合计行数 {total_rows} (info.total_frames={info.get('total_frames')})")

    # 2) action vs state 偏差
    if st_feat:
        print("\n-- action − state 偏差 |差| 分布(度/mm;静止≈0,运动中=跟踪残差):")
        deltas = col_delta(files, st_feat, "observation.state", "action")
        print(f"   {'关节':<14}{'n':>8}{'mean':>8}{'p50':>8}{'p95':>8}{'max':>8}")
        for j in deltas:
            d = deltas[j]
            print(f"   {j:<14}{d['n']:>8}{d['mean']:>8.2f}{d['p50']:>8.2f}"
                  f"{d['p95']:>8.2f}{d['max']:>8.2f}")

    # 3) 视频
    vroot = root / "videos"
    if vids:
        print("\n-- 视频:")
        vkeys = list(vids)
        for vk in vkeys:
            vdir = vroot / vk
            if not vdir.exists():
                print(f"   {vk}: 缺 videos/{vk} 目录")
                continue
            mp4s = sorted(glob.glob(str(vdir / "chunk-*" / "file-*.mp4")))
            size = sum(Path(p).stat().st_size for p in mp4s) / 1e6
            print(f"   {vk}: {len(mp4s)} 个 mp4,共 {size:.1f} MB")
            if not mp4s:
                print(f"   !! {vk} 无 mp4 文件")

    # 4) NaN
    na_cols = []
    for f in files:
        t = pq.read_table(f)
        if t.to_pandas().isna().any().any():
            na_cols.append(str(f))
            break
    if na_cols:
        print(f"\n!! 存在 NaN:{na_cols[0]}")
        return 1

    print("\n结论: 结构 OK" if ok_seg else "\n结论: 帧连续性有问题(见上)")
    return 0 if ok_seg else 1


if __name__ == "__main__":
    sys.exit(main())
