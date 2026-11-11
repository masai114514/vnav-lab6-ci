#!/usr/bin/env python3
"""在 sqlite 层把 rosbag2（.db3）截成一段更短的 rosbag2。

为什么需要这个：
    官方 vnav-lab6-office 解压后是 20.4 GB 的 db3、237 秒、34.7 万条消息。
    冒烟只需要前十几秒，没必要把 20 GB 搬进 CI。截出来通常只有几百 MB，
    可以直接当 release 资产传给 runner。

为什么要在 sqlite 层做（而不是反序列化再写）：
    这份 bag 是 ROS2 录的，消息是 ROS2 的 CDR 布局。用 ROS1 的类型定义去
    deserialize 会失败（``Invalid string length ⋯ at offset 12``，因为
    Header.stamp 在 ROS1 是 time、在 ROS2 是 builtin_interfaces/Time，
    字段名和对齐都不同）。在 sqlite 层搬运原始 BLOB 就完全不碰消息格式，
    后续交给 ``rosbags-convert`` 用正确的 typestore 转换。

用法：
    python3 trim_db3.py <src 目录> <out 目录> [--seconds 15] [--skip 0]

    <src 目录> 里要有 metadata.yaml 和 *.db3。
    输出目录里会生成 metadata.yaml + <out 目录名>.db3。

依赖：只用标准库。
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import sqlite3
import sys

DEFAULT_TOPICS = (
    "/tf,"
    "/tesse/odom,"
    "/tesse/left_cam/rgb/image_raw,"
    "/tesse/depth_cam/mono/image_raw,"
    "/tesse/left_cam/camera_info,"
    "/tesse/depth_cam/camera_info,"
    "/clock"
)


def parse_topics(meta_text: str) -> list[dict]:
    """从 metadata.yaml 里抠出每个话题的元数据（避免依赖 PyYAML）。"""
    seg = meta_text.split("topics_with_message_count:", 1)[1]
    seg = seg.split("\n  version:", 1)[0]
    pat = re.compile(
        r"- message_count: (\d+)\s*\n"
        r"\s*topic_metadata:\s*\n"
        r"\s*name: (\S+)\s*\n"
        r"\s*offered_qos_profiles: (.*?)\s*\n"
        r"\s*serialization_format: (\S+)\s*\n"
        r"\s*type: (\S+)\s*\n"
        r"\s*type_description_hash: (\S+)"
    )
    out = []
    for m in pat.finditer(seg):
        out.append(
            {
                "count": int(m.group(1)),
                "name": m.group(2),
                "qos": m.group(3),
                "ser": m.group(4),
                "type": m.group(5),
                "tdh": m.group(6),
            }
        )
    return out


def render_metadata(template: str, db_name: str, start: int, dur: int,
                    total: int, topics: list[dict]) -> str:
    """照着原 metadata.yaml 的结构重写一份。"""
    lines = [
        "rosbag2_bagfile_information:",
        "  compression_format: ''",
        "  compression_mode: ''",
        "  custom_data: {}",
        "  duration:",
        f"    nanoseconds: {dur}",
        "  files:",
        "  - duration:",
        f"      nanoseconds: {dur}",
        f"    message_count: {total}",
        f"    path: {db_name}",
        "    starting_time:",
        f"      nanoseconds_since_epoch: {start}",
        f"  message_count: {total}",
        "  relative_file_paths:",
        f"  - {db_name}",
        "  ros_distro: rosbags",
        "  starting_time:",
        f"    nanoseconds_since_epoch: {start}",
        "  storage_identifier: sqlite3",
        "  topics_with_message_count:",
    ]
    for t in topics:
        lines += [
            f"  - message_count: {t['count']}",
            "    topic_metadata:",
            f"      name: {t['name']}",
            f"      offered_qos_profiles: {t['qos']}",
            f"      serialization_format: {t['ser']}",
            f"      type: {t['type']}",
            f"      type_description_hash: {t['tdh']}",
        ]
    lines.append("  version: 8")
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="源 rosbag2 目录")
    ap.add_argument("dst", help="输出 rosbag2 目录（会创建）")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--skip", type=float, default=0.0)
    ap.add_argument("--topics", default=DEFAULT_TOPICS)
    args = ap.parse_args()

    src_db = glob.glob(os.path.join(args.src, "*.db3"))
    if len(src_db) != 1:
        print(f"源目录应恰好有 1 个 .db3，实际 {len(src_db)}", file=sys.stderr)
        return 2
    src_db = src_db[0]
    meta_text = open(os.path.join(args.src, "metadata.yaml"), encoding="utf-8").read()

    keep = {t.strip() for t in args.topics.split(",") if t.strip()}
    all_topics = parse_topics(meta_text)
    wanted = [t for t in all_topics if t["name"] in keep]
    missing = keep - {t["name"] for t in all_topics}
    if missing:
        print(f"源里没有这些话题：{sorted(missing)}", file=sys.stderr)

    m = re.search(r"nanoseconds_since_epoch: (\d+)", meta_text)
    bag_start = int(m.group(1))
    t0 = bag_start + int(args.skip * 1e9)
    t1 = t0 + int(args.seconds * 1e9)

    os.makedirs(args.dst, exist_ok=True)
    db_name = os.path.basename(os.path.normpath(args.dst)) + ".db3"
    dst_db = os.path.join(args.dst, db_name)
    if os.path.exists(dst_db):
        os.remove(dst_db)

    src = sqlite3.connect(f"file:{src_db}?mode=ro", uri=True)
    dst = sqlite3.connect(dst_db)
    dst.execute("PRAGMA journal_mode=OFF")
    dst.execute("PRAGMA synchronous=OFF")
    dst.execute("PRAGMA cache_size=-200000")

    for (sql,) in src.execute("SELECT sql FROM sqlite_master WHERE sql IS NOT NULL"):
        dst.execute(sql)

    dst.execute("INSERT INTO schema SELECT * FROM schema")

    names = tuple(t["name"] for t in wanted)
    ph = ",".join("?" * len(names))
    rows = src.execute(
        f"SELECT id,name,type,serialization_format,offered_qos_profiles,"
        f"type_description_hash FROM topics WHERE name IN ({ph})", names
    ).fetchall()
    dst.executemany("INSERT INTO topics VALUES (?,?,?,?,?,?)", rows)
    topic_ids = [r[0] for r in rows]

    types = tuple({r[2] for r in rows})
    tp = ",".join("?" * len(types))
    dst.execute(
        "INSERT INTO message_definitions SELECT * FROM message_definitions "
        f"WHERE topic_type IN ({tp})", types
    )

    ip = ",".join("?" * len(topic_ids))
    cur = src.execute(
        f"SELECT id,topic_id,timestamp,data FROM messages "
        f"WHERE topic_id IN ({ip}) AND timestamp >= ? AND timestamp < ? "
        f"ORDER BY timestamp", (*topic_ids, t0, t1)
    )
    n = 0
    while True:
        batch = cur.fetchmany(500)
        if not batch:
            break
        dst.executemany("INSERT INTO messages VALUES (?,?,?,?)", batch)
        n += len(batch)
        if n % 2000 == 0:
            print(f"  已写 {n} 条 …", flush=True)
    dst.commit()

    # 统计实际写入的条数与时间范围
    per_topic = dict(
        dst.execute("SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id").fetchall()
    )
    real_start, real_end = dst.execute(
        "SELECT MIN(timestamp), MAX(timestamp) FROM messages"
    ).fetchone()
    dur = real_end - real_start
    print(f"写入 {n} 条，时间 {real_start} → {real_end}（{dur / 1e9:.2f} 秒）")

    for t in wanted:
        tid = next(r[0] for r in rows if r[1] == t["name"])
        t["count"] = per_topic.get(tid, 0)
    kept = [t for t in wanted if t["count"] > 0]

    meta_new = render_metadata(meta_text, db_name, real_start, dur, n, kept)
    with open(os.path.join(args.dst, "metadata.yaml"), "w", encoding="utf-8") as f:
        f.write(meta_new)

    dst.close()
    src.close()

    size = os.path.getsize(dst_db)
    print(f"完成：{args.dst}  {size / 1e6:.1f} MB")
    for t in kept:
        print(f"  {t['count']:6d}  {t['name']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
