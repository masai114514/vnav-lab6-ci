#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
push_via_api.py —— 绕过挂掉的 `git push`，用 GitHub Git Data API 把整个仓库推上去

为什么不用 git
-------------
本机的 git 出口走代理必挂：
    CONNECT tunnel failed, response 502
直连（`git -c http.proxy= ...`）又被沙箱在超时后杀掉（exit 137）。
而 api.github.com 是通的（`gh` 一直能用），所以走 Git Data API：
    blob -> tree(base_tree) -> commit -> PATCH ref
这条路对二进制安全（blob 用 base64）、能只推改动过的文件、还能顺手删掉远端多余文件。

用法
----
    PY=/Users/114514ma/.workbuddy/binaries/python/envs/default/bin/python
    $PY push_via_api.py                       # 推到默认仓
    $PY push_via_api.py --message "ci: xxx"
    $PY push_via_api.py --create public       # 顺带建公开仓（4 vCPU）再推
    $PY push_via_api.py --dry-run             # 只看会推哪些文件
    $PY push_via_api.py --repo owner/name --dir <仓目录>

身份 / 令牌
----------
用 `gh auth token` 取令牌，不自己读配置文件。
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import subprocess
import sys
import time

DEFAULT_REPO = "masai114514/vnav-lab6-ci"
HERE = os.path.dirname(os.path.abspath(__file__))

# 不推的东西
# ci_runs/ 是下载下来的运行产物（46 张 PNG + 日志，十几 MB），不该进仓；
# 之前在加入这条之前推过一次，远端残留了这些文件，脚本的「删除远端多余文件」
# 逻辑会在下次推送时把它们清掉。
SKIP_DIRS = {".git", "out", "ci_runs", "__pycache__", ".github/workflows/.backup"}
SKIP_SUFFIX = (".pyc", ".DS_Store")
SKIP_FILES = {".DS_Store"}


# ---------------------------------------------------------------------------
# gh 包装
# ---------------------------------------------------------------------------
def gh(*args, input_data=None, check=True):
    """调 gh。input_data 是 str（JSON），走 stdin。"""
    cmd = ["gh", "api", *args]
    if input_data is not None:
        cmd += ["--input", "-"]
    p = subprocess.run(cmd, input=input_data, capture_output=True, text=True)
    if p.returncode != 0:
        if check:
            raise RuntimeError("gh %s 失败：\n%s\n%s"
                               % (" ".join(args), p.stdout.strip(), p.stderr.strip()))
        return None
    return p.stdout


def gh_json(*args, input_data=None, check=True, retries=4):
    """调 gh 并解析 JSON；对 5xx / 网络抖动做指数退避重试。"""
    last = None
    for attempt in range(retries):
        try:
            out = gh(*args, input_data=input_data, check=check)
        except RuntimeError as e:
            last = e
            if attempt == retries - 1:
                raise
            time.sleep(1.5 * (attempt + 1))
            continue
        if out is None:
            return None
        out = out.strip()
        if not out:
            return {}
        try:
            return json.loads(out)
        except json.JSONDecodeError as e:
            last = e
            if attempt == retries - 1:
                raise RuntimeError("gh %s 返回的不是 JSON：%s" % (" ".join(args), out[:400]))
            time.sleep(1.5 * (attempt + 1))
    raise last  # pragma: no cover


# ---------------------------------------------------------------------------
# 本地文件收集
# ---------------------------------------------------------------------------
def collect(root):
    """返回 [(相对路径, 绝对路径, git_mode)]，按路径排序。"""
    items = []
    for dirpath, dirnames, filenames in os.walk(root):
        # 原地裁剪，os.walk 就不会下去
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
        for fn in sorted(filenames):
            if fn in SKIP_FILES or fn.endswith(SKIP_SUFFIX):
                continue
            ap = os.path.join(dirpath, fn)
            rp = os.path.relpath(ap, root).replace(os.sep, "/")
            if any(rp.startswith(d + "/") for d in SKIP_DIRS):
                continue
            mode = "100755" if os.access(ap, os.X_OK) and not fn.endswith((".py", ".md", ".txt", ".yml", ".yaml", ".json")) else "100644"
            if fn.endswith(".sh"):
                mode = "100755"
            items.append((rp, ap, mode))
    items.sort(key=lambda x: x[0])
    return items


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="用 GitHub API 推仓")
    ap.add_argument("--repo", default=DEFAULT_REPO, help="owner/name")
    ap.add_argument("--dir", default=HERE, help="要推的本地目录（默认本脚本所在目录）")
    ap.add_argument("--message", default=None, help="提交信息")
    ap.add_argument("--create", choices=["public", "private"], default=None,
                    help="先建仓（public=4 vCPU 免费档）再推")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-prune", action="store_true", help="不删远端多余文件")
    args = ap.parse_args()

    root = os.path.abspath(args.dir)
    items = collect(root)
    total_bytes = sum(os.path.getsize(a) for _, a, _ in items)
    print("[push] 仓        = %s" % args.repo)
    print("[push] 本地目录  = %s" % root)
    print("[push] 文件数    = %d（%.2f MB）" % (len(items), total_bytes / 1e6))
    if args.dry_run:
        for rp, ap_, mode in items:
            print("        %s  %s" % (mode, rp))
        print("[push] --dry-run，未推送。")
        return 0
    if not items:
        print("[push] 没有文件，退出。", file=sys.stderr)
        return 1

    owner, name = args.repo.split("/", 1)

    # ---- 0) 建仓 ----------------------------------------------------------
    if args.create:
        print("[push] 建仓 %s（%s）" % (args.repo, args.create))
        payload = json.dumps({
            "name": name,
            "private": args.create == "private",
            "auto_init": True,          # 生成一个初始提交，省得处理「空仓无 ref」
            "description": "MIT 16.485 VNAV Lab5 — CI harness: OpenCV 3.4.2 nonfree + ROS melodic, SIFT/SURF/ORB/FAST/LK",
            "has_issues": False,
            "has_wiki": False,
            "has_projects": False,
        })
        try:
            gh_json("user/repos", input_data=payload)
        except RuntimeError as e:
            if "already exists" not in str(e):
                raise
            print("[push]   仓已存在，直接用")

    # ---- 1) 拿 base commit / base tree ------------------------------------
    info = gh_json("repos/%s" % args.repo)
    branch = (info or {}).get("default_branch") or "main"
    print("[push] 默认分支 = %s" % branch)

    ref = gh_json("repos/%s/git/ref/heads/%s" % (args.repo, branch), check=False)
    if ref and ref.get("object", {}).get("sha"):
        base_commit_sha = ref["object"]["sha"]
        base_commit = gh_json("repos/%s/git/commits/%s" % (args.repo, base_commit_sha))
        base_tree = base_commit["tree"]["sha"]
        parents = [base_commit_sha]
        print("[push] base commit = %s" % base_commit_sha[:10])
    else:
        base_commit_sha, base_tree, parents = None, None, []
        print("[push] 远端还没有这个分支 —— 按首次提交处理")

    # ---- 2) 逐个建 blob ---------------------------------------------------
    # 只上传内容变了的（先比对远端 tree 里的 sha）
    remote = {}
    if base_tree and not args.no_prune:
        t = gh_json("repos/%s/git/trees/%s?recursive=1" % (args.repo, base_tree))
        for e in (t or {}).get("tree", []):
            if e.get("type") == "blob":
                remote[e["path"]] = e["sha"]

    tree_entries = []
    uploaded = skipped = 0
    for i, (rp, ap_, mode) in enumerate(items, 1):
        with open(ap_, "rb") as f:
            data = f.read()
        # 本地先算 git blob sha，和远端一致就跳过（省 API 次数）
        h = hashlib.sha1()
        h.update(b"blob %d\0" % len(data))
        h.update(data)
        sha_local = h.hexdigest()
        if remote.get(rp) == sha_local:
            tree_entries.append({"path": rp, "mode": mode, "type": "blob", "sha": sha_local})
            skipped += 1
            continue

        payload = json.dumps({"content": base64.b64encode(data).decode("ascii"),
                              "encoding": "base64"})
        b = gh_json("repos/%s/git/blobs" % args.repo, input_data=payload)
        tree_entries.append({"path": rp, "mode": mode, "type": "blob", "sha": b["sha"]})
        uploaded += 1
        if i % 25 == 0 or i == len(items):
            print("[push]   blob %d/%d（上传 %d / 复用 %d）"
                  % (i, len(items), uploaded, skipped))

    # 远端有、本地没有的 → 显式删（sha: null）
    if base_tree and not args.no_prune:
        local_paths = {rp for rp, _, _ in items}
        removed = [p for p in remote if p not in local_paths]
        for p in removed:
            tree_entries.append({"path": p, "mode": "100644", "type": "blob", "sha": None})
        if removed:
            print("[push] 删除远端多余文件 %d 个" % len(removed))

    # ---- 3) tree ----------------------------------------------------------
    tree_payload = {"tree": tree_entries}
    if base_tree:
        tree_payload["base_tree"] = base_tree
    tree = gh_json("repos/%s/git/trees" % args.repo, input_data=json.dumps(tree_payload))
    print("[push] tree = %s" % tree["sha"][:10])

    # ---- 4) commit --------------------------------------------------------
    msg = args.message or "ci: LAB5 特征跟踪链路（OpenCV 3.4.2 nonfree + 四描述子 + Harris-LK）"
    commit_payload = {"message": msg, "tree": tree["sha"], "parents": parents}
    commit = gh_json("repos/%s/git/commits" % args.repo, input_data=json.dumps(commit_payload))
    print("[push] commit = %s" % commit["sha"][:10])

    # ---- 5) 更新 ref ------------------------------------------------------
    if base_commit_sha:
        # 已有分支：ref 的更新是 PATCH，得显式指定方法
        p = subprocess.run(
            ["gh", "api", "-X", "PATCH",
             "repos/%s/git/refs/heads/%s" % (args.repo, branch),
             "--input", "-"],
            input=json.dumps({"sha": commit["sha"], "force": False}),
            capture_output=True, text=True)
        if p.returncode != 0:
            raise RuntimeError("更新 ref 失败：\n%s\n%s" % (p.stdout, p.stderr))
    else:
        gh_json("repos/%s/git/refs" % args.repo,
                input_data=json.dumps({"ref": "refs/heads/%s" % branch, "sha": commit["sha"]}))

    print("[push] 完成。%s 的 %s 现在指向 %s" % (args.repo, branch, commit["sha"][:10]))
    print("[push] https://github.com/%s" % args.repo)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[push] 中断", file=sys.stderr)
        sys.exit(130)
    except Exception as e:  # noqa: BLE001
        print("[push] 失败：%s" % e, file=sys.stderr)
        sys.exit(1)
