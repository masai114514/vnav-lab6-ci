#!/usr/bin/env python3
"""Render a compact trajectory video from a Lab 6 ROS pose recording."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
try:
    import imageio_ffmpeg
    matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
except ImportError:
    # GitHub's Ubuntu runner normally has ffmpeg. imageio-ffmpeg is a portable
    # fallback installed by the workflow.
    pass
import matplotlib.animation as animation
import matplotlib.pyplot as plt


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError("trajectory CSV has no data rows: %s" % path)
    columns = [
        "gt_x", "gt_y", "gt_z", "gt_qx", "gt_qy", "gt_qz", "gt_qw",
        "est_x", "est_y", "est_z", "est_qx", "est_qy", "est_qz", "est_qw",
    ]
    data = {name: [] for name in columns}
    dropped = 0
    for row in rows:
        try:
            values = [float(row[name]) for name in columns]
        except (KeyError, TypeError, ValueError):
            dropped += 1
            continue
        # Pose estimators can occasionally publish an invalid pose for a
        # degenerate frame.  Such a sample must not be passed to Matplotlib,
        # whose axis limits reject NaN/Inf values.
        if not all(math.isfinite(value) for value in values):
            dropped += 1
            continue
        for name, value in zip(columns, values):
            data[name].append(value)
    if not data["gt_x"]:
        raise RuntimeError("trajectory CSV has no finite pose rows: %s" % path)
    if dropped:
        print("TRAJECTORY_FILTERED_INVALID_ROWS", dropped)
    return data


def yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def padded_limits(a, b):
    lo = min(min(a), min(b))
    hi = max(max(a), max(b))
    span = max(hi - lo, 0.5)
    return lo - 0.12 * span, hi + 0.12 * span


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--method", required=True)
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()

    data = load_rows(args.csv)
    count = len(data["gt_x"])
    position_error = [
        math.sqrt((data["est_x"][i] - data["gt_x"][i]) ** 2 +
                  (data["est_y"][i] - data["gt_y"][i]) ** 2 +
                  (data["est_z"][i] - data["gt_z"][i]) ** 2)
        for i in range(count)
    ]

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, (ax_traj, ax_err) = plt.subplots(1, 2, figsize=(12.8, 7.2), dpi=100)
    fig.patch.set_facecolor("#f6f7f9")
    fig.suptitle("Lab 6 Office trajectory | %s | RANSAC" % args.method,
                 fontsize=16, fontweight="bold")

    xlim = padded_limits(data["gt_x"], data["est_x"])
    ylim = padded_limits(data["gt_y"], data["est_y"])
    ax_traj.set_xlim(*xlim)
    ax_traj.set_ylim(*ylim)
    ax_traj.set_aspect("equal", adjustable="box")
    ax_traj.set_title("Top-down camera trajectory")
    ax_traj.set_xlabel("world x (m)")
    ax_traj.set_ylabel("world y (m)")
    ax_traj.grid(alpha=0.25)

    gt_line, = ax_traj.plot([], [], color="#2e8b57", lw=2.5,
                            label="Ground truth")
    est_line, = ax_traj.plot([], [], color="#f0a202", lw=2.0,
                             label="Estimated pose")
    gt_point, = ax_traj.plot([], [], "o", color="#2e8b57", ms=8)
    est_point, = ax_traj.plot([], [], "o", color="#f0a202", ms=8)
    gt_heading, = ax_traj.plot([], [], color="#176b3a", lw=2.5)
    est_heading, = ax_traj.plot([], [], color="#b96d00", lw=2.5)
    ax_traj.legend(loc="best")

    err_hi = max(max(position_error) * 1.12, 0.05)
    ax_err.set_xlim(0, max(count - 1, 1))
    ax_err.set_ylim(0, err_hi)
    ax_err.set_title("Position difference during visualization")
    ax_err.set_xlabel("processed frame-pair index")
    ax_err.set_ylabel("||estimated - ground truth|| (m)")
    ax_err.grid(alpha=0.25)
    err_line, = ax_err.plot([], [], color="#5b5f97", lw=2)
    frame_text = ax_err.text(0.03, 0.95, "", transform=ax_err.transAxes,
                             va="top", fontsize=12,
                             bbox={"boxstyle": "round", "facecolor": "white",
                                   "alpha": 0.85, "edgecolor": "#cccccc"})
    fig.text(0.5, 0.025,
             "Actual ROS replay: /gt_camera_pose (green) and /camera_pose (orange)",
             ha="center", color="#555555")

    heading_length = 0.06 * max(xlim[1] - xlim[0], ylim[1] - ylim[0])

    def update(i):
        end = i + 1
        gt_line.set_data(data["gt_x"][:end], data["gt_y"][:end])
        est_line.set_data(data["est_x"][:end], data["est_y"][:end])
        gx, gy = data["gt_x"][i], data["gt_y"][i]
        ex, ey = data["est_x"][i], data["est_y"][i]
        gt_point.set_data([gx], [gy])
        est_point.set_data([ex], [ey])

        gyaw = yaw(data["gt_qx"][i], data["gt_qy"][i],
                   data["gt_qz"][i], data["gt_qw"][i])
        eyaw = yaw(data["est_qx"][i], data["est_qy"][i],
                   data["est_qz"][i], data["est_qw"][i])
        gt_heading.set_data([gx, gx + heading_length * math.cos(gyaw)],
                            [gy, gy + heading_length * math.sin(gyaw)])
        est_heading.set_data([ex, ex + heading_length * math.cos(eyaw)],
                             [ey, ey + heading_length * math.sin(eyaw)])
        err_line.set_data(range(end), position_error[:end])
        frame_text.set_text("frame pair: %d / %d\nposition difference: %.4f m" %
                            (end, count, position_error[i]))
        return (gt_line, est_line, gt_point, est_point, gt_heading,
                est_heading, err_line, frame_text)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    movie = animation.FuncAnimation(fig, update, frames=count,
                                    interval=1000.0 / args.fps, blit=False)
    writer = animation.FFMpegWriter(fps=args.fps, bitrate=2600,
                                    metadata={"title": "Lab 6 %s trajectory" % args.method})
    movie.save(str(args.output), writer=writer)
    plt.close(fig)
    print("TRAJECTORY_VIDEO_PASS", args.output, "frames=", count)


if __name__ == "__main__":
    main()
