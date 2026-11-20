#!/usr/bin/env python
"""Record the two pose topics published by Lab 6 during an actual ROS replay.

The estimator publishes /camera_pose after processing an image pair.  The
ground-truth callback publishes /gt_camera_pose from the same data stream.  We
store the latest ground-truth pose whenever an estimate arrives.  The file is
flushed after every row so a CI shutdown cannot lose the trajectory.
"""
from __future__ import print_function

import csv
import os
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped


def pose_values(msg):
    p = msg.pose.position
    q = msg.pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def main():
    if len(sys.argv) != 2:
        print("usage: trajectory_recorder.py OUTPUT.csv", file=sys.stderr)
        return 2

    output = os.path.abspath(sys.argv[1])
    parent = os.path.dirname(output)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)

    # ROS Melodic uses Python 2, whose csv module expects binary mode.
    handle = open(output, "wb")
    writer = csv.writer(handle)
    writer.writerow([
        "frame", "wall_time",
        "gt_x", "gt_y", "gt_z", "gt_qx", "gt_qy", "gt_qz", "gt_qw",
        "est_x", "est_y", "est_z", "est_qx", "est_qy", "est_qz", "est_qw",
    ])
    handle.flush()

    lock = threading.Lock()
    state = {"gt": None, "frame": 0}

    def on_ground_truth(msg):
        with lock:
            state["gt"] = pose_values(msg)

    def on_estimate(msg):
        with lock:
            ground_truth = state["gt"]
            if ground_truth is None:
                return
            estimate = pose_values(msg)
            frame = state["frame"]
            state["frame"] += 1
            writer.writerow([frame, "%.9f" % rospy.get_time()] +
                            list(ground_truth) + list(estimate))
            handle.flush()
            if frame % 25 == 0:
                rospy.loginfo("trajectory_recorder: recorded %d poses", frame + 1)

    def close_file():
        try:
            handle.flush()
            handle.close()
        except Exception:
            pass

    rospy.init_node("lab6_trajectory_recorder", anonymous=True)
    rospy.on_shutdown(close_file)
    rospy.Subscriber("/gt_camera_pose", PoseStamped, on_ground_truth, queue_size=20)
    rospy.Subscriber("/camera_pose", PoseStamped, on_estimate, queue_size=20)
    rospy.loginfo("trajectory_recorder: writing %s", output)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
