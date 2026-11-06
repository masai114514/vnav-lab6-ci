#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# ROS melodic 的 rospy / cv_bridge 只有 python2.7 的绑定，所以这个脚本必须
# 写成 py2/py3 双兼容，并显式声明 utf-8 编码（否则 py2 读到中文注释直接报
# SyntaxError: Non-ASCII character）。业务逻辑本身两边都成立。
"""
把 TUM RGB-D 序列"当成 office.bag"喂给 lab_6 节点。

为什么需要这个脚本：官方 office.bag 在 Google Drive 上，当前网络下取不到，
而 lab_6 节点只认三个话题。这个脚本按节点期望的话题与消息类型重放 TUM 序列：

    /tesse/left_cam/rgb/image_raw   sensor_msgs/Image  (bgr8)
    /tesse/depth_cam/mono/image_raw sensor_msgs/Image  (32FC1, 单位：米)
    /tesse/odom                     nav_msgs/Odometry

注意 odom 的语义：官方 bag 给的是**机体系**位姿 T_body^W，节点内部再乘一个
固定外参 T_camera^body 得到相机位姿。TUM 的真值直接就是相机位姿 T_cam^W，
所以这里先反解出 body：T_body^W = T_cam^W * (T_camera^body)^{-1}，
这样节点算出来的相机位姿正好等于 TUM 真值。

用法：
  rosrun lab_6 tum_play.py --data ~/data/tum/rgbd_dataset_freiburg1_xyz --rate 10
"""

import argparse
import os
import sys
import time

import numpy as np

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


# 与 pose_estimation.cpp 中的 T_camera_body 完全一致
T_CAMERA_BODY = np.array([[0.0, 0.0, 1.0, 0.0],
                          [-1.0, 0.0, 0.0, 0.05],
                          [0.0, -1.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]])
T_BODY_CAMERA = np.linalg.inv(T_CAMERA_BODY)


def read_index(path):
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            ts, name = line.split()
            entries.append((float(ts), name))
    return entries


def associate(primary, secondary, max_diff=0.02):
    out = []
    j = 0
    for ts, name in primary:
        best = None
        while j < len(secondary) and secondary[j][0] < ts - max_diff:
            j += 1
        k = j
        while k < len(secondary) and secondary[k][0] <= ts + max_diff:
            d = abs(secondary[k][0] - ts)
            if best is None or d < best[0]:
                best = (d, secondary[k])
            k += 1
        if best is not None:
            out.append((ts, name, best[1][1]))
    return out


def read_groundtruth(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            v = [float(x) for x in line.split()]
            ts, tx, ty, tz, qx, qy, qz, qw = v[:8]
            out.append((ts, np.array([tx, ty, tz]), np.array([qx, qy, qz, qw])))
    return out


def nearest_pose(gt, ts):
    lo, hi = 0, len(gt) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if gt[mid][0] < ts:
            lo = mid + 1
        else:
            hi = mid
    cand = [lo]
    if lo > 0:
        cand.append(lo - 1)
    return gt[min(cand, key=lambda i: abs(gt[i][0] - ts))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True, help='TUM 序列目录')
    ap.add_argument('--rate', type=float, default=10.0, help='发布频率 (Hz)')
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--max-frames', type=int, default=0)
    # 两个硬上限。没有它们，CI 会一直挂在这里：
    # 节点崩掉之后没有任何订阅者，get_num_connections() 永远是 0，
    # 下面的等待循环就变成一个死循环（Run #6 挂了 47 分钟就是这么来的）。
    ap.add_argument('--wait-subscribers', type=float, default=45.0,
                    help='等订阅者的最长秒数，超时直接退出（默认 45）')
    ap.add_argument('--max-seconds', type=float, default=0.0,
                    help='整段播放的硬上限；0 = 按 帧数/频率 自动估')
    args, _ = ap.parse_known_args(rospy.myargv()[1:])

    rospy.init_node('tum_play', anonymous=True)
    bridge = cv_bridge.CvBridge()
    pub_rgb = rospy.Publisher('/tesse/left_cam/rgb/image_raw', Image, queue_size=2)
    pub_depth = rospy.Publisher('/tesse/depth_cam/mono/image_raw', Image, queue_size=2)
    pub_odom = rospy.Publisher('/tesse/odom', Odometry, queue_size=10)

    rgb = read_index(os.path.join(args.data, 'rgb.txt'))
    depth = read_index(os.path.join(args.data, 'depth.txt'))
    gt = read_groundtruth(os.path.join(args.data, 'groundtruth.txt'))
    pairs = associate(rgb, depth)[::args.stride]
    if args.max_frames:
        pairs = pairs[:args.max_frames]
    if args.max_seconds <= 0:
        # 1.6 倍余量 + 120 s 启动余量：801 帧 @8 Hz ≈ 100 s → 上限约 280 s
        args.max_seconds = len(pairs) / max(args.rate, 1e-3) * 1.6 + 120.0
    rospy.loginfo('tum_play: %d frames from %s (上限 %.0f s)',
                  len(pairs), args.data, args.max_seconds)

    rate = rospy.Rate(args.rate)
    # 等订阅者连上（有上限：节点崩了就没人订阅，不能死等）
    t_wait = time.time()
    while pub_rgb.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.time() - t_wait > args.wait_subscribers:
            rospy.logerr('tum_play: %.0f s 内没有订阅者 —— 节点多半已经崩了，'
                         '去 out/smoke.log 或 out/run_*.log 找栈', args.wait_subscribers)
            return 2
        rate.sleep()

    t0 = time.time()
    for idx, (ts, rgb_rel, depth_rel) in enumerate(pairs):
        if rospy.is_shutdown():
            break
        if time.time() - t0 > args.max_seconds:
            rospy.logwarn('tum_play: 到达 max-seconds=%.0f s，提前停止（%d/%d）',
                          args.max_seconds, idx, len(pairs))
            break
        img = cv2_imread(os.path.join(args.data, rgb_rel), True)
        dep = cv2_imread(os.path.join(args.data, depth_rel), False)
        if img is None or dep is None:
            continue

        stamp = rospy.Time.now()
        rgb_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'camera'
        pub_rgb.publish(rgb_msg)

        # TUM 深度是 16UC1、1 单位 = 1/5000 米；节点按 32FC1 米制读取
        dep_m = (dep.astype(np.float32) / 5000.0)
        dep_msg = bridge.cv2_to_imgmsg(dep_m, encoding='32FC1')
        dep_msg.header.stamp = stamp
        dep_msg.header.frame_id = 'camera'
        pub_depth.publish(dep_msg)

        # 真值：TUM 给的是相机位姿，反解出机体系位姿再发出去
        t_query = nearest_pose(gt, ts)
        q = t_query[2]
        cam = np.eye(4)
        cam[:3, :3] = quat_to_R(q)
        cam[:3, 3] = t_query[1]
        body = np.dot(cam, T_BODY_CAMERA)   # py2 不支持 @ 运算符，用 np.dot
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x = float(body[0, 3])
        odom.pose.pose.position.y = float(body[1, 3])
        odom.pose.pose.position.z = float(body[2, 3])
        qx, qy, qz, qw = Q_from_R(body[:3, :3])
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        pub_odom.publish(odom)

        if (idx + 1) % 100 == 0:
            rospy.loginfo('tum_play: %d/%d', idx + 1, len(pairs))
        rate.sleep()

    rospy.loginfo('tum_play: done, waiting for the node to flush ...')
    rospy.sleep(2.0)
    return 0


def cv2_imread(path, color):
    import cv2
    return cv2.imread(path, cv2.IMREAD_COLOR if color else cv2.IMREAD_UNCHANGED)


def quat_to_R(q):
    qx, qy, qz, qw = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def Q_from_R(R):
    """旋转矩阵 -> (x, y, z, w)"""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


if __name__ == '__main__':
    sys.exit(main())
