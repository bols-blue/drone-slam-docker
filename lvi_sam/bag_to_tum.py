#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""slam_output.bag の nav_msgs/Odometry を TUM 形式 (t x y z qx qy qz qw) に書き出す。python2/3 両対応。"""
import sys

import rosbag

bag, topic, out = sys.argv[1], sys.argv[2], sys.argv[3]
n = 0
with open(out, "w") as f:
    for _, m, _ in rosbag.Bag(bag).read_messages(topics=[topic]):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        f.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        n += 1
print("[bag_to_tum] %s: %d poses -> %s" % (topic, n, out))
