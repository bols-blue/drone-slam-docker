#!/usr/bin/env python3
"""bag 再生終了後も /clock を進め続ける。

use_sim_time=true のまま /clock が止まると、ros::Rate::sleep() が永久に待ち、
FAST-LIO が SIGINT を受けても地図 (scans.pcd) の保存処理まで進めない。
最後の /clock の時刻から、実時間に合わせて 100 Hz で時刻を進める。
"""
import time

import rospy
from rosgraph_msgs.msg import Clock

rospy.init_node("clock_keeper", disable_signals=False)
last = {"t": None}
sub = rospy.Subscriber("/clock", Clock, lambda m: last.__setitem__("t", m.clock))
pub = rospy.Publisher("/clock", Clock, queue_size=1)
w0 = time.time()
while last["t"] is None and time.time() - w0 < 3.0:
    time.sleep(0.05)
sub.unregister()
base = last["t"] or rospy.Time.from_sec(time.time())
w0 = time.time()
while not rospy.is_shutdown():
    pub.publish(Clock(clock=base + rospy.Duration.from_sec(time.time() - w0)))
    time.sleep(0.01)
