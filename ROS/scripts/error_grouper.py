#!/usr/bin/env python
import rospy
import threading
import numpy as np
from visual_servoing.msg import Error

n1 = False
n2 = False
trans_mat1 = np.ones((2, 2))
trans_mat2 = np.ones((2, 2))
scale1 = 1
scale2 = 1
cam_num = 2
error1 = []
error2 = []
received = [False, False]

def interface1_cb(msg):
    global received
    global error1
    global error2

    if len(msg.error) == 0:
        return

    lock.acquire()
    received[0] = True

    error1[:] = []
    for i in range(len(msg.error)):
      error1.append(msg.error[i])

    if (cam_num == 2) and (False in received):
      lock.release()
      return
    else:
      received = [False, False]
      error_pub.publish(error1 + error2)
      error1 = []
      error2 = []
      lock.release()

def interface2_cb(msg):
    global received
    global error1
    global error2
    global cam_num

    if len(msg.error) == 0:
        return
    lock.acquire()
    received[1] = True

    error2[:] = []
    for i in range(len(msg.error)):
      error2.append(msg.error[i])

    if False in received:
      lock.release()
      return
    else:
      received = [False, False]
      error_pub.publish(error1 + error2)
      error1 = []
      error2 = []
      lock.release()

if __name__ == "__main__":

    lock = threading.Lock()

    # Find out how many cameras we are using.
    rospy.init_node("error_grouper")
    cam_num = int(rospy.get_param('~cam_num', '2'))

    rospy.Subscriber("/tracker1/ledTracker/image_error", Error, interface1_cb)
    if(cam_num == 2):
      rospy.Subscriber("/tracker2/ledTracker/image_error", Error, interface2_cb)

    error_pub = rospy.Publisher("image_error", Error, queue_size=1)

    rospy.spin()
