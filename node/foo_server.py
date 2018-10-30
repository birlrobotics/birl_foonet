#! /usr/bin/env python

import rospy

import actionlib

import birl_foonet.msg

from copy import deepcopy
import ipdb
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose

from birl_foonet.msg import foonet

shared_msg = None

def cb(msg):
    global shared_msg
    shared_msg = msg.markers

class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = birl_foonet.msg.FoonetFeedback()
    _result = birl_foonet.msg.FoonetResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("foonet_action", birl_foonet.msg.FoonetAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        
        r = rospy.Rate(1)
        success = True

        
        

        for _obj_idx in goal.object_id:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            for msg in shared_msg:
                if _obj_idx == msg.id:
                    # ipdb.set_trace()
                    foonet_msg = foonet()
                    foonet_msg.header = msg.header
                    foonet_msg.pose   = msg.pose.pose
                    foonet_msg.object_id = msg.id
                    self._feedback.object_pose.append(foonet_msg)
                    break
            else:
                self._feedback.missing_id.append(_obj_idx)
            self._as.publish_feedback(self._feedback)

        
            # print self._feedback.object_pose

            r.sleep()
            
        if success:
            self._result = self._feedback
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('foonet_server')
    if rospy.wait_for_message("ar_pose_marker",AlvarMarkers,timeout=5):
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
    else:
        rospy.logfatal("Could not connect to /ar_pose_marker server, is the server node running?");
        rospy.signal_shutdown("Required component missing"); 
    server = FibonacciAction()
    rospy.spin()