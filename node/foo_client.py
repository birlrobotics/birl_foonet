 #! /usr/bin/env python
import rospy
import actionlib
import birl_foonet.msg

def foonet_client():

    client = actionlib.SimpleActionClient("foonet_action", birl_foonet.msg.FoonetAction)     
    if client.wait_for_server(rospy.Duration(3.0)):
        object_id = [0,1,2]
        goal = birl_foonet.msg.FoonetGoal(object_id)
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

if __name__ == '__main__':

    rospy.init_node('foonet_client_py')
    result = foonet_client()
    print (result)
