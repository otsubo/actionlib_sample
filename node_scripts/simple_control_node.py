#!/usr/bin/env python
import numpy as np
import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from actionlib_sample.msg import SimpleControlAction
from actionlib_sample.msg import SimpleControlResult
from actionlib_sample.msg import SimpleControlFeedback

class SimpleControl( object ):
    def __init__( self ):
        self.pub = rospy.Publisher( '/joint_states', JointState, queue_size=1)
        self.action_server = actionlib.SimpleActionServer('control_action_server', SimpleControlAction,execute_cb=self.control_loop, auto_start=False)
        self.encoder_values = np.array([0.0, 0.0])
        self.joint_name = ["Joint_2", "Joint_3"]
        self.p_gain = rospy.get_param('~p_gain', -1.1)
        self.d_gain = rospy.get_param('~d_gain', -0.01)
        self.i_gain = rospy.get_param('~i_gain', -0.1)
        self.thre = rospy.get_param('~thre', 0.001)

        self.zero_vector = [0.0, 0.0]
        self.action_server.start()
        self.prev_dist = None
        self.error_sum = np.array([0.0, 0.0])

        rate = rospy.get_param('~rate', 100.)
        rospy.Timer(rospy.Duration(1. / rate), self.read)

    def read(self, event):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name = self.joint_name
        js_msg.position = self.encoder_values
        js_msg.velocity = self.zero_vector
        js_msg.effort = self.zero_vector
        self.pub.publish(js_msg)

    def control_loop(self, goal):
        print(goal.position)
        r = rospy.Rate( 10.0 )
        for i in range( 10 * goal.timeout_sec):
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                break
            else:
                goal_array = np.array(goal.position)
                dist = self.encoder_values - goal_array
                if np.linalg.norm(dist) < self.thre:
                    result = SimpleControlResult(conversion=True)
                    self.action_server.set_succeeded(result)
                    self.error_sum = 0
                    return
                else:
                    array_msg = Float64MultiArray()
                    array_msg.data = dist
                    feedback = SimpleControlFeedback(dist=array_msg)
                    self.action_server.publish_feedback(feedback)
                    if self.prev_dist is not None:
                        print(f'p_output: {dist * self.p_gain}')
                        print(f'd_output: {self.prev_dist * self.d_gain}')
                        print(f'i_output: {self.error_sum * self.i_gain}')
                        self.encoder_values += dist * self.p_gain  + self.prev_dist * self.d_gain + self.error_sum * self.i_gain
                        self.prev_dist = dist
                        self.error_sum += dist
                    else:
                        self.encoder_values += dist * self.p_gain
                        self.prev_dist = dist
                        self.error_sum += dist
            r.sleep()

if __name__ == '__main__':
    rospy.init_node( 'simple_control_node')
    SimpleControl()
    rospy.spin()
