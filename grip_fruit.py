#!/usr/bin/env python3

import rospy
import actionlib

from pr2_controllers_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# /pressure/l_gripper_motor
# /pressure/l_gripper_motor_info
# displacement
# fruit_size

# /joint_states
# /tf
# /tf_static


class GripperDemo():

    def __init__(self):
        self.sub = rospy.Subscriber("/joint_states", JointState, self.on_joint_states, queue_size=1)
        self.action_client = actionlib.SimpleActionClient("/l_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.pub = rospy.Publisher("/l_gripper_controller/command", Pr2GripperCommand, queue_size=1)
        self.joint_states = JointState()
        self.fruit_size = 0
        
        self.disp_pub = rospy.Publisher("/displacement", Float64, queue_size=1)
        self.fruit_size_pub = rospy.Publisher("/fruit_size", Float64, queue_size=1)
        

        self.action_client.wait_for_server()

        rospy.loginfo("Action Server Ready.")


        self.open_gripper()
        rospy.sleep(1)
        self.find_fruit_size()
        rospy.sleep(1)
        self.squeeze_gripper()
        self.open_gripper()

        rospy.loginfo("Done")

    def open_gripper(self):
        rospy.loginfo('opening gripper')
        goal = Pr2GripperCommandGoal()
        goal.command.position = 0.15
        goal.command.max_effort = 30
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        
    def squeeze_gripper(self):
        inc = 2
        depth = 8
        displacements = [x / 100000 for x in range(inc * 100, (depth + inc) * 100, inc * 100)] # [0.0002, 0.0004, 0.0006, 0.0008)
        for d in displacements:
            goal = Pr2GripperCommandGoal()
            goal.command.position = self.fruit_size - d
            goal.command.max_effort = -1
            self.action_client.send_goal(goal)
            
            rate = rospy.Rate(100)
            while self.action_client.get_state() <= actionlib.GoalStatus.ACTIVE: 
                self.publish(d)
                rate.sleep()
            self.action_client.wait_for_result()
            rospy.loginfo(f'moving by {d}')
            rospy.sleep(1)

    def find_fruit_size(self):
        rospy.loginfo('finding fruit size')

        effort = 50
        
        goal = Pr2GripperCommandGoal()
        goal.command.position = 0
        goal.command.max_effort = effort
        self.action_client.send_goal(goal)

        while not rospy.is_shutdown():
            if abs(self.get_joint("l_gripper_joint")["velocity"]) > 0:
                effort = 15
                goal.command.position = 0
                goal.command.max_effort = effort
                self.action_client.send_goal(goal)
                break
        
        self.action_client.wait_for_result()

        rospy.loginfo('fruit size found')
        self.fruit_size = self.get_joint("l_gripper_joint")["position"]

    def on_joint_states(self, msg: JointState):
        self.joint_states = msg

        index = self.joint_states.name.index("l_gripper_joint")
        # print(f"effort: {self.joint_states.effort[index]}")

    def get_joint(self, joint_name: str):
        index = self.joint_states.name.index(joint_name)
        return {"position": self.joint_states.position[index], "velocity": self.joint_states.velocity[index], "effort": self.joint_states.effort[index]}
    
    def publish(self, disp):
        self.disp_pub.publish(Float64(data=disp))
        self.fruit_size_pub.publish(Float64(data=self.fruit_size))


if __name__ == "__main__":
    rospy.init_node("gripper_demo")
    node = GripperDemo()
    # rospy.spin()
