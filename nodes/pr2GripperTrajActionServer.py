#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Scott Niekum
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Scott Niekum

import roslib; roslib.load_manifest('pr2_gripper_traj_action')
import rospy 
import actionlib as al 
from pr2_controllers_msgs.msg import * 
import matplotlib.pyplot as plt
from pr2_gripper_traj_action.msg import *
from std_msgs.msg import *


class PR2GripperTrajActionServer:
    def __init__(self,whicharm):
        self.whicharm = whicharm
        if(whicharm == 0):
            traj_segment_name = 'r_arm_controller/current_segment'
            gripper_cont_name = '/r_gripper_controller/gripper_action'
            act_name = 'r_gripper_traj_action'
        else:
            traj_segment_name = 'l_arm_controller/current_segment'
            gripper_cont_name = '/l_gripper_controller/gripper_action'
            act_name = 'l_gripper_traj_action'
        
        self.gripper_client = al.SimpleActionClient(gripper_cont_name, Pr2GripperCommandAction)
        while not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the gripper action server..."
        print "Connected to gripper action server"
        self.grip_goal = Pr2GripperCommandGoal()
        
        rospy.Subscriber(traj_segment_name, Int32, self.trajSegmentCallback)
        self.curr_traj_segment = -1
        
        self.server = al.SimpleActionServer(act_name, Pr2GripperTrajAction, self.execute, False)
        self.feedback = pr2_gripper_traj_action.msg.Pr2GripperTrajFeedback()
        self.result = pr2_gripper_traj_action.msg.Pr2GripperTrajResult()
        self.server.start()


    #Callback telling us what segment point the jointTrajectory controller is on, so that we can sync to it
    def trajSegmentCallback(self, msg):
        self.curr_traj_segment = msg.data


    #Send a command to the gripper action 
    def commandGripper(self, position, max_effort, blocking = 0):
        self.grip_goal.command.position = position
        self.grip_goal.command.max_effort = max_effort
        self.gripper_client.send_goal(self.grip_goal)
        
        #if blocking, wait for the gripper to finish
        if blocking:
            self.gripper_client.wait_for_result()


    def execute(self, goal):
        gripper_data = goal.gripper_traj
        success = True
        n_pts = len(gripper_data)
        
        #Try to synchronize gripper with joint trajectory
        i = -1
        while (i < n_pts and not rospy.is_shutdown()):
            # check that preempt has not been requested by the client
            if self.server.is_preempt_requested():
                print "GRIPPER TRAJ ACTION PREEMPTED"
                self.server.set_preempted()
                success = False
                break
            
            last_i = i
            i = self.curr_traj_segment-1
            
            #If this is a new point from the same traj, then command it
            if i >= n_pts:
                print "GripperTrajActionServer: Warning -- segment",i,"received, but only",n_pts,"exist"
            elif (i >= 0) and (i > last_i):
                self.commandGripper(gripper_data[i], -1)
                self.feedback.points_complete = i
                self.server.publish_feedback(self.feedback)
            
        if success:
            self.result.success = True
            print "GRIPPER TRAJ SUCCESS"   
            self.server.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('pr2_gripper_traj_action_server')
    r_server = PR2GripperTrajActionServer(0)
    l_server = PR2GripperTrajActionServer(1)
    rospy.spin()
