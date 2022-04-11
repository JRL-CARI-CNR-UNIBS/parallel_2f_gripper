#!/usr/bin/env python3
"""
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import sys
import sensor_msgs.msg
import manipulation_msgs.srv

torque=10

def job_exec_srv(req):

    print("req.property_id: {}".format(req.property_id))
    string=req.property_id
    global torque
    if string.startswith('pos_'):
        string=req.property_id
    elif req.property_id == "close":
        print("Close")
        torque=-50
    elif req.property_id == "open":
        print("Open");
        torque=50
    rospy.sleep(1)
    print("executed")

    return manipulation_msgs.srv.JobExecutionResponse(0)



def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("parallel_2f_gripper")
    pub = rospy.Publisher('/gripper/joint_target', sensor_msgs.msg.JointState, queue_size=10)
    msg=sensor_msgs.msg.JointState()
    msg.name=['right_finger_joint']
    msg.position=[0]
    msg.velocity=[0]
    s = rospy.Service("/parallel_2f_gripper", manipulation_msgs.srv.JobExecution, job_exec_srv)
    r = rospy.Rate(10) # 10hz

    global torque
    print("start spinning")
    while not rospy.is_shutdown():
        msg.effort=[torque]
        msg.header.stamp=rospy.Time.now()
        pub.publish(msg)
        r.sleep()

    pub.publish(msg)

    rospy.spin()

if __name__ == '__main__':
    main()
