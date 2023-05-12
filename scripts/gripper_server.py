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
import numpy
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from threading import Thread
from pybullet_utils.srv import ChangeControlMode
from parallel_2f_gripper.srv import MoveGripper


def green_p(msg):
    print('\033[92m' + msg + '\033[0m')


def red_p(msg):
    print('\033[91m' + msg + '\033[0m')


def blue_p(msg):
    print('\033[94m' + msg + '\033[0m')


def yellow_p(msg):
    print('\033[93m' + msg + '\033[0m')


def cyan_p(msg):
    print('\033[96m' + msg + '\033[0m')


def move_gripper(srv, name, position, velocity, effort, joint_target_publish_rate, change_mode_srv, real_js):
    default_torque = 20
    mm_default_velocity = 160
    mm_max = 218
    mm_zero_joint = 100
    mm_crank_lenght = 70
    joint_max_val = 1.00
    joint_min_val = -0.80
    joint_tollerance = 0.01

    ratio_mm_to_rad = ((joint_max_val - joint_min_val) / (mm_max))
    default_velocity = mm_default_velocity * ratio_mm_to_rad

    control_mode = srv.control_mode
    target_position = numpy.arcsin(((srv.position / 2) - (mm_zero_joint / 2)) / mm_crank_lenght)
    target_velocity = ratio_mm_to_rad * (srv.velocity)
    target_torque = srv.torque


    if (control_mode != 'position' and control_mode != 'velocity' and control_mode != 'torque' and control_mode != 'open' and control_mode != 'close'):
        red_p('Wrong control mode')
        return 'false'
    if (control_mode == 'open'):
        target_position = joint_max_val
        print('Mode: open. Target_position: ' + str(target_position))

    if (control_mode == 'close'):
        target_position = joint_min_val
        print('Mode: close. Target_position: ' + str(target_position))

    if (control_mode == 'position'):
        if not target_position:
            if (target_position != 0):
                red_p('Position control mode require a target position')
                return 'false'
        print('Mode: position. Target_position: rad:' + str(target_position) + ' mm:' + str(srv.position))

    if (control_mode == 'torque'):
        if not target_torque:
            red_p('Torque control mode require a target torque')
            return 'false'
        if (target_torque > 0.0):
            target_position = joint_min_val
        else:
            target_position = joint_max_val
        print('Mode: torque. Target_position: ' + str(target_position))

    if (control_mode == 'velocity'):
        if not target_velocity:
            red_p('Velocty control mode require a target velocity')
            return 'false'
        if (target_velocity > 0.0):
            target_position = joint_min_val
        else:
            target_position = joint_max_val
        print('Mode: velocity. Target_position: ' + str(target_position))

    if not target_velocity:
        target_velocity = default_velocity
    print('            Target_velocity: ' + str(target_velocity))
    if not target_torque:
        target_torque = default_torque
    print('            Target_torque: ' + str(target_torque))

    green_p('A01')

    if (change_mode_srv(['parallel_gripper'], 'position') == 'false'):
        red_p('error with change_control_mode service')
        return 'false'

    green_p('A02')

    yellow_p('Wait for /parallel_gripper/joint_state message')
    current_joint_state = rospy.wait_for_message('/parallel_gripper/joint_states', JointState)
    yellow_p('/parallel_gripper/joint_state message recived')
    current_position = current_joint_state.position[current_joint_state.name.index('right_finger_joint')]

    finish = False
    if (control_mode == 'close'):
        dx = -abs(target_velocity) / joint_target_publish_rate
    elif (control_mode == 'open'):
        dx = abs(target_velocity) / joint_target_publish_rate
    else:
        if (current_position < target_position):
            dx = abs(target_velocity) / joint_target_publish_rate
        else:
            dx = -abs(target_velocity) / joint_target_publish_rate
    position[0] = current_position
    rate = rospy.Rate(joint_target_publish_rate)
    print('dx: ' + str(dx))
    while not finish:
        current_position += dx
        if (current_position > joint_max_val and dx > 0):
            current_position -= dx
            finish = True
            print('Joint limit: '+ str(current_position))
        position[0] = current_position
        real_position = real_js[0].position[real_js[0].name.index('right_finger_joint')]
        real_torque = real_js[0].effort[real_js[0].name.index('right_finger_joint')]
        if ((real_position < joint_min_val and dx < 0) or (real_position > joint_max_val and dx > 0)):
            finish = True
            print('Joint limit: '+ str(current_position))
        elif (dx < 0 and real_position < target_position + joint_tollerance):
            finish = True
            print('Target position: '+ str(current_position))
        elif (dx > 0 and real_position > target_position - joint_tollerance):
            finish = True
            print('Target position: '+ str(current_position))
        elif (((dx < 0) and (real_torque < -(target_torque * 2))) or ((dx > 0) and (real_torque > (target_torque * 2)))):
            finish = True
            print('Target_torque')
        rate.sleep()
    return 'true'


def jointStateSubscriber(data, args):
    real_js = args[0]
    real_js[0] = data


def jointTargetPublisher(jt_pub, joint_target_publish_rate, name, position, velocity, effort):
    rate = rospy.Rate(joint_target_publish_rate)
    jt_msg = JointState()
    while not rospy.is_shutdown():
        jt_msg.header = Header()
        jt_msg.header.stamp = rospy.Time.now()
        jt_msg.name = name
        jt_msg.position = position
        jt_msg.velocity = velocity
        jt_msg.effort = effort
        jt_pub.publish(jt_msg)
        rate.sleep()


def main(my_argv):
    if (len(my_argv) == 1):
        node_name = my_argv[1] + '_server'
    else:
        node_name = 'parallel_gripper_server'

    rospy.init_node(node_name)

    if (rospy.has_param('/joint_target_publish_rate')):
        joint_target_publish_rate = rospy.get_param('/joint_target_publish_rate')
    else:
        red_p('/joint_target_publish_rate pasam not set')
        raise SystemExit

    print('Wait for service change_control_mode...')
    rospy.wait_for_service('/change_control_mode')
    print('Connected to /change_control_mode')
    change_mode_srv = rospy.ServiceProxy('change_control_mode', ChangeControlMode)
    if (change_mode_srv(['parallel_gripper'], 'position') == 'false'):
        red_p('error with change_control_mode service')
        raise SystemExit

    initial_joint_state = rospy.wait_for_message('/parallel_gripper/joint_states', JointState)
    name = ['right_finger_joint']
    position = []
    velocity = []
    effort = []
    if name[0] in initial_joint_state.name:
        index = initial_joint_state.name.index(name[0])
        position.append(initial_joint_state.position[index])
        velocity.append(initial_joint_state.velocity[index])
        effort.append(initial_joint_state.effort[index])

    jt_msg = JointState()
    jt_msg.header = Header()
    jt_msg.header.stamp = rospy.Time.now()
    jt_msg.name = name
    jt_msg.position = position
    jt_msg.velocity = velocity
    jt_msg.effort = effort

    real_js = [rospy.wait_for_message('/parallel_gripper/joint_states', JointState)]
    rospy.Subscriber('/parallel_gripper/joint_states', JointState, jointStateSubscriber, (real_js, ))

    jt_pub = rospy.Publisher('/parallel_gripper/joint_target', JointState, queue_size=1)

    jt_pub.publish(jt_msg)

    jt_pub_thread = Thread(target=jointTargetPublisher, args=(jt_pub, joint_target_publish_rate, name, position, velocity, effort))
    jt_pub_thread.start()

    rospy.Service("/move_parallel_gripper", MoveGripper, lambda msg: move_gripper(msg, name, position, velocity, effort, joint_target_publish_rate, change_mode_srv, real_js))

    rospy.spin()
    jt_pub_thread.join()


if __name__ == '__main__':
    main(sys.argv)
