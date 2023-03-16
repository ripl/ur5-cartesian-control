#! /usr/bin/env python3

from tkinter import W
import rospy
import signal
from std_msgs.msg import String
from robotiq_s_interface import Gripper
from threading import BoundedSemaphore
import numpy as np

from scipy.spatial.transform import Rotation as R

from ur5_cartesian_control.trajectory_client import TrajectoryClient, Pose

eps = 1e-3

class JoystickControl:
    def __init__(self) -> None:
        # get parameters
        rospy.init_node('external_cartesian_control')


        # NOTE: No idea why ~joy_topic is not found.
        self.input_topic = rospy.get_param("~joy_topic", '/external_control')

        # Choices: ['basic', 'wide', 'pinch', 'scissor']
        self.gripper_mode = rospy.get_param("~gripper_mode", "pinch")
        self.gripper_open_value = rospy.get_param("~gripper_open_value", 0.0)
        self.gripper_close_value = rospy.get_param("~gripper_close_value", 1.0)

        self.pos_step = rospy.get_param("~pos_step", 0.08)
        self.rot_step = rospy.get_param("~rot_step", 0.6)

        # subscribe to joy commands
        self._sub_joy = rospy.Subscriber(self.input_topic, String, self.control_callback, queue_size=1)

        self.semaphore = BoundedSemaphore()

        # Instantiate TrajectoryClient
        self.traj_client = TrajectoryClient(confirm_before_motion=False)

        # Instantiate Gripper Interface
        self.gripper = Gripper(
            open_value=self.gripper_open_value,
            close_value=self.gripper_close_value,
        )
        self._initialize_gripper()

        # Initial pose
        self.init_pose = Pose(pos=(0.1, -0.4, 0.4), quat=(0.928, -0.371, 0, 0))
        self.prev_pose = self.init_pose

        self.preg_grip_input = 1.0  # Close

        # Move to the initial pose
        self.traj_client.send_cartesian_trajectory([self.prev_pose])

        # NOTE: Is it necessary??
        # listen for SIGINT
        signal.signal(signal.SIGINT, self._shutdown)

    def _initialize_gripper(self):
        self.gripper.activate()

        if self.gripper_mode == 'basic':
            self.gripper.basic_mode()
        elif self.gripper_mode == 'wide':
            self.gripper.wide_mode()
        elif self.gripper_mode == 'pinch':
            self.gripper.pinch_mode()
        elif self.gripper_mode == 'scissor':
            self.gripper.scissor_mode()
        else:
            raise ValueError(f'unknown mode: {self.gripper_mode}')

    def control_callback(self, msg):
        # return if another message is using the gripper
        gripper_is_busy = self.semaphore.acquire(blocking=False)
        if gripper_is_busy:
            return

        msg = [float(m) for m in msg.data.split()]

        quat = msg[3:7]

        traj = [Pose(pos=msg[:3], quat=quat)]

        # assert self.pos_step < 0.2, 'step >= 0.1 can be dangerous...'

        self.traj_client.send_cartesian_trajectory(traj, init_time=0.0, time_step=10)
        rospy.loginfo('sending trajectory to the client... DONE')

        # Open / Close the gripper
        if abs(msg[7]) > eps:
            if msg[7] > eps:
                self.gripper.close()
            else:
                self.gripper.open()

        # release lock and exit
        self.semaphore.release()

    def _shutdown(self, *args):
        rospy.loginfo('Shutting down...')
        self.gripper.shutdown()


if __name__ == '__main__':
    joy_con = JoystickControl()

    # keep spinning
    rospy.spin()
