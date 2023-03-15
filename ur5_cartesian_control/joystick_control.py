#! /usr/bin/env python3

from tkinter import W
import rospy
import signal
from ros_utils import *
from sensor_msgs.msg import Joy
from robotiq_s_interface import Gripper
from threading import BoundedSemaphore
import numpy as np

from scipy.spatial.transform import Rotation as R

from ur5_cartesian_control.trajectory_client import TrajectoryClient, Pose

# NOTE: Make sure that Logitech joystick is in "X" (Xbox?) mode. Not "D" (DirectInput) mode.
# buttons
# NOTE: Buttons
# BUTTON_A = 0
# BUTTON_B = 1
# BUTTON_X = 2
# BUTTON_Y = 3
# BUTTON_LB = 4
# BUTTON_RB = 5
# BUTTON_BACK = 6
# BUTTON_START = 7
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7

# NOTE: Axes
# idx 0: [left pad] -1.0 (right) <--> 1.0 (left)
# idx 1: [left pad] -1.0 (down) <--> 1.0 (up)
# idx 2: LT
# idx 3: [right pad] -1.0 (right) <--> 1.0 (left)
# idx 4: [right pad] -1.0 (down) <--> 1.0 (up)
# idx 5: RT
# idx 6: [left cross] -1.0 (right) <--> 1.0 (left)
# idx 7: [left cross] -1.0 (down) <--> 1.0 (up)

eps = 1e-3

class JoystickControl:
    def __init__(self) -> None:
        # get parameters
        rospy.init_node('joystick_cartesian_control')

        
        # NOTE: No idea why ~joy_topic is not found.
        self.joy_input_topic = rospy.get_param("~joy_topic", '/joy_teleop/joy')

        # Choices: ['basic', 'wide', 'pinch', 'scissor']
        self.gripper_mode = rospy.get_param("~gripper_mode", "pinch")
        self.gripper_open_value = rospy.get_param("~gripper_open_value", 0.0)
        self.gripper_close_value = rospy.get_param("~gripper_close_value", 1.0)

        self.pos_step = rospy.get_param("~pos_step", 0.08)
        self.rot_step = rospy.get_param("~rot_step", 0.6)

        # subscribe to joy commands
        self._sub_joy = rospy.Subscriber(self.joy_input_topic, Joy, self.joy_callback, queue_size=1)

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

    def joy_callback(self, msg):
        # return if another message is using the gripper
        gripper_is_busy = self.semaphore.acquire(blocking=False)
        if gripper_is_busy:
            return

        # Left pad
        lp_x, lp_y = msg.axes[0], msg.axes[1]
        rp_x, rp_y = msg.axes[3], msg.axes[4]
        reset = msg.buttons[BUTTON_Y]

        # Left cross (up / down <==> close / open)
        grip = msg.axes[7]


        assert self.pos_step < 0.2, 'step >= 0.1 can be dangerous...'
        if not (abs(lp_x) < eps and abs(lp_y) < eps and abs(rp_y) < eps):
            ROS_INFO(f'lp: {lp_x:.2f}, {lp_y:.2f}')
            ROS_INFO(f'rp: {rp_x:.2f}, {rp_y:.2f}')
            traj = self._generate_trajectory(dx=lp_x * self.pos_step, 
                                             dy=-lp_y * self.pos_step,
                                             dz=rp_y * self.pos_step,
                                             delta_euler_z=rp_x * self.rot_step)
            ROS_INFO(f'trajectory:\n{traj}')
            # ROS_INFO('sending trajectory to the client...')
            self.traj_client.send_cartesian_trajectory(traj, init_time=0.0, time_step=0.7)
            # ROS_INFO('sending trajectory to the client... DONE')

        elif reset:
            self.traj_client.send_cartesian_trajectory([self.init_pose])
            self.prev_pose = self.init_pose

        # Open / Close the gripper
        if abs(grip) > eps:
            if grip > eps:
                self.gripper.close()
            else:
                self.gripper.open()

        # release lock and exit
        self.semaphore.release()

    def _generate_trajectory(self, dx, dy, dz, delta_euler_z=0):
        delta_pos = np.array([dx, dy, dz])
        num_waypoints = 1
        traj = []
        for i in range(num_waypoints):
            new_pos = self.prev_pose.pos + delta_pos
            if abs(delta_euler_z) > eps:
                delta_pos[2] = 0.  # TEMP:

                euler = R.from_quat(self.prev_pose.quat).as_euler('xyz')
                euler += np.array([0, 0, delta_euler_z])
                quat = R.from_euler('xyz', euler).as_quat()
            else:
                quat = self.prev_pose.quat
            traj.append(
                Pose(pos=new_pos, quat=quat)
            )
            self.prev_pose = traj[-1]
        return traj

    def _shutdown(self, *args):
        ROS_INFO('Shutting down...')
        self.gripper.shutdown()


if __name__ == '__main__':
    joy_con = JoystickControl()

    # keep spinning
    rospy.spin()
