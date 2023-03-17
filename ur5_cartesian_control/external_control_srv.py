#! /usr/bin/env python3

from tkinter import W
import rospy
import signal
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from robotiq_s_interface import Gripper
from threading import BoundedSemaphore
import numpy as np
from ros_utils import ROS_INFO, ROS_WARN

from scipy.spatial.transform import Rotation as R
from ur5_cartesian_control.helper import get_arm, get_planning_scene, point2numpy

from ur5_cartesian_control.trajectory_client import TrajectoryClient
from ur5_cartesian_control.srv import CartesianMoveAndGrip, CartesianMoveAndGripResponse

eps = 1e-3

class ExternalControlSrv:
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

        self.semaphore = BoundedSemaphore()

        # Instantiate Gripper Interface
        self.gripper = Gripper(
            open_value=self.gripper_open_value,
            close_value=self.gripper_close_value,
        )
        self._initialize_gripper()
        self.preg_grip_input = 1.0  # Close

        ROS_INFO("hello world!!!")
        # Instantiate TrajectoryClient
        ROS_INFO('Instantiating Trajectory client...')
        self.traj_client = TrajectoryClient(confirm_before_motion=False)
        ROS_INFO('Instantiating Trajectory client...DONE')

        self._srv = rospy.Service('~execute_trajectory', CartesianMoveAndGrip, self.control_callback)

        ROS_INFO('Loading forward_cartesian_traj_controller...')
        self.traj_client.switch_controller('forward_cartesian_traj_controller')
        ROS_INFO('Loading forward_cartesian_traj_controller...DONE')


        # Initial pose
        position = Point(x=0.1, y=-0.4, z=0.4)
        orientation = Quaternion(x=0.928, y=-0.371, z=0, w=0)

        self.init_pose = Pose(position=position, orientation=orientation)
        self.prev_pose = self.init_pose


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

    def control_callback(self, request):
        # return if another message is using the gripper
        gripper_is_free = self.semaphore.acquire(blocking=False)
        if not gripper_is_free:
            ROS_INFO('Gripper is busy...')
            return CartesianMoveAndGripResponse(success=False)

        target_pose = request.target_pose
        exec_time = request.exec_time
        grip = request.grip

        # Scale execution time with trajectory length
        init_pos = point2numpy(self.prev_pose.position)
        last_pos = point2numpy(target_pose.position)

        traj_len = np.linalg.norm(last_pos - init_pos)

        if exec_time is None:
            exec_time = max(1.0, traj_len * 10)
        else:
            exec_time = max(1.0, traj_len * 10, exec_time)

        traj = [self.prev_pose, target_pose]
        ROS_INFO(f'sending trajectory: {traj}')
        self.traj_client.send_cartesian_trajectory(traj, init_time=0.0, time_step=exec_time)
        ROS_INFO(f'sending trajectory: {traj} DONE')

        self.prev_pose = target_pose

        # Open / Close the gripper
        if grip:
            self.gripper.close()
        else:
            self.gripper.open()

        # release lock and exit
        self.semaphore.release()
        return CartesianMoveAndGripResponse(success=True)

    def _shutdown(self, *args):
        ROS_INFO('Shutting down...')
        self.gripper.shutdown()


if __name__ == '__main__':
    joy_con = ExternalControlSrv()

    # keep spinning
    rospy.spin()
