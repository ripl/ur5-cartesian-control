#! /usr/bin/env python3

from tkinter import W
import rospy
import signal
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from robotiq_s_interface import AdjustableGripper
from threading import BoundedSemaphore
import numpy as np
from ros_utils import ROS_INFO, ROS_WARN
import tf

from scipy.spatial.transform import Rotation as R
from ur5_cartesian_control.helper import get_arm, get_planning_scene, point2numpy

from ur5_cartesian_control.trajectory_client import TrajectoryClient
from ur5_cartesian_control.srv import CartesianMoveTo, CartesianMoveToResponse
from ur5_cartesian_control.srv import Grasp, GraspResponse
from ur5_cartesian_control.srv import LookupTransform, LookupTransformResponse

eps = 1e-3

class ExternalControlSrv:
    def __init__(self) -> None:
        # get parameters
        rospy.init_node('external_cartesian_control')


        # NOTE: No idea why ~joy_topic is not found.
        self.input_topic = rospy.get_param("~joy_topic", '/external_control')

        # Choices: ['basic', 'wide', 'pinch', 'scissor']
        self.gripper_mode = rospy.get_param("~gripper_mode", "pinch")
        self.gripper_force = rospy.get_param("~gripper_force", 10)

        self.pos_step = rospy.get_param("~pos_step", 0.08)
        self.rot_step = rospy.get_param("~rot_step", 0.6)

        self.semaphore = BoundedSemaphore()

        # Instantiate Gripper Interface
        ROS_INFO('Initializing Gripper...')
        self.gripper = AdjustableGripper(
            gripper_force=self.gripper_force  # This is fed to command.rFRA
        )
        self._initialize_gripper()
        ROS_INFO('Initializing Gripper...done')

        # Instantiate TrajectoryClient
        ROS_INFO('Instantiating Trajectory client...')
        self.traj_client = TrajectoryClient(confirm_before_motion=False)
        ROS_INFO('Instantiating Trajectory client...DONE')

        ROS_INFO('Loading forward_cartesian_traj_controller...')
        self.traj_client.switch_controller('forward_cartesian_traj_controller')
        ROS_INFO('Loading forward_cartesian_traj_controller...DONE')

        self._srv_move = rospy.Service('~execute_trajectory', CartesianMoveTo, self.move_to)
        self._srv_grip = rospy.Service('~grasp', Grasp, self.grasp)
        self._srv_tf = rospy.Service('~lookup_tf', LookupTransform, self.lookup_transform)
        self._tf_listener = tf.TransformListener()

        # Initial pose
        position = Point(x=0.1, y=-0.4, z=0.4)
        orientation = Quaternion(x=0.928, y=-0.371, z=0, w=0)

        self.init_pose = Pose(position=position, orientation=orientation)

        # Get current pose and move to the initial pose
        self._init_arm()

        # NOTE: Is it necessary??
        # listen for SIGINT
        signal.signal(signal.SIGINT, self._shutdown)

    def _get_curr_pose(self) -> Pose:
        # source_frame = '/camera_arm/camera_color_optical_frame'
        source_frame = '/ur_arm_tool0_controller'
        target_frame = '/ur_arm_base'
        timeout = 3.

        self._tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(timeout))
        trans, rot = self._tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        position = Point(x=trans[0], y=trans[1], z=trans[2])
        orientation = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])

        return Pose(position=position, orientation=orientation)

    def _init_arm(self) -> Pose:
        """Move to the initial position"""
        curr_pose = self._get_curr_pose()

        # Move to the init location
        traj = [curr_pose, self.init_pose]
        self.traj_client.send_cartesian_trajectory(traj, init_time=0.0, time_step=3)

    def lookup_transform(self, request: LookupTransform):
        """Call tf lookup to retrieve the transformation."""
        source_frame = request.source_frame.data
        target_frame = request.target_frame.data

        timeout = 3.
        self._tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(timeout))
        ROS_INFO(f'Looking up transform from {source_frame} to {target_frame}...')
        trans, rot = self._tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        ROS_INFO(f'Looking up transform from {source_frame} to {target_frame}...done')
        ROS_INFO(f'trans: {trans}\trot: {rot}')

        pose = Pose(position=Point(*trans), orientation=Quaternion(*rot))
        return LookupTransformResponse(transform=pose)

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

        self.gripper.grasp(1.0)  # Close the gripper

    def move_to(self, request: CartesianMoveTo):
        target_pose = request.target_pose
        exec_time = request.exec_time

        # Scale execution time with trajectory length
        curr_pose = self._get_curr_pose()
        init_pos = point2numpy(curr_pose.position)
        last_pos = point2numpy(target_pose.position)

        traj_len = np.linalg.norm(last_pos - init_pos)

        if exec_time is None:
            exec_time = max(1.0, traj_len * 10)
        else:
            exec_time = max(1.0, traj_len * 10, exec_time)

        traj = [curr_pose, target_pose]
        ROS_INFO(f'sending trajectory: {traj}')
        self.traj_client.send_cartesian_trajectory(traj, init_time=0.0, time_step=exec_time)
        ROS_INFO(f'sending trajectory: {traj} DONE')

        return CartesianMoveToResponse(success=True)

    def grasp(self, request: Grasp):
        # return if another message is using the gripper
        gripper_is_free = self.semaphore.acquire(blocking=False)
        if not gripper_is_free:
            ROS_WARN('Gripper is busy...')
            return GraspResponse(success=False)

        grip = request.grip

        # Open / Close the gripper
        # NOTE: 0: fully open, 1: fully close
        if grip < 0 or 1 < grip:
            ROS_WARN(f'Invalid grip value is recieved: {grip} (expected: 0 <= grip <= 1)')
        else:
            self.gripper.grasp(grip)

        # release lock and exit
        self.semaphore.release()
        return GraspResponse(success=True)

    def _shutdown(self, *args):
        ROS_INFO('Shutting down...')
        self.gripper.shutdown()


if __name__ == '__main__':
    joy_con = ExternalControlSrv()

    # keep spinning
    rospy.spin()
