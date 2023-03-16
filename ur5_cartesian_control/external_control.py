#! /usr/bin/env python3

from tkinter import W
import rospy
import signal
from std_msgs.msg import String
from robotiq_s_interface import Gripper
from threading import BoundedSemaphore
import numpy as np
from ros_utils import ROS_INFO, ROS_WARN

from scipy.spatial.transform import Rotation as R
from ur5_cartesian_control.helper import get_arm, get_planning_scene

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
        self._sub_joy = rospy.Subscriber(self.input_topic, String, self.control_callback, queue_size=None)

        self.semaphore = BoundedSemaphore()

        # Instantiate Gripper Interface
        self.gripper = Gripper(
            open_value=self.gripper_open_value,
            close_value=self.gripper_close_value,
        )
        self._initialize_gripper()
        self.preg_grip_input = 1.0  # Close

        # Instantiate TrajectoryClient
        self.traj_client = TrajectoryClient(confirm_before_motion=False)

        # ROS_INFO('Loading arm_controller...')
        # self.traj_client.switch_controller('arm_controller')
        # ROS_INFO('Loading arm_controller...DONE')
        # TODO: move to the init joint pos

        # TODO: table
        # NOTE: planning_scene: from moveit_commander import PlanningSceneInterface
        # NOTE: arm: from ur5_interface import Arm
        # arm = get_arm()
        # planning_scene, table = get_planning_scene(arm)
        # self.arm = arm
        # self._scene = planning_scene
        # self._scene._pub_co.publish(table)

        # init_joint_conf = [0.6547493934631348, -1.6995657126056116, 1.3889776468276978, 5.665678977966309, -1.5855563322650355, -2.5722370783435267] 
        # self.move_to_joint_conf(init_joint_conf)

        ROS_INFO('Loading forward_cartesian_traj_controller...')
        self.traj_client.switch_controller('forward_cartesian_traj_controller')
        ROS_INFO('Loading forward_cartesian_traj_controller...DONE')


        # Initial pose
        self.init_pose = Pose(pos=(0.1, -0.4, 0.4), quat=(0.928, -0.371, 0, 0))
        self.prev_pose = self.init_pose


        # self.traj_client.send_cartesian_trajectory([self.prev_pose])

        # NOTE: Is it necessary??
        # listen for SIGINT
        signal.signal(signal.SIGINT, self._shutdown)


    # def move_to_joint_conf(self, joint_conf):
    #     import numpy as np
    #     _, plan = self.arm.plan_to_joint_config(joint_conf)
    #     if plan is None:
    #         ROS_WARN("Planning failed.")
    #         raise RuntimeError('Planning Failed')
    #     success = self.arm.execute_plan(plan)
    #     if not success:
    #         ROS_WARN("Planning was successful but execution failed.")
    #         return success

    #     # make sure that the arm reached the OBSERVE pose
    #     epsilon = 0.05
    #     dist = np.linalg.norm( np.array(joint_conf)-np.array(self.arm.get_joint_values()) )
    #     if dist > epsilon:
    #         ROS_WARN("The Arm interface failed while trying to plan to the joint configuration. The Arm is probably in a weird pose.")
    #         return False
    #     return True


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
        gripper_is_free = self.semaphore.acquire(blocking=False)
        if not gripper_is_free:
            ROS_INFO('Gripper is busy...')
            return

        msg = [float(m) for m in msg.data.split()]

        quat = msg[3:7]

        new_pose = Pose(pos=msg[:3], quat=quat)
        traj = [self.prev_pose, new_pose]

        # Scale execution time with trajectory length
        traj_len = np.linalg.norm(new_pose.pos - self.prev_pose.pos)
        exec_time = max(1.0, traj_len * 10)

        self.prev_pose = new_pose

        # assert self.pos_step < 0.2, 'step >= 0.1 can be dangerous...'

        self.traj_client.send_cartesian_trajectory(traj, init_time=0.0, time_step=exec_time)

        # Open / Close the gripper
        if abs(msg[7]) > eps:
            if msg[7] > eps:
                self.gripper.close()
            else:
                self.gripper.open()

        # release lock and exit
        self.semaphore.release()

    def _shutdown(self, *args):
        ROS_INFO('Shutting down...')
        self.gripper.shutdown()


if __name__ == '__main__':
    joy_con = JoystickControl()

    # keep spinning
    rospy.spin()
