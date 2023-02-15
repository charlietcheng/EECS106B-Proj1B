#!/usr/bin/env python

"""
Starter script for Project 1B. 
Author: Chris Correa, Valmik Prabhu
"""

# Python imports
import sys
import numpy as np
import itertools
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from paths.trajectories import LinearTrajectory
from sawyer_pykdl import sawyer_kinematics
from paths.paths import MotionPath
# Lab imports
from utils.utils import *
from trac_ik_python.trac_ik import IK
# ROS imports
try:
    import tf
    import tf2_ros
    import rospy
    import baxter_interface
    import intera_interface
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotTrajectory
except:
    pass

NUM_JOINTS = 7

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  
    Parameters
    ----------
    tag_number : int
    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    to_frame = 'ar_marker_{}'.format(tag_number)
    # to_frame = 'ar_marker_13'
    try:
        trans = tfBuffer.lookup_transform('base', to_frame, rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    # print('tag is found', tag_pos)

    return np.array(tag_pos)

def get_trajectory(limb, kin, ik_solver, tag_pos):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    num_way = 200
    controller_name = 'workspace'
    task = 'line'

    # target_position = tag_pos[0]
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tag_pos = [lookup_tag('15')]
    try:
        
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
        print('except')
    except Exception as e:
        print(e)
    

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    # import pdb 
    # pdb.set_trace()
    # print("Current Position:", current_position)
    # print("target Position:", tag_pos[0])

    if task == 'line':
        trajectory = LinearTrajectory([[current_position[0], current_position[1], current_position[2]], [tag_pos[0][0],tag_pos[0][1], tag_pos[0][2]+0.5]], 2)
    else:
        raise ValueError('task {} not recognized'.format(task))
    path = MotionPath(limb, kin, ik_solver, trajectory)
    
    return path.to_robot_trajectory(num_way, controller_name!='workspace')


class Controller:

    def __init__(self, limb, kin):
        """
        Constructor for the superclass. All subclasses should call the superconstructor

        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        """

        # Run the shutdown function when the ros node is shutdown
        rospy.on_shutdown(self.shutdown)
        self._limb = limb
        self._kin = kin

        # Set this attribute to True if the present controller is a jointspace controller.
        self.is_jointspace_controller = False

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly.  

        Parameters
        ----------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        """
        pass

    def interpolate_path(self, path, t, current_index = 0):
        """
        interpolates over a :obj:`moveit_msgs.msg.RobotTrajectory` to produce desired
        positions, velocities, and accelerations at a specified time

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        t : float
            the time from start
        current_index : int
            waypoint index from which to start search

        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        current_index : int
            waypoint index at which search was terminated 
        """

        # a very small number (should be much smaller than rate)
        epsilon = 0.0001

        max_index = len(path.joint_trajectory.points)-1

        # If the time at current index is greater than the current time,
        # start looking from the beginning
        if (path.joint_trajectory.points[current_index].time_from_start.to_sec() > t):
            current_index = 0

        # Iterate forwards so that you're using the latest time
        while (
            not rospy.is_shutdown() and \
            current_index < max_index and \
            path.joint_trajectory.points[current_index+1].time_from_start.to_sec() < t+epsilon
        ):
            current_index = current_index+1

        # Perform the interpolation
        if current_index < max_index:
            time_low = path.joint_trajectory.points[current_index].time_from_start.to_sec()
            time_high = path.joint_trajectory.points[current_index+1].time_from_start.to_sec()

            target_position_low = np.array(
                path.joint_trajectory.points[current_index].positions
            )
            target_velocity_low = np.array(
                path.joint_trajectory.points[current_index].velocities
            )
            target_acceleration_low = np.array(
                path.joint_trajectory.points[current_index].accelerations
            )

            target_position_high = np.array(
                path.joint_trajectory.points[current_index+1].positions
            )
            target_velocity_high = np.array(
                path.joint_trajectory.points[current_index+1].velocities
            )
            target_acceleration_high = np.array(
                path.joint_trajectory.points[current_index+1].accelerations
            )

            target_position = target_position_low + \
                (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + \
                (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)
            target_acceleration = target_acceleration_low + \
                (t - time_low)/(time_high - time_low)*(target_acceleration_high - target_acceleration_low)

        # If you're at the last waypoint, no interpolation is needed
        else:
            target_position = np.array(path.joint_trajectory.points[current_index].positions)
            target_velocity = np.array(path.joint_trajectory.points[current_index].velocities)
            target_acceleration = np.array(path.joint_trajectory.points[current_index].velocities)

        return (target_position, target_velocity, target_acceleration, current_index)


    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.stop_moving()
        rospy.sleep(0.1)

    def stop_moving(self):
        """
        Set robot joint velocities to zero
        """
        zero_vel_dict = joint_array_to_dict(np.zeros(NUM_JOINTS), self._limb)
        self._limb.set_joint_velocities(zero_vel_dict)

    def plot_results(
        self,
        times,
        actual_positions, 
        actual_velocities, 
        target_positions, 
        target_velocities
    ):
        """
        Plots results.
        If the path is in joint space, it will plot both workspace and jointspace plots.
        Otherwise it'll plot only workspace

        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx7 or nx6 :obj:`numpy.ndarray`
            actual joint positions for each time in times
        actual_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            actual joint velocities for each time in times
        target_positions: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace positions for each time in times
        target_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)

        # Find the actual workspace positions and velocities
        actual_workspace_positions = np.zeros((len(times), 3))
        actual_workspace_velocities = np.zeros((len(times), 3))
        actual_workspace_quaternions = np.zeros((len(times), 4))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(actual_positions[i], self._limb)
            fk = self._kin.forward_position_kinematics(joint_values=positions_dict)
            
            actual_workspace_positions[i, :] = fk[:3]
            actual_workspace_velocities[i, :] = \
                self._kin.jacobian(joint_values=positions_dict)[:3].dot(actual_velocities[i])
            actual_workspace_quaternions[i, :] = fk[3:]
        # check if joint space
        if self.is_jointspace_controller:
            # it's joint space

            target_workspace_positions = np.zeros((len(times), 3))
            target_workspace_velocities = np.zeros((len(times), 3))
            target_workspace_quaternions = np.zeros((len(times), 4))

            for i in range(len(times)):
                positions_dict = joint_array_to_dict(target_positions[i], self._limb)
                target_workspace_positions[i, :] = \
                    self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
                target_workspace_velocities[i, :] = \
                    self._kin.jacobian(joint_values=positions_dict)[:3].dot(target_velocities[i])
                target_workspace_quaternions[i, :] = np.array([0, 1, 0, 0])

            # Plot joint space
            plt.figure()
            joint_num = len(self._limb.joint_names())
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Position Error")
                plt.legend()

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Velocity Error")
                plt.legend()
            print("Close the plot window to continue")
            plt.show()

        else:
            # it's workspace
            target_workspace_positions = target_positions
            target_workspace_velocities = target_velocities
            target_workspace_quaternions = np.zeros((len(times), 4))
            target_workspace_quaternions[:, 1] = 1

        plt.figure()
        workspace_joints = ('X', 'Y', 'Z')
        joint_num = len(workspace_joints)
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_workspace_positions[:,joint], label='Actual')
            plt.plot(times, target_workspace_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Position Error")
            plt.legend()

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_velocities[:,joint], label='Actual')
            plt.plot(times, target_velocities[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Velocity Error")
            plt.legend()

        print("Close the plot window to continue")
        plt.show()

        # Plot orientation error. This is measured by considering the
        # axis angle representation of the rotation matrix mapping
        # the desired orientation to the actual orientation. We use
        # the corresponding angle as our metric. Note that perfect tracking
        # would mean that this "angle error" is always zero.
        angles = []
        for t in range(len(times)):
            quat1 = target_workspace_quaternions[t]
            quat2 = actual_workspace_quaternions[t]
            theta = axis_angle(quat1, quat2)
            angles.append(theta)

        plt.figure()
        plt.plot(times, angles)
        plt.xlabel("Time (s)")
        plt.ylabel("Error Angle of End Effector (rad)")
        print("Close the plot window to continue")
        plt.show()
        

    def execute_path(self, path, rate=200, timeout=None, log=False):
        """
        takes in a path and moves the baxter in order to follow the path.  

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(path, t, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break

        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True

    def follow_ar_tag(self, tag, rate=200, timeout=None, log=False):
        """
        takes in an AR tag number and follows it with the baxter's arm.  You 
        should look at execute_path() for inspiration on how to write this. 

        Parameters
        ----------
        tag : int
            which AR tag to use
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """
        while True: 

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            try:
                # import pdb 
                # pdb.set_trace()
                trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
                print('except')
            except Exception as e:
                print(e)

            # tfBuffer = tf2_ros.Buffer()    #// This creates a buffer of transforms!
            # listener = tf2_ros.TransformListener(tfBuffer) #// This puts transforms into the buffer
            trans_ar = tfBuffer.lookup_transform('base', 'ar_marker_15', rospy.Time(0), rospy.Duration(5.0))


            current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
            current_velocity = get_joint_velocities(self._limb)
            print(current_position)
            tag_pos = np.array([getattr(trans_ar.transform.translation, dim) for dim in ('x', 'y', 'z')])
            
            kin = sawyer_kinematics('right')
            ik_solver = IK("base", "right_hand")
            limb = intera_interface.Limb('right')
            # import pdb ([])
            # pdb.set_trace()
            trajectory = LinearTrajectory([[current_position[0], current_position[1], current_position[2]], [tag_pos[0],tag_pos[1], tag_pos[2]+0.5]], 10)
            # path = MotionPath(limb, kin, ik_solver, trajectory)
            # import pdb 
            # pdb.set_trace()
            path = get_trajectory(limb, kin, ik_solver, trajectory)
            # path = MotionPath(limb, kin, ik_solver, trajectory)
            # import pdb
            # pdb.set_trace()

                # For plotting
            # if log:path
            #     times = list()
            #     actual_positions = list()
            #     actual_velocities = list()
            #     target_positions = list()
            #     target_velocities = list()

            # For interpolation
            max_index = len(path.joint_trajectory.points)-1
            current_index = 0

            # For timing
            start_t = rospy.Time.now()
            r = rospy.Rate(rate)

            while not rospy.is_shutdown():
                # Find the time from start
                t = (rospy.Time.now() - start_t).to_sec()

                # If the controller has timed out, stop moving and return false
                if timeout is not None and t >= timeout:
                    # Set velocities to zero
                    self.stop_moving()
                    return False

                current_position = get_joint_positions(self._limb)
                current_velocity = get_joint_velocities(self._limb)

                # Get the desired position, velocity, and effort
                (
                    target_position, 
                    target_velocity, 
                    target_acceleration, 
                    current_index
                ) = self.interpolate_path(path, t, current_index)
                # import pdb 
                # pdb.set_trace()
                # rob_pos = np.zeros(7,)
                # rob_pos[:3] = tag_pos
                # rob_pos[4] = 1
                # rob_pos[2] = 0.5
                # print(rob_pos)
                # import pdb 
                # pdb.set_trace()
                tg_position = np.array([tag_pos[0], tag_pos[1], tag_pos[2], 0,1,0,0])
                tg_position[2] = current_position[2]
                tg_velocity = np.array([[(tg_position[0]-current_position[0])/10], [(tg_position[1]-current_position[1])/10], [(tg_position[2]-current_position[2])/10]])
                # target_velocity[4] = 1
                # import pdb 
                # pdb.set_trace()
                #tg_velocity[:3] = rob_pos[:3] - current_position[:3] / 100
                tg_acceleration = np.array([])
                import pdb 
                pdb.set_trace()
                # Run controller
                self.step_control(tg_position, tg_velocity, tg_acceleration)
                # self.step_control(tag_pos, current_position - target_position /5, target_acceleration)


                # Sleep for a bit (to let robot move)
                r.sleep()

                # if current_index >= max_index:
                #     self.stop_moving()
                    # continue

    def follow_ar_tag1(self, tag, rate=200, timeout=None, log=False):
        r = rospy.Rate(rate)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()


        while not rospy.is_shutdown(): 
            t = (rospy.Time.now() - start_t).to_sec()

            try:
                trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
            except Exception as e:
                print(e)
            
            current_position = self._kin.forward_position_kinematics()
            
            trans_ar = tfBuffer.lookup_transform('base', 'ar_marker_16', rospy.Time(0), rospy.Duration(10.0))

            tag_pos = np.array([getattr(trans_ar.transform.translation, dim) for dim in ('x', 'y', 'z')])


            target_position = np.array([tag_pos[0], tag_pos[1], current_position[2] ,0, 1, 0, 0])
            #print(trans)

            mag = np.linalg.norm(target_position[:3]-current_position[:3])
            dir = (target_position[:3]-current_position[:3])/mag

            target_velocity = np.array([dir[0]/40, dir[1]/40, 0, 0, 0, 0])

            print(target_velocity)
            tg_acceleration = np.array([])

            WorkspaceVelocityController.step_control(self, target_position, target_velocity, tg_acceleration)
            r.sleep()

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

        if log:
            self.plot_results(
            times,
            actual_positions, 
            actual_velocities, 
            target_positions, 
            target_velocities
        )

class FeedforwardJointVelocityController(Controller):
    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        self._limb.set_joint_velocities(joint_array_to_dict(target_velocity, self._limb))

class WorkspaceVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the
    PDJointVelocityController is that this controller compares the baxter's current WORKSPACE position and
    velocity desired WORKSPACE position and velocity to come up with a WORKSPACE velocity command to be sent
    to the baxter.  Then this controlcontrol_inputler should convert that WORKSPACE velocity command into a joint velocity
    command and sends that to the baxter.  Notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.eye(6)*Kp
        # self.Kp[0][0] = 0.0001
        # self.Kp[1][1] = 0.0001
        # self.Kp[2][2] = 0.00001
        self.Kv = np.eye(6)*Kv
        self.is_jointspace_controller = False




    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Makes a call to the robot to move according to its current position and the desired position 
        according to the input path and the current time.
        target_position will be a 7 vector describing the desired SE(3) configuration where the first
        3 entries are the desired position vector and the next 4 entries are the desired orientation as
        a quaternion, all written in spatial coordinates.
        target_velocity is the body-frame se(3) velocity of the desired SE(3) trajectory gd(t). This velocity
        is given as a 6D Twist (vx, vy, vz, wx, wy, wz).
        This method should call self._kin.forward_position_kinematics() to get the current workspace 
        configuration and self._limb.set_joint_velocities() to set the joint velocity to something.  
        Remember that we want to track a trajectory in SE(3), and implement the controller described in the
        project document PDF.
        Parameters
        ----------
        target_position: (7,) ndarray of desired SE(3) position (px, py, pz, qx, qy, qz, qw) (position + quaternion).
        target_velocity: (6,) ndarray of desired body-frame se(3) velocity (vx, vy, vz, wx, wy, wz).
        target_acceleration: ndarray of desired accelerations (should you need this?).
        """
        
        control_input = None
        curr = get_joint_positions(self._limb)

        gst = get_g_matrix(np.array([curr[0], curr[1], curr[2]]),np.array([curr[3], curr[4], curr[5], curr[6]] ))
        gsd = get_g_matrix(np.array([target_position[0], target_position[1], target_position[2]]),np.array([target_position[3], target_position[4], target_position[5], target_position[6]] ))
        #Vsd,gsd = target_velocity
        # import pdb controller = get_controller(args.controller_name, limb, kin)
        # pdb.set_trace()
        gtd = np.linalg.inv(gst)@gsd
        w_hat = gtd[:3,:3]
        w = np.array([gtd[1][2], gtd[2][0],gtd[0][1]])
        v = gtd[0:3,3]
        Ainv = np.eye(3) - (1/2) * w_hat +( (2 * sin(length(w))) - (length(w) * (1 + cos(length(w))))) * (w_hat **2) / (2 * length(w)**2 * sin(length(w))) 
        zi_td = np.hstack((w, Ainv@v))
        # zi_td = vee(log(gtd))
        zis_td = adj(gst)@zi_td
        zis_td = zis_td.reshape(6,1)
        target_velocity = target_velocity.reshape((6,1))
        Us = self.Kp@zis_td+ target_velocity

        J = self._kin.jacobian()
        pinv = np.linalg.pinv(J)
        control_input = pinv@Us

        self._limb.set_joint_velocities(joint_array_to_dict(control_input, self._limb))



class PDJointVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the 
    PDJointVelocityController is that this controller turns the desired workspace position and velocity
    into desired JOINT position and velocity.  Then it compares the difference between the baxter's 
    current JOINT position and velocity and desired JOINT position and velocity to come up with a
    joint velocity command and sends that to the baxter.  notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.eye(7)*Kp
        self.Kv = np.eye(7)*Kv
        self.is_jointspace_controller = True

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. his method should call
        get_joint_positions and get_joint_velocities from the utils package to get the current joint 
        position and velocity and self._limb.set_joint_velocities() to set the joint velocity to something.  
        You may find joint_array_to_dict() in utils.py useful as well.

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        control_input = None
        curr = get_joint_positions(self._limb)
        currv = get_joint_velocities(self._limb)
        e = target_position - curr 
        edot = target_velocity - currv 
        control_input = self.Kp@e + self.Kv@edot

        print(control_input)

        self._limb.set_joint_velocities(joint_array_to_dict(control_input, self._limb))

class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.eye(7)*Kp
        self.Kv = np.eye(7)*Kv
        self.is_jointspace_controller = True

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Makes a call to the robot to move according to its current position and the desired position 
        according to the input path and the current time. This method should call
        get_joint_positions and get_joint_velocities from the utils package to get the current joint 
        position and velocity and self._limb.set_joint_torques() to set the joint torques to something. 
        You may find joint_array_to_dict() in utils.py useful as well.
        Recall that in order to implement a torque based controller you will need access to the 
        dynamics matrices M, C, G such that
        M ddq + C dq + G = u
        For this project, you will access the inertia matrix and gravity vector as follows:
        Inertia matrix: self._kin.inertia(positions_dict)
        Coriolis matrix: self._kin.coriolis(positions_dict, velocity_dict)
        Gravity matrix: self._kin.gravity(positions_dict) (You might want to scale this matrix by 0.01 or another scalar)
        These matrices were computed by a library and the coriolis matrix is approximate, 
        so you should think about what this means for the kinds of trajectories this 
        controller will be able to successfully track.
        Look in section 4.5 of MLS.
        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """

        #converters
        target_acceleration = target_acceleration.reshape(7,1)
        # target_velocity = target_velocity.reshape(7,1)
        # target_position = target_position.reshape(7,1)
        
        # control_input = None
        curr = get_joint_positions(self._limb)
        positions_dict = joint_array_to_dict(curr, self._limb)
        currv = get_joint_velocities(self._limb)
        velocity_dict = joint_array_to_dict(currv, self._limb)
    
        # currv = currv.reshape(7,1)
        M = self._kin.inertia(positions_dict).reshape(7,7)
        C = self._kin.coriolis(positions_dict, velocity_dict)
        G = 0.01*self._kin.gravity(positions_dict)
       
        
        # M = M.reshape(-1)
        # C = C.reshape(-1)
        # G = G.reshape(-1)
        # qdot = target_velocity - currv 
        # qdot = qdot.reshape(7,1)
        # print("M:", M)
        # print("C:", C.shape)
        # print("G:", G.shape)
        # print("qd:", qdot.shape)
        

        # print()
        # tau = np.matmul(M,(target_acceleration)) + C + G + control_U
        # control_input = np.array(tau[0])
        # control_input = control_input[0].T
        # print(control_input.shape)
        # import pdb
        # pdb.set_trace()

        e = target_position - curr 
        e = e.reshape(7,1)
        edot = target_velocity - currv 
        edot = edot.reshape(7,1)
        #control_U = self.Kp@e # + self.Kv@edot
        print(e)
        control_input = M @ target_acceleration + C + G + self.Kp@e +self.Kv@edot
        print("output:", control_input)

        self._limb.set_joint_torques(joint_array_to_dict(control_input, self._limb))
