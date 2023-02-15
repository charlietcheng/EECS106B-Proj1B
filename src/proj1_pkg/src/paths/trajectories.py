#!/usr/bin/env/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse

"""
Set of classes for defining SE(3) trajectories for the end effector of a robot 
manipulator
"""

class Trajectory:

    def __init__(self, total_time):
        """
        Parameters
        ----------
        total_time : float
            desired duration of the trajectory in seconds 
        """
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        pass

    def display_trajectory(self, num_waypoints=67, show_animation=False, save_animation=False):
        """
        Displays the evolution of the trajectory's position and body velocity.
        Parameters
        ----------
        num_waypoints : int
            number of waypoints in the trajectory
        show_animation : bool
            if True, displays the animated trajectory
        save_animatioon : bool
            if True, saves a gif of the animated trajectory
        """
        trajectory_name = self.__class__.__name__
        times = np.linspace(0, self.total_time, num=num_waypoints)
        target_positions = np.vstack([self.target_pose(t)[:3] for t in times])
        target_velocities = np.vstack([self.target_velocity(t)[:3] for t in times])
        
        fig = plt.figure(figsize=plt.figaspect(0.5))
        colormap = plt.cm.brg(np.fmod(np.linspace(0, 1, num=num_waypoints), 1))

        # Position plot
        ax0 = fig.add_subplot(1, 2, 1, projection='3d')
        pos_padding = [[-0.1, 0.1],
                        [-0.1, 0.1],
                        [-0.1, 0.1]]
        ax0.set_xlim3d([min(target_positions[:, 0]) + pos_padding[0][0], 
                        max(target_positions[:, 0]) + pos_padding[0][1]])
        ax0.set_xlabel('X')
        ax0.set_ylim3d([min(target_positions[:, 1]) + pos_padding[1][0], 
                        max(target_positions[:, 1]) + pos_padding[1][1]])
        ax0.set_ylabel('Y')
        ax0.set_zlim3d([min(target_positions[:, 2]) + pos_padding[2][0], 
                        max(target_positions[:, 2]) + pos_padding[2][1]])
        ax0.set_zlabel('Z')
        ax0.set_title("%s evolution of\nend-effector's position." % trajectory_name)
        line0 = ax0.scatter(target_positions[:, 0], 
                        target_positions[:, 1], 
                        target_positions[:, 2], 
                        c=colormap,
                        s=2)

        # Velocity plot
        ax1 = fig.add_subplot(1, 2, 2, projection='3d')
        vel_padding = [[-0.1, 0.1],
                        [-0.1, 0.1],
                        [-0.1, 0.1]]
        ax1.set_xlim3d([min(target_velocities[:, 0]) + vel_padding[0][0], 
                        max(target_velocities[:, 0]) + vel_padding[0][1]])
        ax1.set_xlabel('X')
        ax1.set_ylim3d([min(target_velocities[:, 1]) + vel_padding[1][0], 
                        max(target_velocities[:, 1]) + vel_padding[1][1]])
        ax1.set_ylabel('Y')
        ax1.set_zlim3d([min(target_velocities[:, 2]) + vel_padding[2][0], 
                        max(target_velocities[:, 2]) + vel_padding[2][1]])
        ax1.set_zlabel('Z')
        ax1.set_title("%s evolution of\nend-effector's translational body-frame velocity." % trajectory_name)
        line1 = ax1.scatter(target_velocities[:, 0], 
                        target_velocities[:, 1], 
                        target_velocities[:, 2], 
                        c=colormap,
                        s=2)

        if show_animation or save_animation:
            def func(num, line):
                line[0]._offsets3d = target_positions[:num].T
                line[0]._facecolors = colormap[:num]
                line[1]._offsets3d = target_velocities[:num].T
                line[1]._facecolors = colormap[:num]
                return line

            # Creating the Animation object
            line_ani = animation.FuncAnimation(fig, func, frames=num_waypoints, 
                                                          fargs=([line0, line1],), 
                                                          interval=max(1, int(1000 * self.total_time / (num_waypoints - 1))), 
                                                          blit=False)
        plt.show()
        if save_animation:
            line_ani.save('%s.gif' % trajectory_name, writer='pillow', fps=60)
            print("Saved animation to %s.gif" % trajectory_name)

class LinearTrajectory(Trajectory):

    def __init__(self, points, total_time):
        """
        Remember to call the constructor of Trajectory
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        pass
        Trajectory.__init__(self,total_time)
        self.points = points
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        #return np.array([0, -(time**2 - self.total_time/2)+(self.total_time/2)**2, 0, 0, 1, 0, 0])

        start = self.points[0]
        end = self.points[1]
        half = self.total_time/2
        distance = [(end[0]-start[0]), (end[1]-start[1]) , (end[2]-start[2])]
        slope = [(4*distance[0])/(self.total_time**2), (4*distance[1])/(self.total_time**2), (4*distance[2])/(self.total_time**2)]
        if time<(self.total_time/2):
            p = [(slope[0]*(time**2))/2 + start[0], (slope[1]*(time**2))/2 + start[1], (slope[2]*(time**2))/2 + start[2]]
        else:
            p = [(-(slope[0]*time*(time-2*self.total_time))/2) - distance[0]+start[0], (-(slope[1]*time*(time-2*self.total_time))/2) - distance[1]+start[1], (-(slope[2]*time*(time-2*self.total_time))/2) - distance[2]+start[2]]

        print(p[0])
        return np.array([p[0], p[1], p[2], 0, 1, 0, 0])


        # if time==0:
        #     return np.array([start[0], start[1], start[2], 0, 1, 0, 0])

        # if time==self.total_time:
        #     return np.array([end[0], end[1], end[2], 0, 1, 0, 0])

        
        # return np.array([start[0]+self.total_time/(1+np.e**(-1.5*time+(0.75*self.total_time)))*slope[0], 
        #                 start[1]+self.total_time/(1+np.e**(-1.5*time+(0.75*self.total_time)))*slope[1], 
        #                 start[2]+self.total_time/(1+np.e**(-1.5*time+(0.75*self.total_time)))*slope[2], 0, 1, 0, 0])

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        The function get_g_matrix from utils may be useful to perform some frame
        transformations.
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """

        start = self.points[0]
        end = self.points[1]
        distance = [(end[0]-start[0]), (end[1]-start[1]) , (end[2]-start[2])]
        slope = [(4*distance[0])/(self.total_time**2), (4*distance[1])/(self.total_time**2), (4*distance[2])/(self.total_time**2)]
        print(slope)
        if time<(self.total_time/2):
            v = [slope[0]*time, slope[1]*time, slope[2]*time]
        else:
            v = [-slope[0]*(time-self.total_time), -slope[1]*(time-self.total_time), -slope[2]*(time-self.total_time)]

        return np.array([v[0], v[1], v[2], 0, 0, 0])
        
class CircularTrajectory(Trajectory):

    def __init__(self, center_position, radius, total_time):
        """
        Remember to call the constructor of Trajectory
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        pass
        Trajectory.__init__(self, 5)
        self.center = center_position
        self.radius = radius
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        slope = (8*np.pi)/(self.total_time**2)
        theta1 = 0.5*slope*time**2
        theta2 = -0.5*slope*time**2 + slope*self.total_time*time + np.pi - (3/8)*slope*(self.total_time**2)
        if time<(self.total_time/2):
            eq = theta1
        else:
            eq = theta2
        return np.array([self.radius*np.cos(eq) + self.center[0], self.radius*np.sin(eq) + self.center[1], self.center[2], 0, 1, 0, 0])

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        The function get_g_matrix from utils may be useful to perform some frame
        transformations.
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        slope = (8*np.pi)/(self.total_time**2)
        theta1 = 0.5*slope*time**2
        theta2 = -0.5*slope*time**2 + slope*self.total_time*time + np.pi - (3/8)*slope*(self.total_time**2)
        if time<(self.total_time/2):
            eq = theta1
            tdot = slope*time
        else:
            eq = theta2
            tdot = -slope*time + slope*self.total_time
        return np.array([-self.radius*np.sin(eq)*tdot , self.radius*np.cos(eq)*tdot, 0, 0, 0, 0])

class PolygonalTrajectory(LinearTrajectory):
    def __init__(self, points, total_time):
        """
        Remember to call the constructor of Trajectory.
        You may wish to reuse other trajectories previously defined in this file.
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        
        Trajectory.__init__(self,total_time)
        self.points = points
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        timeSplit = self.total_time/(len(self.points)-1)
        print(time)
        print(timeSplit)
        curr = int(time//timeSplit)
        print(curr)
        if curr+1<(len(self.points)-1) :
            trajectory = LinearTrajectory([self.points[curr], self.points[curr+1]], timeSplit)
        else: 
            trajectory = LinearTrajectory([self.points[curr], self.points[0]], timeSplit)
        
        return trajectory.target_pose(time-curr*timeSplit)

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        The function get_g_matrix from utils may be useful to perform some frame
        transformations.
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        timeSplit = self.total_time/(len(self.points)-1)
        print(time)
        print(timeSplit)
        curr = int(time//timeSplit)
        print(curr)
        if curr+1<(len(self.points)-1) :
            trajectory = LinearTrajectory([self.points[curr], self.points[curr+1]], timeSplit)
        else: 
            trajectory = LinearTrajectory([self.points[curr], self.points[0]], timeSplit)
        
        return trajectory.target_velocity(time-curr*timeSplit)

def define_trajectories(args):
    """ Define each type of trajectory with the appropriate parameters."""
    trajectory = None
    if args.task == 'line':
        trajectory = LinearTrajectory([[0,0,0], [10,10,0]], 20)
    elif args.task == 'circle':
        trajectory = CircularTrajectory([0,0,0], 3, 2*np.pi)
    elif args.task == 'polygon':
        trajectory = PolygonalTrajectory([[0,10,0], [5,10,0], [6,9,0], [0,11,0], [5,10,0]], 20)
    return trajectory

if __name__ == '__main__':
    """
    Run this file to visualize plots of your paths. Note: the provided function
    only visualizes the end effector position, not its orientation. Use the 
    animate function to visualize the full trajectory in a 3D plot.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, polygon.  Default: line'
    )
    parser.add_argument('--animate', action='store_true', help=
        'If you set this flag, the animated trajectory will be shown.'
    )
    args = parser.parse_args()

    trajectory = define_trajectories(args)
    
    if trajectory:
        trajectory.display_trajectory(show_animation=args.animate)
