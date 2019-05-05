#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
# from scipy.integrate import quad, trapz
import sys
from copy import copy

import rospy
# from final_project_pkg.msg import BicycleCommandMsg, BicycleStateMsg
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf

class PathPlanner():
    def __init__(self, map_topic):
        """
        Turtlebot planner that uses sequential sinusoids to steer to a goal pose

        Parameters
        ----------
        l : float
            length of car
        """
        self.sub = rospy.Subscriber(map_topic, OccupancyGrid, self.updateOccupancyGrid)

    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=5, enable_proportional_feedback=False):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You 
        may or may not have to convert the state to a v state with state2v()
        This is a very optional function.  You may want to plan each component separately
        so that you can reset phi in case there's drift in phi 

        Parameters
        ----------
        start_state: :obj:`BicycleStateMsg`
        goal_state: :obj:`BicycleStateMsg`
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        x_path =        self.steer_x(
                            start_state, 
                            goal_state, 
                            0, 
                            dt, 
                            delta_t
                        )
        phi_path =      self.steer_phi(
                            x_path[-1][2], 
                            goal_state, 
                            x_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )
        alpha_path =    self.steer_alpha(
                            phi_path[-1][2], 
                            goal_state, 
                            phi_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )
        y_path =        self.steer_y(
                            alpha_path[-1][2], 
                            goal_state, 
                            alpha_path[-1][0] + dt, 
                            dt, 
                            delta_t
                        )     

        path = []
        for p in [x_path, phi_path, alpha_path, y_path]:
            path.extend(p)
        if not enable_proportional_feedback:
            return path
        else:
            return self.add_proportional_feedback(path, gamma=1, alpha=0.1, delta=1)

    def steer_x(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the x direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        start_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_x = goal_state_v[0] - start_state_v[0]

        v1 = delta_x/delta_t
        v2 = 0

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def state2v(self, state):
        """
        Takes a state in (x,y,theta,phi) coordinates and returns a state of (x,phi,alpha,y)

        Parameters
        ----------
        state : :obj:`BicycleStateMsg`
            some state

        Returns
        -------
        4x1 :obj:`numpy.ndarray` 
            x, phi, alpha, y
        """
        return np.array([state.x, state.phi, np.sin(state.theta), state.y])

    def v_path_to_u_path(self, path, start_state, dt):
        """
        convert a trajectory in v commands to u commands

        Parameters
        ----------
        path : :obj:`list` of (float, float, float)
            list of (time, v1, v2) commands
        start_state : :obj:`BicycleStateMsg`
            starting state of this trajectory
        dt : float
            how many seconds between timesteps in the trajectory

        Returns
        -------
        :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        def v2cmd(v1, v2, state):
            u1 = v1/np.cos(state.theta) 
            u2 = v2 
            # u1 = v1/np.cos(state.theta) * 0.1
            # u2 = v2 * 8
            return BicycleCommandMsg(u1, u2)

        curr_state = copy(start_state)
        for i, (t, v1, v2) in enumerate(path):
            cmd_u = v2cmd(v1, v2, curr_state)
            path[i] = [t, cmd_u, curr_state]

            curr_state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )

        return path

    def updateOccupancyGrid(self, msg):
        self.currentPosition = (msg.origin.position.x, msg.origin.position.y, msg.origin.position.z)
        current_orientation_xyzw = (msg.origin.orientation.x,
                                    msg.origin.orientation.y,
                                    msg.origin.orientation.z,
                                    msg.origin.orientation.w)
        self.currentOrientation = tf.transformations.euler_from_quaternion(current_orientation_xyzw)
        self.occupancyGrid = msg.data

# #!/usr/bin/env python
# """
# Path Planner Class for Lab 8
# Author: Valmik Prabhu
# """

# import sys
# import rospy
# import moveit_commander
# from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
# from geometry_msgs.msg import PoseStamped
# from shape_msgs.msg import SolidPrimitive

# class PathPlanner(object):
#     """
#     Path Planning Functionality for Baxter/Sawyer

#     We make this a class rather than a script because it bundles up 
#     all the code relating to planning in a nice way thus, we can
#     easily use the code in different places. This is a staple of
#     good object-oriented programming

#     Fields:
#     _robot: moveit_commander.RobotCommander; for interfacing with the robot
#     _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
#     _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
#     _planning_scene_publisher: ros publisher; publishes to the planning scene


#     """
#     def __init__(self, group_name):
#         """
#         Constructor.

#         Inputs:
#         group_name: the name of the move_group.
#             For Baxter, this would be 'left_arm' or 'right_arm'
#             For Sawyer, this would be 'right_arm'
#         """

#         # If the node is shutdown, call this function    
#         rospy.on_shutdown(self.shutdown)

#         # Initialize moveit_commander
#         moveit_commander.roscpp_initialize(sys.argv)

#         # Initialize the robot
#         self._robot = moveit_commander.RobotCommander()

#         # Initialize the planning scene
#         self._scene = moveit_commander.PlanningSceneInterface()

#         # This publishes updates to the planning scene
#         self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

#         # Instantiate a move group
#         self._group = moveit_commander.MoveGroupCommander(group_name)

#         # Set the maximum time MoveIt will try to plan before giving up
#         self._group.set_planning_time(5)

#         # Set the bounds of the workspace
#         self._group.set_workspace([-2, -2, -2, 2, 2, 2])

#         # Sleep for a bit to ensure that all inititialization has finished
#         rospy.sleep(0.5)

#     def shutdown(self):
#         """
#         Code to run on shutdown. This is good practice for safety

#         Currently deletes the object's MoveGroup, so that further commands will do nothing
#         """
#         self._group = None
#         rospy.loginfo("Stopping Path Planner")

#     def plan_to_pose(self, target, orientation_constraints=None):
#         """
#         Generates a plan given an end effector pose subject to orientation constraints

#         Inputs:
#         target: A geometry_msgs/PoseStamped message containing the end effector pose goal
#         orientation_constraints: A list of moveit_msgs/OrientationConstraint messages

#         Outputs:
#         path: A moveit_msgs/RobotTrajectory path
#         """

#         self._group.set_pose_target(target)
#         self._group.set_start_state_to_current_state()

#         if orientation_constraints is not None:
#             constraints = Constraints()
#             constraints.orientation_constraints = orientation_constraints
#             self._group.set_path_constraints(constraints)

#         plan = self._group.plan()

#         return plan

#     def plan_to_joint_pos(self, target_joints):
#         """
#         Generates a plan given an target joint state

#         Inputs:
#         target_joints : nx' :obj:`numpy.ndarray`
#             where n is the number of joints

#         Outputs:
#         path: A moveit_msgs/RobotTrajectory path
#         """
#         self._group.set_start_state_to_current_state()
#         self._group.set_joint_value_target(target_joints)
#         return self._group.plan()

#     def execute_plan(self, plan):
#         """
#         Uses the robot's built-in controllers to execute a plan

#         Inputs:
#         plan: a moveit_msgs/RobotTrajectory plan
#         """

#         return self._group.execute(plan, wait=True)


#     def add_box_obstacle(self, size, name, pose):
#         """
#         Adds a rectangular prism obstacle to the planning scene

#         Inputs:
#         size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
#         name: unique name of the obstacle (used for adding and removing)
#         pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
#         """    

#         # Create a CollisionObject, which will be added to the planning scene
#         co = CollisionObject()
#         co.operation = CollisionObject.ADD
#         co.id = name
#         co.header = pose.header

#         # Create a box primitive, which will be inside the CollisionObject
#         box = SolidPrimitive()
#         box.type = SolidPrimitive.BOX
#         box.dimensions = size

#         # Fill the collision object with primitive(s)
#         co.primitives = [box]
#         co.primitive_poses = [pose.pose]

#         # Publish the object
#         self._planning_scene_publisher.publish(co)

#     def remove_obstacle(self, name):
#         """
#         Removes an obstacle from the planning scene

#         Inputs:
#         name: unique name of the obstacle
#         """

#         co = CollisionObject()
#         co.operation = CollisionObject.REMOVE
#         co.id = name

#         self._planning_scene_publisher.publish(co)