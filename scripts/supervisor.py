#!/usr/bin/env python3

from enum import Enum
from visualization_msgs.msg import Marker
from time import sleep
import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
from frontier import frontier
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from utils.grids import StochOccupancyGrid2D
import tf
import numpy as np

NUM_ANIMALS = 2

class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    USER_INP = 7
    EXPLORE = 8
    WAYPOINT = 9
    FINAL_RETURN = 10
    DONE = 11


class SupervisorParams:

    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = rospy.get_param("sim")

        # How is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = rospy.get_param("rviz")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = rospy.get_param("map")

        # Threshold at which we consider the robot at a location
        self.pos_eps = rospy.get_param("~pos_eps", 0.15) # 0.1, works without pose at .4
        self.theta_eps = rospy.get_param("~theta_eps", 3) # 0.3

        # Time to stop at a stop sign
        self.stop_time = rospy.get_param("~stop_time", 2.)

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 0.5) # was formerly 0.3m

        # Time taken to cross an intersection
        self.crossing_time = rospy.get_param("~crossing_time", 3.)

        if verbose:
            print("SupervisorParams:")
            print("    use_gazebo = {}".format(self.use_gazebo))
            print("    rviz = {}".format(self.rviz))
            print("    mapping = {}".format(self.mapping))
            print("    pos_eps, theta_eps = {}, {}".format(self.pos_eps, self.theta_eps))
            print("    stop_time, stop_min_dist, crossing_time = {}, {}, {}".format(self.stop_time, self.stop_min_dist, self.crossing_time))


class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.params = SupervisorParams(verbose=True)

        self.animal_list = {"cat": (), "dog": (), "elephant": (), "bird": (), "giraffe": ()}
        self.list_to_rescue = set()

        # Current state
        self.x = 0
        self.y = 0
        self.theta = 0

        # Goal state
        self.x_g = 1.5
        self.y_g = -4.0
        self.theta_g = 0.0

        # Current mode
        self.mode = Mode.EXPLORE # NAV
        self.prev_mode = None  # For printing purposes

        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        self.occupancy = None
        self.occupancy_updated = False
        self.plan_resolution = 0.1

        ########## PUBLISHERS ##########

        # Command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

        # Command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ########## SUBSCRIBERS ##########

        # Stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        # Animal detector
        rospy.Subscriber('/detector/animals', DetectedObject, self.animal_sound)

        # High-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)

        # Map for frontier
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/map_metadata", MapMetaData, self.map_md_callback)

        # List of animals
        rospy.Subscriber('/detector/animals', DetectedObject, self.animal_sound)

        # List of animals to rescue
        rospy.Subscriber('/list_to_rescue', DetectedObjectList, self.rescue_animals_callback)

        # If using gazebo, we have access to perfect state
        if self.params.use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()

        # If using rviz, we can subscribe to nav goal click
        if self.params.rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        else:
            self.x_g, self.y_g, self.theta_g = 1.5, -4., 0.
            self.mode = Mode.NAV
        

    ########## SUBSCRIBER CALLBACKS ##########

    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if self.params.mapping else "/odom"
        print("Rviz command received!")
        
        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (nav_pose_origin.pose.orientation.x,
                          nav_pose_origin.pose.orientation.y,
                          nav_pose_origin.pose.orientation.z,
                          nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def rescue_animals_callback(self, msg):
        if self.mode == Mode.USER_INP:
            for animal in msg.objects:
                self.list_to_rescue.add(animal)

    def animal_sound(self, msg):
        if self.mode == Mode.EXPLORE:
            dist = msg.distance
            # Coordinates of the object
            thresh_dist = 0.5

            angle = self.theta + (msg.thetaleft + msg.thetaright) / 2
            
            x_coord = self.x + dist * np.cos(angle) - thresh_dist * np.cos(angle)
            y_coord = self.y + dist * np.sin(angle) - thresh_dist * np.sin(angle)

            self.animal_list[msg.name] = ((x_coord, y_coord), self.theta)
            """
            marker = Marker()
            id_d = {"cat": 1, "dog": 2, "elephant":3}
            vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)
 
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()

            # IMPORTANT: If you're creating multiple markers, 
            #            each need to have a separate marker ID.
            marker.id = id_d[msg.name]

            marker.type = 2 # sphere

            marker.pose.position.x = x_coord
            marker.pose.position.y = y_coord
            marker.pose.position.z = 1

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 10
            marker.scale.y = 10
            marker.scale.z = 10

            marker.color.a = 1 # Don't forget to set the alpha!
            marker.color.r = 50 *  marker.id
            marker.color.g = 10 *  marker.id
            marker.color.b = 100 *  marker.id
            
            vis_pub.publish(marker) """

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.params.stop_min_dist and self.mode == Mode.NAV:
            self.init_stop_sign()
    
    def map_md_callback(self, msg):
        """
        receives maps meta data and stores it
        """
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x, msg.origin.position.y)

    def map_callback(self, msg):
        """
        receives new map info and updates the map
        """
        #print("got map")
        self.map_probs = msg.data
        
        # if we've received the map metadata and have a way to update it:
        if (
            self.map_width > 0
            and self.map_height > 0
            and len(self.map_probs) > 0
        ):
            self.occupancy = StochOccupancyGrid2D(
                self.map_resolution,
                self.map_width,
                self.map_height,
                self.map_origin[0],
                self.map_origin[1],
                7,
                self.map_probs,
            )
            
            #if self.x_g is not None:
                # if we have a goal to plan to, replan
            #    rospy.loginfo("replanning because of new map")
            #    self.replan()  # new map, need to replan
    
    def snap_to_grid(self, x):
        return (
            self.plan_resolution * round(x[0] / self.plan_resolution),
            self.plan_resolution * round(x[1] / self.plan_resolution),
        )

    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """

        return abs(x - self.x) < self.params.pos_eps and \
               abs(y - self.y) < self.params.pos_eps and \
               abs(theta - self.theta) < self.params.theta_eps

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return self.mode == Mode.STOP and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.params.stop_time)

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return self.mode == Mode.CROSS and \
               rospy.get_rostime() - self.cross_start > rospy.Duration.from_sec(self.params.crossing_time)

    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" if self.params.mapping else "/odom"
                translation, rotation = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x, self.y = translation[0], translation[1]
                self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current mode: %s", self.mode)
            self.prev_mode = self.mode
        
        global Mode
        ########## Code starts here ##########
        # TODO: Currently the state machine will just go to the pose without stopping
        #       at the stop sign.
        print(self.mode)
        #print(Mode)
        if self.mode == Mode.EXPLORE:
            rate = rospy.Rate(100)
            while np.average(np.less(self.occupancy.probs,0)) >= 0.05:
                x_init = (self.x, self.y)
                front = frontier(
                    x_init,
                    self.occupancy
                )
                found, centroid = front.get_frontier_closest()
                if found:
                    self.x_g, self.y_g = centroid
                else:
                    print("Frontier Not Found")
                    break
                rate = rospy.Rate(3)
                self.nav_to_pose()
                rate.sleep()
            print("EXIT")
            print()
            print()
            print()
            # WP1 = (1.944, 0.366, 3.11)
            # WP2 = (2.296, 1.618, 1.577)
            # self.x_g, self.y_g, self.theta_g = WP1
            # while not self.close_to(self.x_g, self.y_g, self.theta_g):
            #         self.nav_to_pose()
            #         rate.sleep()
                    
            # self.x_g, self.y_g, self.theta_g = WP2
            # while not self.close_to(self.x_g, self.y_g, self.theta_g):
            #     self.nav_to_pose()
            #     rate.sleep()

            self.x_g, self.y_g, self.theta_g = 3.15, 1.6, 0
            print("going home!")
            while not self.close_to(self.x_g, self.y_g, self.theta_g):
                self.nav_to_pose()
                #print("pos eps: ", self.params.pos_eps)
                rate.sleep()
            self.mode = Mode.USER_INP
            print("Switching mode to user input rescue time!")
        if self.mode == Mode.IDLE:
            # Send zero velocity
            self.stay_idle()
            if len(self.animal_list) >= NUM_ANIMALS:
                # Enough animals have been seen
                self.mode = MODE.USER_INP

        elif self.mode == Mode.POSE:
            # Moving towards a desired pose
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # At a stop sign
            self.nav_to_pose()

        elif self.mode == Mode.CROSS:
            # Crossing an intersection
            self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        elif self.mode == Mode.USER_INP:
            print("Here are the animals we have identified:")
            rate = rospy.Rate(10)
            for animal in self.animal_list:
                print(animal, self.animal_list[animal])
            print("Please run ./request_publisher.py with the names of the animals to rescue.")

            while(len(self.list_to_rescue) < 1):
                rate.sleep()
            # self.list_to_rescue = set(["cat", "dog", "elephant"])
            # self.animal_list = {"cat": ((0.9, 2.1), 0), "dog": ((1.1, 2.1), 0), "elephant": ((1.7, 1), 0)}
            for animal_name in self.list_to_rescue:
                if animal_name in self.animal_list: 
                    if self.animal_list[animal_name] is not None:
                        curr_animal = self.animal_list[animal_name]
                        print("Going to:", curr_animal)
                        self.x_g, self.y_g, self.theta_g = curr_animal[0][0], curr_animal[0][1], curr_animal[1]
                        while not self.close_to(self.x_g, self.y_g, self.theta_g):
                            self.nav_to_pose()
                            rate.sleep()
                        #rate2 = rospy.Rate(1)
                        #rate2.sleep(1)
                        #rate2.sleep(1)
                        #rate2.sleep(1)
                        print("SLEEPING")
                        rate2 = rospy.Rate(0.2)
                        rate2.sleep()
                        print("Visited: ", curr_animal)
                    else:
                        print("Did not find ", animal_name, " in exploration")
            self.mode = Mode.FINAL_RETURN

        elif self.mode == Mode.FINAL_RETURN:
            rate = rospy.Rate(10)
            self.x_g, self.y_g, self.theta_g = 3.15, 1.6, 0
            print("going home!")
            self.list_to_rescue = set()
            while not self.close_to(self.x_g, self.y_g, self.theta_g):
                self.nav_to_pose()
                #print("pos eps: ", self.params.pos_eps)
                rate.sleep()
            self.mode = Mode.DONE

        elif self.mode == Mode.DONE:
            self.stay_idle()

            # spaced_input = input()
            # animal_inds = spaced_input.split(' ')
            # for ind in animal_inds:
            #     curr_animal = animal_names[int(ind)]
            #     print("Going to:", curr_animal)
            #     self.x_g, self.y_g, self.theta_g = self.animal_list[curr_animal][0][0], self.animal_list[curr_animal][0][1], self.animal_list[curr_animal][1]
            #     self.nav_to_pose()
            #     print("Visited:", curr_animal)

        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))

        ############ Code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    sup = Supervisor()
    input("Press enter to begin!")
    sup.run()
