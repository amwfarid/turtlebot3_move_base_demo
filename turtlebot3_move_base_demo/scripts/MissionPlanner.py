#!//bin/python

import random
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty
from ObstacleController import ObstacleController

class MissionPlanner:

    def __init__(self):

        # Mission points to randomly interchange. They are hard-coded
        # to represent points in the standard turtlebot3 world.

        self.mission_points = [(1.75, 1.0), (2.0, 0.0), (1.75, -1.0),
                               (1.0, -1.75), (0.0, -2.0), (-1.0, -1.75),
                               (-1.75, -1.0), (-2.0, 0.0), (-1.75, 1.0),
                               (-1.0, 1.75), (0.0, 2.0), (1.0, 1.75),
                               (-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5),
                               (0.5, -0.5)]

        self.goal_coords = ()
        self.prev_goal_coords = ()

        self.pose_init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        while self.pose_init_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to connect")
            rospy.sleep(1)

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        while self.goal_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to connect")
            rospy.sleep(1)

        self.mission_status_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.__mission_status_monitor)

        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_cost_maps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # The initial pose of the robot is correctly set in Rviz to match
        # the initial pose of the robot in Gazebo. This is done to avoid
        # making edits to files outside of this package.

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = -2.0
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0

        self.pose_init_pub.publish(msg)

        rospy.sleep(2)


    def start_random_mission(self):

        # Clearing the cost map on every mission helps remove obstacles
        # from previous memory

        self.clear_cost_maps()
        rospy.sleep(1)

        if not self.prev_goal_coords:
            self.prev_goal_coords = (-2.0, -0.5)  # Default starting point
        else:
            self.prev_goal_coords = self.goal_coords

        self.goal_coords = self.mission_points[random.randrange(len(self.mission_points))]

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.goal_coords[0]
        msg.pose.position.y = self.goal_coords[1]
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

        rospy.loginfo('[mission_planner_demo] Mission started towards: '+str(self.goal_coords))


    def __mission_status_monitor(self, data):
        if data.status.status == 3:
            rospy.loginfo('[mission_planner_demo] Goal reached. Creating new mission.')
            self.start_random_mission()

        # Although somewhat rare, the path planner can fail. A recovery mechanism
        # is attempted.
        elif data.status.status == 4:
            rospy.loginfo('[mission_planner_demo] Goal failed. ' + str(self.prev_goal_coords) +
                          ' -> ' +str(self.goal_coords))
            
            # For simplicity, a new mission will be started.
            self.start_random_mission()

if __name__ == '__main__':
    rospy.init_node('mission_planner_demo', anonymous=False)

    m = MissionPlanner()

    rospy.loginfo('[mission_planner] Started')

    m.start_random_mission()

    try:
        while not rospy.is_shutdown():
            pass

    except Exception as e:
        print(e)

    rospy.loginfo("[mission_planner_demo] Stopped")
