#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from path_drive.msg import PathPoint, FullPath, PathDriveAction, PathDriveActionFeedback, PathDriveActionResult
from nav_msgs.msg import OccupancyGrid


class PathDriveServer:

    def __init__(self, name):
        self.is_navigating = False
        self.waypoints = []
        self.feedback = PathDriveActionFeedback()
        self.result = PathDriveActionResult()

        print("--- publisher ---")
        # --- Publishers ---
        self.pub_goal = rospy.Publisher('move_to_goal/goal', Pose, queue_size=1)

        print("--- subscriber ---")
        # --- Subscribers ---
        self.sub_goal_reached = rospy.Subscriber('move_to_goal/reached', Bool, self._goal_reached_callback)
        
        self._setup()

        print("--- start server ---")
        # --- Server ---
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PathDriveAction, execute_cb=self.execute_callback, auto_start = False) 
        self._as.start()
        print "--- server ready ---"

    def _setup(self):
        map = rospy.wait_for_message('map', OccupancyGrid)
        self.map_info = map.info

    def execute_callback(self, data):
        success = True
        #ate = rospy.Rate(1)
        self.waypoints = data.waypoints.fullpath

        self.waypointsAvailable = True

        while not rospy.is_shutdown():
            if self.is_navigating == False:
                self._navigate()
            if self.waypointsAvailable == False:
                success = True
                break
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                break
            self.feedback.feedback.driving.data = self.is_navigating

            self._as.publish_feedback(self.feedback.feedback)
            #rate.sleep()
 
        if success:
            self._as.set_succeeded(self.result.result)

    def _navigate(self):
        # get waypoint and start moving towards it
        # when success the process next
        self.is_navigating = True

        if(len(self.waypoints) > 0):
            point = self.waypoints.pop(0)
            x = point.path_x
            y = point.path_y

            print(self.waypoints)

            # -- move to goal --
            self._move(x, y)

        else:
            self.result.result.reached_last_goal.data = True
            self.is_navigating = False
            self.waypointsAvailable = False

    def _move(self, x, y):
        """
        Moves the rob2t to a place defined by coordinates x and y.
        """
        print('Navigate to: ' + str(x) + ' | ' + str(y))
        goal = Pose()

        target_x = (x * self.map_info.resolution) + self.map_info.origin.position.x
        target_y = (y * self.map_info.resolution) + self.map_info.origin.position.y

        goal.position.x = target_x
        goal.position.y = target_y
        goal.orientation.w = 1

        self.pub_goal.publish(goal)
    
    
    def _goal_reached_callback(self, reached):
        print reached
        if reached.data == True:
            print "reached"
            self._navigate()
        else:
            print "nope"
            self.waypoints = []
            self.result.result.reached_last_goal.data = False
            self.is_navigating = False


if __name__ == '__main__':
    rospy.init_node('path_drive_server')
    server = PathDriveServer(rospy.get_name())
    rospy.spin()