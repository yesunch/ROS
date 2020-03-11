#!/usr/bin/python2.7
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
import visualization_msgs
import copy

from move import Move
from state import State
from robot import Robot
from map import Map

class TrajectoryPlanner:
    def __init__(self):
        self.map = None
        self.start = State(0, 0)
        self.goal = State(0.5, 0.5)

        self.moves = [Move(0.5, 0),  # forward
                      Move(-0.5, 0),  # back
                      Move(0, 1.5708),  # turn left 90
                      Move(0, -1.5708)] # turn right 90
        self.robot = Robot(0.5, 0.5)
        self.is_working = False # Replace with mutex after all

        self.map_subscriber = rospy.Subscriber("map", OccupancyGrid, self.new_map_callback)
        #self.start_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.new_start_callback)
        #self.goal_subscriber = rospy.Subscriber("goal", PoseStamped, self.new_goal_callback)

        self.path_publisher = rospy.Publisher("trajectory", MarkerArray, queue_size=1)
        self.pose_publisher = rospy.Publisher("debug_pose", PoseStamped, queue_size=1)

        # what will be there. A module goes into variable. Isn't it too much memory consumption. Maybe I should assign function replan() to this variable?
        #self.planner = planners.astar.replan

    def ready_to_plan(self):
        return self.map is not None and self.start is not None and self.goal is not None

    def new_goal_callback(self, goal_pose):
        if not self.is_working:
            self.is_working = True
            new_goal = State.from_pose(goal_pose.pose)
            if self.map is not None and self.map.is_allowed(new_goal, self.robot):
                self.goal = new_goal
                rospy.loginfo("New goal was set")
                if self.ready_to_plan():
                    self.replan()
            else:
                rospy.logwarn("New goal is bad or no map available")

            self.is_working = False

    def new_start_callback(self, start_pose):
        if not self.is_working:
            self.is_working = True
            new_start = State.from_pose(start_pose.pose.pose)
            if self.map is not None and self.map.is_allowed(new_start, self.robot):
                self.start = new_start
                rospy.loginfo("New start was set")
                if self.ready_to_plan():
                    self.replan()
            else:
                rospy.logwarn("New start is bad or no map available")
            self.is_working = False

    def new_map_callback(self, grid_map):
        if not self.is_working:
            self.is_working = True
            self.map = Map(grid_map)
            rospy.loginfo("New map was set")
            if self.ready_to_plan():
                self.replan()
            self.is_working = False

    def replan(self):
        rospy.loginfo("Planning was started")
        rospy.loginfo("start from:"+str(self.start.x)+" "+str(self.start.y))
        rospy.loginfo("go to:"+str(self.goal.x)+" "+str(self.goal.y))
        final_state = self.replan_width(self.map, self.moves, self.robot, self.start, self.goal)
        import pdb
        #pdb.set_trace()
        if final_state is None:
            rospy.loginfo("No path found")
        else:
            # Restore and publish path
            rospy.loginfo("Restoring path from final state...")
            path = self.restore_path(final_state)
            self.path_publisher.publish(path)
            rospy.loginfo("Planning was finished...")
            rospy.loginfo(path)

    def replan_width(self, map, moves, robot, start, goal):
        import pdb
        #pdb.set_trace()
        robot_dimension = max(robot.width, robot.height)
        final_state = None
        openned = [start]
        closed = []
        while (openned and final_state is None):
            item = openned.pop()
            closed.append(item)
            for move in moves:
                new_item = item.try_apply(map, move, robot)
                #import pdb
                #pdb.set_trace()
                if (new_item is not None):
                    if (new_item.dist_to(goal) < robot_dimension):
                        final_state = new_item
                        break
                    if not any(other_item.is_same_as(new_item) for other_item in openned):
                        openned.insert(0, new_item)
        return final_state


    def restore_path(self, final_state):
        current_state = copy.copy(final_state)
        path = MarkerArray()
        pose_id = 0
        while True:
            pose_marker = current_state.to_marker(self.robot)
            pose_marker.id = pose_id
            path.markers.append(pose_marker)

            current_state = current_state.parent
            pose_id += 1

            if current_state is None:
                break
        return path



def main():
    rospy.init_node("trajectory_planner")
    planner = TrajectoryPlanner()
    rospy.spin()

main()
