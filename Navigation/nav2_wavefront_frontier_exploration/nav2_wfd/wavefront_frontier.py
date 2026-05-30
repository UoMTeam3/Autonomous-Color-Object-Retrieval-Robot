#! /usr/bin/env python3

import math
import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 5

BLACKLIST_RADIUS = 0.4          # meters
GOAL_REACHED_RADIUS = 0.35      # meters
PROGRESS_TIMEOUT = 20.0         # seconds
MIN_PROGRESS_DELTA = 0.10       # meters


class OccupancyGrid2d:
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map_msg):
        self.map = map_msg

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return (wx, wy)

    def worldToMap(self, wx, wy):
        if wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y:
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

        if my >= self.map.info.height or mx >= self.map.info.width:
            raise Exception("World coordinates out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx


class FrontierPoint:
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y


class FrontierCache:
    def __init__(self):
        self.cache = {}

    def getPoint(self, x, y):
        idx = self.__cantorHash(x, y)
        if idx not in self.cache:
            self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def __cantorHash(self, x, y):
        s = x + y
        return (s * (s + 1)) // 2 + y

    def clear(self):
        self.cache.clear()


class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8


def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x / length, sum_y / length


def getNeighbors(point, costmap, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if 0 <= x < costmap.getSizeX() and 0 <= y < costmap.getSizeY():
                if x == point.mapX and y == point.mapY:
                    continue
                neighbors.append(fCache.getPoint(x, y))

    return neighbors


def isFrontierPoint(point, costmap, fCache):
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False

    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:
            return False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree


def findFree(mx, my, costmap):
    fCache = FrontierCache()
    start = fCache.getPoint(mx, my)
    start.classification |= PointClassification.MapClosed.value

    bfs = [start]

    while bfs:
        loc = bfs.pop(0)

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if (n.classification & PointClassification.MapClosed.value) == 0:
                n.classification |= PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)


def getFrontier(pose, costmap):
    fCache = FrontierCache()
    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
    freePoint = findFree(mx, my, costmap)

    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value

    mapPointQueue = [start]
    frontiers = []

    while mapPointQueue:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value:
            continue

        if isFrontierPoint(p, costmap, fCache):
            p.classification |= PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while frontierQueue:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value):
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if (w.classification & (
                            PointClassification.FrontierOpen.value |
                            PointClassification.FrontierClosed.value |
                            PointClassification.MapClosed.value
                        )) == 0:
                            w.classification |= PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification |= PointClassification.FrontierClosed.value

            newFrontierCoords = []
            for x in newFrontier:
                x.classification |= PointClassification.MapClosed.value
                newFrontierCoords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(centroid(newFrontierCoords))

        for v in getNeighbors(p, costmap, fCache):
            if (v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value)) == 0:
                if any(
                    costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value
                    for x in getNeighbors(v, costmap, fCache)
                ):
                    v.classification |= PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification |= PointClassification.MapClosed.value

    return frontiers


class WaypointFollowerTest(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_tester')

        self.exploration_done_pub = self.create_publisher(
            Bool, 
            '/exploration_done', 
            1)

        self.explore_enabled = True   # default ON

        self.create_subscription(
            Bool,
            '/explore_enable',
            self.explore_enable_callback,
            1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.costmapSub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancyGridCallback,
            pose_qos
        )

        self.pose_timer = self.create_timer(0.5, self.update_pose)
        self.explore_timer = self.create_timer(1.0, self.exploration_loop)
        self.progress_timer = self.create_timer(1.0, self.check_progress)

        self.currentPose = None
        self.costmap = None
        self.initial_pose_received = False

        self.waypoints = []
        self.goal_handle = None
        self.processing_goal = False
        self.exploration_done = False

        self.blacklist = []
        self.current_goal_target = None
        self.goal_start_time = None
        self.goal_start_distance = None
        self.best_distance_so_far = None

        self.get_logger().info('Running frontier exploration')

    def explore_enable_callback(self, msg):

        prev_state = self.explore_enabled
        self.explore_enabled = msg.data

        if not self.explore_enabled:
            self.warn_msg("Exploration DISABLED → stopping robot")

            # Cancel any active goal immediately
            self.cancel_current_goal()

        elif not prev_state and self.explore_enabled:
            self.info_msg("Exploration ENABLED → resuming")

    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(msg)

    def update_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation

            self.currentPose = pose
            self.initial_pose_received = True

        except Exception:
            self.get_logger().warn("Waiting for TF map -> base_link")

    def distance_to(self, point_xy):
        if self.currentPose is None:
            return float('inf')
        return math.hypot(
            point_xy[0] - self.currentPose.position.x,
            point_xy[1] - self.currentPose.position.y
        )

    def is_blacklisted(self, frontier):
        for bx, by in self.blacklist:
            if math.hypot(frontier[0] - bx, frontier[1] - by) < BLACKLIST_RADIUS:
                return True
        return False

    def blacklist_current_goal(self, reason="unknown"):
        if self.current_goal_target is not None:
            self.blacklist.append(self.current_goal_target)
            self.warn_msg(f"Blacklisting frontier {self.current_goal_target} ({reason})")

    def setWaypoints(self, waypoints):
        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def exploration_loop(self):

        if not self.explore_enabled:
            return

        if self.exploration_done or self.processing_goal:
            return

        if self.currentPose is None or self.costmap is None:
            self.info_msg("Waiting for pose or map...")
            return

        try:
            frontiers = getFrontier(self.currentPose, self.costmap)
        except Exception as e:
            self.warn_msg(f"Frontier extraction failed: {e}")
            return

        if not frontiers:
            self.info_msg("No more frontiers. Exploration complete.")
            self.exploration_done = True
            self.exploration_done_pub.publish(Bool(data=True))
            return

        valid_frontiers = [f for f in frontiers if not self.is_blacklisted(f)]

        if not valid_frontiers:
            self.warn_msg("All detected frontiers are blacklisted. Exploration complete.")
            self.exploration_done = True
            self.exploration_done_pub.publish(Bool(data=True))
            return

        closest_frontier = min(valid_frontiers, key=self.distance_to)

        if self.distance_to(closest_frontier) < GOAL_REACHED_RADIUS:
            self.blacklist.append(closest_frontier)
            self.warn_msg(f"Skipping too-close frontier {closest_frontier}")
            return

        self.send_goal_to_frontier(closest_frontier)

    def send_goal_to_frontier(self, frontier):

        if not self.explore_enabled:
            return
        self.setWaypoints([frontier])

        action_request = FollowWaypoints.Goal()
        action_request.poses = self.waypoints

        self.info_msg(f"Sending frontier goal: {frontier}")

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("Waiting for /follow_waypoints action server...")

        self.processing_goal = True
        self.current_goal_target = frontier
        self.goal_start_time = time.time()

        current_dist = self.distance_to(frontier)
        self.goal_start_distance = current_dist
        self.best_distance_so_far = current_dist

        send_goal_future = self.action_client.send_goal_async(action_request)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.error_msg(f"Goal service call failed: {e}")
            self.blacklist_current_goal("service failure")
            self.reset_goal_state()
            return

        if not self.goal_handle.accepted:
            self.error_msg("Goal rejected")
            self.blacklist_current_goal("goal rejected")
            self.reset_goal_state()
            return

        self.info_msg("Goal accepted")
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.info_msg("Goal succeeded")
            else:
                self.warn_msg(f"Goal failed with status {status}")
                self.blacklist_current_goal(f"status {status}")

        except Exception as e:
            self.error_msg(f"Result callback failed: {e}")
            self.blacklist_current_goal("result callback exception")

        self.reset_goal_state()

    def reset_goal_state(self):
        self.processing_goal = False
        self.goal_handle = None
        self.current_goal_target = None
        self.goal_start_time = None
        self.goal_start_distance = None
        self.best_distance_so_far = None

    def check_progress(self):

        if not self.explore_enabled:
            return
        
        if not self.processing_goal:
            return

        if self.currentPose is None or self.current_goal_target is None:
            return

        current_distance = self.distance_to(self.current_goal_target)

        if self.best_distance_so_far is None or current_distance < self.best_distance_so_far:
            self.best_distance_so_far = current_distance

        if current_distance < GOAL_REACHED_RADIUS:
            self.info_msg("Robot is very close to current frontier goal")
            return

        elapsed = time.time() - self.goal_start_time if self.goal_start_time is not None else 0.0
        progress = (self.goal_start_distance - self.best_distance_so_far) if self.goal_start_distance is not None else 0.0

        if elapsed > PROGRESS_TIMEOUT and progress < MIN_PROGRESS_DELTA:
            self.warn_msg(
                f"No meaningful progress for {elapsed:.1f}s "
                f"(progress: {progress:.2f} m). Cancelling goal."
            )
            self.blacklist_current_goal("progress timeout")
            self.cancel_current_goal()

    def cancel_current_goal(self):
        if self.goal_handle is None:
            self.reset_goal_state()
            return

        try:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        except Exception as e:
            self.error_msg(f"Failed to cancel goal: {e}")
            self.reset_goal_state()

    def cancel_done_callback(self, future):
        try:
            _ = future.result()
            self.warn_msg("Goal cancelled")
        except Exception as e:
            self.error_msg(f"Cancel callback failed: {e}")

        self.reset_goal_state()

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main(args=None):
    rclpy.init(args=args)

    node = WaypointFollowerTest()

    try:
        while not node.initial_pose_received:
            node.info_msg("Waiting for TF pose...")
            rclpy.spin_once(node, timeout_sec=1.0)

        while node.costmap is None:
            node.info_msg("Waiting for initial map...")
            rclpy.spin_once(node, timeout_sec=1.0)

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)



if __name__ == '__main__':
    main()