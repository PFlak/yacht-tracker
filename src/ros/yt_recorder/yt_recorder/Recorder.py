import os
import json

from rclpy.node import Node

from gps_msgs.msg import GPSFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Recorder(Node):

    def __init__(self):
        Node.__init__(self=self, node_name='yt_recorder_node')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('default_path_dir', '/home/dev/ros2_ws/src/ros/yt_recorder/paths')
        self.declare_parameter('path_filename', 'path_sniardwy.json')
        self.declare_parameter('sim_refresh_rate', 30)
        self.declare_parameter('id', 'sail')

        self.sim_mode = self.get_parameter('sim_mode').value
        self.default_path_dir = self.get_parameter('default_path_dir').value
        self.path_filename = self.get_parameter('path_filename').value
        self.sim_refresh_rate = self.get_parameter('sim_refresh_rate').value
        self.id = self.get_parameter('id').value

        self.sim_dt = 1.0 / self.sim_refresh_rate

        self.gps_pub = self.create_publisher(GPSFix, f'yt_recorder/{self.id}/gps', 0)

        if self.sim_mode:

            self.path_pub = self.create_publisher(Path, f'yt_recorder/{self.id}/path', 10)

            self.waypoints, self.sim_waypoint_time_difference = self.get_waypoints_from_file()
            self.publish_path()

            self.intp_traj = self.interpolate_trajectory()

            self.sim_timer = self.create_timer(self.sim_dt, self.cb_main_timer)

    def cb_main_timer(self):
        now = self.get_clock().now()
        try:
            x, y = next(self.intp_traj)

        except Exception:
            x, y = self.waypoints[-1]

        finally:
            gps = GPSFix()
            gps.header.frame_id = 'map'
            gps.header.stamp = now.to_msg()
            gps.longitude = x
            gps.latitude = y

            self.gps_pub.publish(gps)

    def interpolate_trajectory(self):
        for i in range(len(self.waypoints) - 1):
            x0, y0 = self.waypoints[i]
            x1, y1 = self.waypoints[i + 1]
            segment_time = self.sim_waypoint_time_difference
            steps = int(segment_time / self.sim_dt)

            for s in range(steps):
                t = s * self.sim_dt / segment_time
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                yield x, y

        yield self.waypoints[-1]

    def get_waypoints_from_file(self) -> tuple[list[tuple[float, float]], float]:

        file_path = os.path.join(self.default_path_dir, self.path_filename)

        data = None

        with open(file_path, 'r') as file:
            data = json.load(file)

        waypoints = [(p['x'], p['y']) for p in data['points']]

        return waypoints, data['time']

    def publish_path(self) -> None:
        now = self.get_clock().now()

        poses = []

        for waypoint in self.waypoints:
            x, y = waypoint
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now.to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y

            poses.append(pose)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = now.to_msg()
        path.poses = poses

        self.path_pub.publish(path)
