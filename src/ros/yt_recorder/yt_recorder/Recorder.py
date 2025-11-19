import os
import json
from datetime import datetime
import requests as re

from rclpy.node import Node
from rclpy.timer import Timer

from gps_msgs.msg import GPSFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from yt_recorder.models.boat_position import BoatPosition
from yt_recorder.models.boat_route import BoatRoute
from yt_recorder.models.position_update import PositionUpdate
from yt_recorder.models.recorder_synchronization import RecorderSynchronization


class Recorder(Node):

    def __init__(self):
        Node.__init__(self=self, node_name='yt_recorder_node')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('default_path_dir', '/home/dev/ros2_ws/src/ros/yt_recorder/paths')
        self.declare_parameter('path_filename', 'path_sniardwy.json')
        self.declare_parameter('sim_refresh_rate', 30)
        self.declare_parameter('id', 0)

        self.declare_parameter('default_sync_time', 5.0)
        self.declare_parameter('default_position_snapshot_time', 2.0)

        self.declare_parameter('server_url', 'http://127.0.0.1:8000')

        self.sim_mode = self.get_parameter('sim_mode').value
        self.default_path_dir = self.get_parameter('default_path_dir').value
        self.path_filename = self.get_parameter('path_filename').value
        self.sim_refresh_rate = self.get_parameter('sim_refresh_rate').value
        self.id = self.get_parameter('id').value
        self.server_url = self.get_parameter('server_url').value

        self.declare_parameter('api_sync_path', f'recorder/{self.id}/sync')
        self.declare_parameter('api_update_path', f'recorder/{self.id}/position_update')

        self.api_sync_path = self.get_parameter('api_sync_path').value
        self.api_sync_path = self.server_url + '/' + self.api_sync_path
        self.api_update_path = self.get_parameter('api_update_path').value
        self.api_update_path = self.server_url + '/' + self.api_update_path

        self.longitude = 0.0
        self.latitude = 0.0

        self.boat_route: BoatRoute = BoatRoute(boat_id=self.id)

        self.sim_dt = 1.0 / self.sim_refresh_rate

        self.gps_pub = self.create_publisher(GPSFix, f'yt_recorder/sail_{str(self.id)}/gps', 0)

        if self.sim_mode:

            self.path_pub = self.create_publisher(Path, f'yt_recorder/sail_{str(self.id)}/path', 10)

            self.waypoints, self.sim_waypoint_time_difference = self.get_waypoints_from_file()
            self.publish_path()

            self.intp_traj = self.interpolate_trajectory()

            self.sim_timer = self.create_timer(self.sim_dt, self.cb_sim_timer)

        else:
            # TODO: Implement embedded integration
            ...

        self.sync_time = self.get_parameter('default_sync_time').value
        self.sync_timer = self.create_timer(self.sync_time, self.cb_sync_timer)

        self.position_snapshot_time = self.get_parameter('default_position_snapshot_time').value
        self.position_snapshot_timer = self.create_timer(self.position_snapshot_time, self.cb_position_snapshot_timer)

        self.update_time = 0.0
        self.update_timer = None

    def cb_sim_timer(self):
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

            self.longitude = x
            self.latitude = y

            self.gps_pub.publish(gps)

    def cb_sync_timer(self):
        now = datetime.now().timestamp()

        recorder_synchronization = RecorderSynchronization(boat_id=self.id,
                                                           status='ok',
                                                           timestamp=now,
                                                           sync_period=self.sync_time,
                                                           position_update_period=self.update_time,
                                                           position_stamp_period=self.position_snapshot_time,
                                                           boat_route=self.boat_route if len(self.boat_route.positions) else None)

        body = recorder_synchronization.model_dump_json()

        try:
            response = re.post(self.api_sync_path, data=body)
            if response.status_code != 200:
                raise Exception()

            self.boat_route = BoatRoute(boat_id=self.id)

            data: dict = response.json()
            self.get_logger().info(f"{data}")

            data = RecorderSynchronization.model_validate(data)

            self.sync_timer, self.sync_time = self.adjust_custom_timer(self.sync_timer, self.cb_sync_timer, float(data.sync_period), self.sync_time)
            self.update_timer, self.update_time = self.adjust_custom_timer(self.update_timer, self.cb_update_timer, float(data.position_update_period), self.update_time)
            self.position_snapshot_timer, self.position_snapshot_time = self.adjust_custom_timer(self.position_snapshot_timer, self.cb_position_snapshot_timer, float(data.position_stamp_period), self.position_snapshot_time)
            self.display_text_message(data.direct_message)

        except Exception as e:
            self.get_logger().error(f"{e}")

    def cb_position_snapshot_timer(self):
        now = datetime.now().timestamp()

        boat_position = BoatPosition(id=0,
                                     recorder_id=self.id,
                                     timestamp=now,
                                     latitude=self.latitude,
                                     longitude=self.longitude)

        self.boat_route.positions.append(boat_position)

        self.get_logger().info('Position snapshot taken')

    def cb_update_timer(self):
        try:
            now = datetime.now().timestamp()

            boat_position = BoatPosition(id=0,
                                         recorder_id=self.id,
                                         timestamp=now,
                                         latitude=self.latitude,
                                         longitude=self.longitude)

            position_update = PositionUpdate(boat_id=self.id,
                                             timestamp=now,
                                             boat_position=boat_position,
                                             status='ok')

            body = position_update.model_dump_json()

            response = re.post(self.api_update_path, data=body)

            self.get_logger().info(f"{response.json()}")
        except Exception as e:
            self.get_logger().error(f"{e}")

    def adjust_custom_timer(self, timer: Timer, cb_timer, time: float, old_time: float):
        if time == old_time:
            return timer, time

        self.destroy_custom_timer(timer)
        new_timer = self.create_custom_timer(cb_timer, time)
        return new_timer, time

    def create_custom_timer(self, cb_timer, time: float):

        new_timer = self.create_timer(time, cb_timer)
        self.get_logger().info(f'New Timer created with time period: {time}')
        return new_timer

    def destroy_custom_timer(self, timer: Timer):
        if not timer:
            self.get_logger().warning('Timer does not exist')
            return

        destroy_status = self.destroy_timer(timer)

        if not destroy_status:
            self.get_logger().error('Could not destroy timer')

        return destroy_status

    def display_text_message(self, text: str):
        self.get_logger().info(f'Displaying text: {text}')

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
