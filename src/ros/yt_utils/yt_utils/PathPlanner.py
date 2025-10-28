import os
import json

from rclpy.node import Node

from marti_nav_msgs.srv import PlanRoute
from std_srvs.srv import Trigger

from geometry_msgs.msg import Pose


class PathPlanner(Node):

    def __init__(self):
        Node.__init__(self=self, node_name='yt_path_planner')

        self.declare_parameter('default_exec_time', 60)
        self.declare_parameter('default_path', '/home/dev/ros2_ws/src/ros/yt_recorder/paths')
        self.declare_parameter('filename', 'path')

        self.default_exec_time = self.get_parameter('default_exec_time').value

        self.waypoints: list[Pose] = []

        self.create_service(PlanRoute, 'yt_path_planner/aggregate', self.cb_aggregate)
        self.create_service(Trigger, 'yt_path_planner/save', self.cb_save)

    @property
    def default_path(self) -> str:
        return self.get_parameter('default_path').value

    @property
    def filename(self) -> str:
        param_filename: str = self.get_parameter('filename').value
        param_filename = param_filename.split('.json')[0]

        files = os.listdir(self.default_path)
        files.sort()

        i = 1

        filename = param_filename

        if len(files):
            for file in files:
                if param_filename == file:
                    filename = param_filename + i
                    i += 1

        filename += '.json'
        filename = os.path.join(self.default_path, filename)

        return filename

    def cb_aggregate(self, request: PlanRoute.Request, response: PlanRoute.Response):
        self.waypoints = request.waypoints

        self.get_logger().info("Waypoints updated")

        response.success = True
        response.message = 'ok'
        return response

    def cb_save(self, request: Trigger.Request, response: Trigger.Response):
        try:
            points = []

            for point in self.waypoints:
                point_dict = {
                    'x': point.position.x,
                    'y': point.position.y
                }

                points.append(point_dict)

            points_dict = {
                'points': points,
                'time': self.default_exec_time
            }

            filename = self.filename

            with open(filename, 'w') as file:
                json.dump(points_dict, file, indent=4)

            response.success = True
            response.message = f'Path saved to: {filename}'

        except Exception as e:
            response.success = False
            response.message = f'{e}'
        finally:
            return response
