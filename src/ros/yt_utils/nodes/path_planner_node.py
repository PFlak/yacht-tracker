import rclpy
from yt_utils.PathPlanner import PathPlanner


def main():
    rclpy.init()

    node = PathPlanner()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
