import rclpy
from yt_recorder.Recorder import Recorder


def main():
    rclpy.init()

    node = Recorder()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
