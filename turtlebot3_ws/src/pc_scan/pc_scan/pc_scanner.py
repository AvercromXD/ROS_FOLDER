import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


class PCScanner(Node):
    def __init__(self):
        super().__init__("pc_saver")
        self.pc_sub = self.create_subscription(PointCloud2, '/camera_2/points', self.callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/virtual_cam/points', 10)
        self.srv = self.create_service(Trigger, 'pc_scan', self.pc_scan_callback)
        self.points = None

    def pc_scan_callback(self, req, res):
        if self.points == None:
            self.get_logger().error("No Pointcloud published yet")
            res.success = False
            res.message = "No Pointcloud published yet"
            return res
        self.pc_pub.publish(self.points)
        res.success = True
        res.message = "Success"
        return res


    def callback(self, msg):
        self.points = msg

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PCScanner()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
