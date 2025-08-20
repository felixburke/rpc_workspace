import rclpy
from rclpy.node import Node
from tams_lasertag_client.srv import SubmitHit
from sensor_msgs.msg import Image, CameraInfo

class GameClientNode(Node):

    def __init__(self):
        super().__init__('game_client_node')

        self.image_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            1
        )
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/oakd/rgb/preview/camera_info',
            self.camera_info_callback,
            1
        )

        self.camera_info = None

        self.client = self.create_client(SubmitHit, 'submit_hit')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Game client node initialized and ready to process images.')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        self.get_logger().info('Received image')

        if self.camera_info is None:
            self.get_logger().warn('Camera info not available yet, skipping image processing')
            return

        # Here you would typically process the image and call the service
        request = SubmitHit.Request()
        request.image = msg
        request.camera_info = self.camera_info  # Use the stored camera info

        self.client.call_async(request).add_done_callback(self.hit_result_callback)

    def hit_result_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Hit result: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

if __name__ == '__main__':
    rclpy.init()
    node = GameClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
