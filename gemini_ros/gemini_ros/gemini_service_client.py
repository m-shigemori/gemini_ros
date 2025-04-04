import sys
import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiRequest

class GeminiServiceClient(Node):
    def __init__(self):
        super().__init__('gemini_service_client')
        self.cli = self.create_client(GeminiRequest, 'gemini_request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GeminiRequest.Request()

    def send_request(self, input_str):
        self.req.input = input_str
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    gemini_service_client = GeminiServiceClient()
    response = gemini_service_client.send_request(sys.argv[1])
    gemini_service_client.get_logger().info(
        'Result of add_two_ints: for %s: %s' %
        (sys.argv[1], response.output))
    gemini_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()