import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiRequest

class GeminiServiceServer(Node):
    def __init__(self):
        super().__init__('gemini_service_server')
        self.srv = self.create_service(GeminiRequest, 'gemini_request', self.gemini_request_callback)

    def gemini_request_callback(self, request, response):
        self.get_logger().info('Incoming request: %s' % (request.input))
        response.output = 'Received: %s' % (request.input)
        return response

def main(args=None):
    rclpy.init(args=args)
    gemini_service_server = GeminiServiceServer()
    rclpy.spin(gemini_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()