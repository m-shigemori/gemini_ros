#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gemini_interface.srv import GeminiRequest
import cv2
import PIL.Image
from google import genai
from cv_bridge import CvBridge


class GeminiVLMNode(Node):
    def __init__(self):
        super().__init__('gemini_vlm_node')

        self.client = genai.Client(api_key=os.environ.get('GEMINI_API_KEY'))

        workspace_directory = os.path.expanduser('~/colcon_ws/src/gemini_ros')
        self.media_data_path = os.path.join(workspace_directory, 'media_data')

        if not os.path.exists(self.media_data_path):
            os.makedirs(self.media_data_path)

        self.file_counter = 1
        self.image_data = None

        self.subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )

        self.srv = self.create_service(
            GeminiRequest, '/gemini_vlm_service', self.handle_vlm_request
        )

        self.get_logger().info("Gemini VLM サービスが起動しました")

    def image_callback(self, msg):
        self.image_data = msg

    def handle_vlm_request(self, request, response):
        self.capture_image()
        self.process_image_and_response(request, response)
        return response

    def capture_image(self):
        file_name = os.path.join(self.media_data_path, f'input_{self.file_counter}.jpg')
        cv_image = CvBridge().imgmsg_to_cv2(self.image_data)
        cv2.imwrite(file_name, cv_image)

    def process_image_and_response(self, request, response):
        file_name = os.path.join(self.media_data_path, f'input_{self.file_counter}.jpg')
        image = PIL.Image.open(file_name)

        gemini_response = self.client.models.generate_content(
            model="gemini-2.0-flash",
            contents=[request.input, image],
            config={
                # "max_output_tokens": 30,
            }
        )

        response.output = gemini_response.text.replace('*', '').replace('\n', ' ').strip()

        self.file_counter += 1


def main():
    rclpy.init()
    node = GeminiVLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
