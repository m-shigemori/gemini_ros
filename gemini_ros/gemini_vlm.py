#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import PIL.Image
from google import genai


class GeminiVLMNode(Node):
    def __init__(self):
        super().__init__('gemini_vlm_node')

        api_key = os.environ.get('GEMINI_API_KEY')
        self.client = genai.Client(api_key=api_key)

        self.file_name = 'input.jpg'

        self.subscription = self.create_subscription(
            String, '/gemini_vlm_request', self.handle_vlm_request, 10
        )

        self.publisher = self.create_publisher(
            String, '/gemini_vlm_response', 10
        )

        self.get_logger().info("Gemini VLM が起動しました")

    def handle_vlm_request(self, msg):
        self.capture_image()
        self.publish_response(msg)

    def capture_image(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        cv2.imwrite(self.file_name, frame)

    def publish_response(self, msg):
        image = PIL.Image.open(self.file_name)

        response = self.client.models.generate_content(
            model = "gemini-2.0-flash",
            contents = [msg.data, image],
            config = {
                # "max_output_tokens": 30,
            }
        )

        cleaned_response = response.text.replace('*', '').replace('\n', ' ').strip()
        response_msg = String()
        response_msg.data = cleaned_response
        self.publisher.publish(response_msg)
        self.get_logger().info(cleaned_response)

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
