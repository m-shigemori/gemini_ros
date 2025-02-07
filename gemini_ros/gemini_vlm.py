#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from google import genai
import PIL.Image


class GeminiVLMNode(Node):
    def __init__(self):
        super().__init__('gemini_vlm_node')

        api_key = os.environ.get('GEMINI_API_KEY')
        self.client = genai.Client(api_key=api_key)

        self.subscription = self.create_subscription(
            String,
            '/gemini_vlm_request',
            self.handle_vlm_request,
            10
        )
        self.publisher = self.create_publisher(
            String, '/gemini_vlm_response', 10)

    def handle_vlm_request(self, msg):
        if msg.data == 'start':
            self.capture_and_process_image()

    def capture_and_process_image(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()

        cv2.imwrite("captured_image.jpg", frame)
        image = PIL.Image.open("captured_image.jpg")

        response = self.client.models.generate_content(
            model="gemini-2.0-flash",
            contents=["この画像を説明してほしいです、日本語で", image],
        )

        cleaned_response = response.text.replace(
            '*', '').replace('\n', ' ').strip()
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
