#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiRequest
import cv2
import PIL.Image
from google import genai


class GeminiVLMNode(Node):
    def __init__(self):
        super().__init__('gemini_vlm_node')

        self.client = genai.Client(api_key=os.environ.get('GEMINI_API_KEY'))

        self.file_name = 'input.jpg'

        self.srv = self.create_service(
            GeminiRequest, '/gemini_vlm_service', self.handle_vlm_request
        )

        self.get_logger().info("Gemini VLM サービスが起動しました")

    def handle_vlm_request(self, request, response):
        self.capture_image()
        self.process_image_and_response(request, response)
        return response

    def capture_image(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        cv2.imwrite(self.file_name, frame)

    def process_image_and_response(self, request, response):
        image = PIL.Image.open(self.file_name)

        gemini_response = self.client.models.generate_content(
            model="gemini-2.0-flash",
            contents=[request.input, image],
            config={
                # "max_output_tokens": 30,
            }
        )

        cleaned_response = gemini_response.text.replace('*', '').replace('\n', ' ').strip()
        response.output = cleaned_response
        self.get_logger().info(f"Response: {cleaned_response}")

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