#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiRequest
from google import genai


class GeminiLLMNode(Node):
    def __init__(self):
        super().__init__("gemini_llm_node")

        self.client = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))

        self.srv = self.create_service(
            GeminiRequest, 'gemini_llm_service', self.handle_llm_request
        )

        self.get_logger().info("Gemini LLM サービスが起動しました")

    def handle_llm_request(self, request, response):
        self.get_logger().info(f"Received request: {request.input}")

        gemini_response = self.client.models.generate_content(
            model="gemini-2.0-flash",
            contents=request.input,
            config={"max_output_tokens": 30}
        )

        response.output = gemini_response.text.replace('*', '').replace('\n', ' ').strip()
        self.get_logger().info(f"Sending response: {response.output}")

        return response


def main():
    rclpy.init()
    node = GeminiLLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
