#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from google import genai

class GeminiLLMNode(Node):
    def __init__(self):
        super().__init__("gemini_llm")

        api_key = os.environ.get("GEMINI_API_KEY")
        self.client = genai.Client(api_key=api_key)

        self.subscription = self.create_subscription(
            String, "gemini_request", self.request_callback, 10
        )
        self.publisher = self.create_publisher(String, "gemini_response", 10)

        self.get_logger().info("Gemini LLM が起動しました")

    def request_callback(self, msg):
        self.get_logger().info(msg.data)

        response = self.client.models.generate_content(
            model="gemini-2.0-flash",
            contents=msg.data + "30文字以内で答えて",
            config={
                # "max_output_tokens": 30,
            }
        )

        cleaned_response = response.text.replace('*', '').replace('\n', ' ').strip()

        response_msg = String()
        response_msg.data = cleaned_response

        self.publisher.publish(response_msg)

        # self.get_logger().info(cleaned_response)

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
