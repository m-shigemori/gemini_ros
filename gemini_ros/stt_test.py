#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiRequest

class GeminiSTTClient(Node):
    def __init__(self):
        super().__init__('gemini_stt_client')
        self.client = self.create_client(GeminiRequest, '/gemini_stt_service')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービスが利用できるまで待機中...')

    def call_service(self, input_data):
        request = GeminiRequest.Request()
        request.input = input_data

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"受信したレスポンス: {future.result().output}")
        else:
            self.get_logger().error('サービス呼び出しに失敗しました')

def main():
    rclpy.init()
    client = GeminiSTTClient()

    try:
        input_data = (f"お客から注文内容を確認した後のレスポンスです"
                      f"注文内容が正しかった場合「yes」"
                      f"違う場合や訂正したい場合は「no」と答えてください"
                      )

        client.call_service(input_data)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()