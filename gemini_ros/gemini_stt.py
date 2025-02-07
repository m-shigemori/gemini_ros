#!/usr/bin/env python3
import os
import sounddevice as sd
import scipy.io.wavfile as wav
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from google import genai


class GeminiSTTNode(Node):
    def __init__(self):
        super().__init__('gemini_stt_node')

        api_key = os.environ.get("GEMINI_API_KEY")
        self.client = genai.Client(api_key=api_key)

        self.SAMPLERATE = 44100
        self.DURATION = 5
        self.FILE_NAME = 'output.wav'


        self.create_subscription(
            String, '/gemini_stt_request', self.handle_stt_request, 10
        )

        self.response_publisher = self.create_publisher(
            String, '/gemini_stt_response', 10
        )

        self.get_logger().info("Gemini STT が起動しました")

    def handle_stt_request(self, msg):
        # print(sd.query_devices())
        if msg.data == 'start':
            self.record_audio()

    def record_audio(self):
        self.get_logger().info('Recording audio...')
        audio_data = sd.rec(int(self.SAMPLERATE * self.DURATION), samplerate=self.SAMPLERATE, channels=1)
        sd.wait()
        wav.write(self.FILE_NAME, self.samplerate, audio_data)

        self.get_logger().info(f'Audio saved to {self.FILE_NAME}')
        self.publish_response()

    def publish_response(self):
        sound_file = self.client.files.upload(file=self.FILE_NAME)

        response = self.client.models.generate_content(
            model='gemini-2.0-flash',
            contents=['注文した商品を読み上げて', sound_file],
            config={
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
    node = GeminiSTTNode()

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
