#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiRequest
import sounddevice as sd
import scipy.io.wavfile as wav
from google import genai


class GeminiSTTNode(Node):
    def __init__(self):
        super().__init__('gemini_stt_node')

        self.client = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))

        workspace_directory = os.path.expanduser('~/colcon_ws/src/gemini_ros')
        self.media_data_path = os.path.join(workspace_directory, 'media_data')

        if not os.path.exists(self.media_data_path):
            os.makedirs(self.media_data_path)

        self.file_name = os.path.join(self.media_data_path, 'input.wav')
        self.samplerate = 44100
        self.duration = 5

        self.srv = self.create_service(
            GeminiRequest, '/gemini_stt_service', self.handle_stt_request
        )

        self.get_logger().info("Gemini STT サービスが起動しました")

    def handle_stt_request(self, request, response):
        self.record_audio()
        self.process_audio_and_generate_response(request, response)
        return response

    def record_audio(self):
        self.get_logger().info('Recording audio...')
        audio_data = sd.rec(int(self.samplerate * self.duration), samplerate=self.samplerate, channels=1)
        sd.wait()
        wav.write(self.file_name, self.samplerate, audio_data)

    def process_audio_and_generate_response(self, request, response):
        sound_file = self.client.files.upload(file=self.file_name)

        gemini_response = self.client.models.generate_content(
            model='gemini-2.0-flash',
            contents=[request.input, sound_file],
            config={
                # "max_output_tokens": 30,
            }
        )

        response.output = gemini_response.text.replace('*', '').replace('\n', ' ').strip()
        self.get_logger().info(f"Response: {response.output}")

        return response

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
