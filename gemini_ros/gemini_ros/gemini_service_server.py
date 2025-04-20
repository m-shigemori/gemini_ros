import os
import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiService
from google import genai

class GeminiServiceServer(Node):
    def __init__(self):
        super().__init__('gemini_service_server')

        self.declare_parameter('model_name', 'gemini-2.0-flash')
        self.declare_parameter('api_key', '')
        self.declare_parameter('max_output_tokens', 256)
        self.declare_parameter('use_crispe', True)
        self.declare_parameter('use_history', True)
        self.declare_parameter('history_length', 10)
        self.declare_parameter('capacity', '')
        self.declare_parameter('role', '')
        self.declare_parameter('insight', '')
        self.declare_parameter('statement', '')
        self.declare_parameter('personality', '')
        self.declare_parameter('experiment', '')

        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        self.max_output_tokens = self.get_parameter('max_output_tokens').get_parameter_value().integer_value
        self.use_crispe = self.get_parameter('use_crispe').get_parameter_value().bool_value
        self.use_history = self.get_parameter('use_history').get_parameter_value().bool_value
        self.history_length = self.get_parameter('history_length').get_parameter_value().integer_value

        if self.api_key == '':
            self.get_logger().error('API key not set')
            exit(1)

        self.client = genai.Client(api_key=self.api_key)
        self.history = []

        self.crispe_prompt = "\n".join([
            self.get_parameter('capacity').get_parameter_value().string_value,
            self.get_parameter('role').get_parameter_value().string_value,
            self.get_parameter('insight').get_parameter_value().string_value,
            self.get_parameter('statement').get_parameter_value().string_value,
            self.get_parameter('personality').get_parameter_value().string_value,
            self.get_parameter('experiment').get_parameter_value().string_value,
        ]) if self.use_crispe else ""

        self.srv = self.create_service(GeminiService, 'gemini_service', self.gemini_callback)

    def gemini_callback(self, request, response):
        contents = []
        prompt = ""

        if self.use_history:
            contents.extend(self.history)

        if self.crispe_prompt:
            prompt += self.crispe_prompt + "\n"
            
        prompt += "[Prompt]\n" + request.prompt

        if request.mode == 'LLM':
            contents.append(prompt)
            
        elif request.mode == 'VLM':
            if not os.path.exists(request.image_file_path):
                error_msg = f"Image file not found: {request.image_file_path}"
                self.get_logger().error(error_msg)
                response.response = error_msg
                return response

            uploaded_file = self.client.files.upload(file=request.image_file_path)
            contents.extend([uploaded_file, prompt])
            
        elif request.mode == 'STT':
            if not os.path.exists(request.audio_file_path):
                error_msg = f"Audio file not found: {request.audio_file_path}"
                self.get_logger().error(error_msg)
                response.response = error_msg
                return response

            uploaded_file = self.client.files.upload(file=request.audio_file_path)
            contents.extend([prompt, uploaded_file])
            
        else:
            response.response = 'Invalid mode'
            return response

        result = self.client.models.generate_content(
            model=self.model_name,
            contents=contents,
            config={"max_output_tokens": self.max_output_tokens}
        )

        response.response = result.text

        if self.use_history:
            self.history.append(request.prompt)
            self.history.append(result.text)
            while len(self.history) > self.history_length * 2:
                self.history = self.history[2:]

        return response

def main(args=None):
    rclpy.init(args=args)
    node = GeminiServiceServer()
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
