import rclpy
from rclpy.node import Node
from gemini_interface.srv import GeminiService
import sys

class GeminiServiceClient(Node):
    def __init__(self):
        super().__init__('gemini_service_client')
        self.cli = self.create_client(GeminiService, 'gemini_service')

        self.get_logger().info('サービス gemini_service を待機中...')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('サービスが見つかりませんでした。')
            sys.exit(1)

        self.get_logger().info('サービスが見つかりました。')
        self.req = GeminiService.Request()

    def send_request(self, mode, prompt, image_path='', audio_path=''):
        self.req.mode = mode
        self.req.prompt = prompt
        self.req.image_file_path = image_path
        self.req.audio_file_path = audio_path

        future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('サービス呼び出しに失敗しました')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = GeminiServiceClient()

    zenkaku_digits = '０１２３４５６７８９'
    hankaku_digits = '0123456789'
    translation_table = str.maketrans(zenkaku_digits, hankaku_digits)

    print("=== Gemini Service Client ===")
    print("利用可能なモード:")
    print("  1: LLM (テキスト)")
    print("  2: VLM (画像)")
    print("  3: STT (音声)")
    print("  q: 終了")

    while rclpy.ok():
        try:
            mode_input = input("\nモードを選択してください (1/2/3/q): ")

            mode_input = mode_input.translate(translation_table)

            if mode_input.lower() == 'q':
                print("クライアントを終了します。")
                break

            result = None

            if mode_input == '1':
                prompt = input("プロンプトを入力してください: ")
                result = client.send_request('LLM', prompt)

            elif mode_input == '2':
                image_path = input("画像ファイルパスを入力してください: ")
                prompt = input("画像に対するプロンプトを入力してください: ")
                result = client.send_request('VLM', prompt, image_path=image_path)

            elif mode_input == '3':
                audio_path = input("音声ファイルパスを入力してください: ")
                prompt = input("音声に対するプロンプトを入力してください: ")
                result = client.send_request('STT', prompt, audio_path=audio_path)

            else:
                print("無効な選択です。もう一度入力してください。")
                continue

            if result is not None:
                print(f"\n[応答]:\n{result.response}")

        except KeyboardInterrupt:
            print("\n割り込みを検出しました。終了します。")
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()