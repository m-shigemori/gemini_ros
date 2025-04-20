# gemini_ros

このパッケージは、GoogleのGeminiモデルをROS 2環境で利用するための機能を提供します。

## パッケージ構成

- **gemini_bringup**: Gemini関連のノードを起動するためのLaunchファイルが含まれています。
- **gemini_interface**: Geminiとの通信に使用するカスタムサービス定義(`.srv`ファイル)が含まれています。
- **gemini_ros**: GeminiのサービスサーバーとクライアントのROS 2ノードのPythonスクリプト、設定ファイル、およびその他の関連ファイルが含まれています。
  - `config`: 設定ファイル(`.yaml`)が含まれています。
  - `data_prompts`: テスト用のデータプロンプトファイルが含まれています。

## 主要な機能

- **Geminiサービスサーバー**: Gemini APIと通信し、テキスト生成、画像理解、音声認識などの機能を提供するROS 2サービスを提供します。
- **Geminiサービスクライアント**: Geminiサービスサーバーにリクエストを送信し、応答を受け取るためのROS 2クライアントノードを提供します。
- **CRISPEフレームワークのサポート**: プロンプトエンジニアリングのためのCRISPEフレームワークを統合し、より効果的な対話を実現します (設定ファイルで有効/無効を切り替え可能)。
- **対話履歴の管理**: 過去の対話を保存し、コンテキストとして利用することで、より自然な対話を可能にします (履歴の長さは設定可能)。
- **LLM、VLM、STTモードのサポート**: テキスト生成 (LLM)、画像理解 (VLM)、音声認識 (STT) の各モードに対応しています。

## 依存関係

- ROS 2 (Humble Hawksbill)
- `rclpy` (ROS 2 Pythonクライアントライブラリ)
- `gemini-pro` (Google Gemini APIのPythonクライアントライブラリ)

## インストール手順

1. ROS 2環境がセットアップされていることを確認してください。
2. このパッケージをROS 2ワークスペースの `src` ディレクトリにクローンします。
   ```bash
   cd ~/colcon_ws/src
   git clone <このリポジトリのURL>
   ```
3. ワークスペースをビルドします。
   ```bash
   cd ~/colcon_ws
   colcon build
   ```
4. ROS 2環境をソースします。
   ```bash
   source install/setup.bash
   ```

## 使い方

1. Geminiサービスサーバーノードを起動します。APIキーなどの必要なパラメータは、Launchファイルまたは個別の設定ファイルで指定できます。
   ```bash
   ros2 launch gemini_bringup gemini_service.launch.py
   ```
2. Geminiサービスクライアントノードを起動し、対話を開始します。クライアントノードは、LLM (テキスト)、VLM (画像)、STT (音声) の各モードをサポートしています。
   ```bash
   ros2 run gemini_ros gemini_service_client
   ```
   クライアントの指示に従って、モードを選択し、プロンプトやファイルパスを入力してください。

## 設定

- **モデル名**: 使用するGeminiモデルを指定します (`model_name`パラメータ)。デフォルトは `gemini-2.0-flash` です。
- **APIキー**: Google CloudのGemini APIキーを設定します (`api_key`パラメータ)。
- **最大出力トークン数**: Geminiモデルからの応答の最大トークン数を設定します (`max_output_tokens`パラメータ)。
- **CRISPEの使用**: CRISPEフレームワークを使用するかどうかを設定します (`use_crispe`パラメータ)。
- **対話履歴の使用**: 対話履歴を保存し使用するかどうかを設定します (`use_history`パラメータ)。
- **履歴の長さ**: 保存する過去の対話の最大数を設定します (`history_length`パラメータ)。
- **CRISPEコンテキスト**: `config/crispe_framework.yaml` ファイルで、CRISPEフレームワークの各要素 (capacity, role, insight, statement, personality, experiment) を設定できます。

## 注意事項

- Gemini APIの利用にはAPIキーが必要です。安全にAPIキーを管理してください。
- VLMモードおよびSTTモードでは、指定された画像ファイルまたは音声ファイルが存在することを確認してください。