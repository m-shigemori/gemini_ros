# Gemini ROS2

![gemini_ros package image](gemini_ros.jpg)

このパッケージは、GoogleのGeminiモデルをROS 2環境で利用するための機能を提供します。

## パッケージ構成

- **gemini_bringup**: Gemini関連のノードを起動するためのLaunchファイルが含まれています
- **gemini_interface**: Geminiとの通信に使用するカスタムサービス定義(`.srv`ファイル)が含まれています
- **gemini_ros**: GeminiのサービスサーバーとクライアントのROS 2ノードのPythonスクリプト、設定ファイル、およびその他の関連ファイルが含まれています
  - `config`: 設定ファイル(`.yaml`)が含まれています
  - `data_prompts`: テスト用のデータプロンプトファイルが含まれています

## 主要な機能

- **Geminiサービスサーバー**: Gemini APIと通信し、テキスト生成、画像理解、音声認識などの機能を提供するROS 2サービスを提供します
- **Geminiサービスクライアント**: Geminiサービスサーバーにリクエストを送信し、応答を受け取るためのROS 2クライアントノードを提供します
- **CRISPEフレームワークのサポート**: プロンプトエンジニアリングのためのCRISPEフレームワークを統合し、より効果的な対話を実現します (設定ファイルで有効/無効を切り替え可能)
- **対話履歴の管理**: 過去の対話を保存し、コンテキストとして利用することで、より自然な対話を可能にします (履歴の長さは設定可能)
- **LLM、VLM、STTモードのサポート**: テキスト生成 (LLM)、画像理解 (VLM)、音声認識 (STT) の各モードに対応しています

## 依存関係

- ROS 2 (Humble Hawksbill)

## Gemini APIキーの取得

本パッケージを利用するには、Google Gemini APIのAPIキーが必要です。以下の手順を参考に、APIキーを取得してください。

1.  [Google AI Studio](https://aistudio.google.com/) にアクセスします。
2.  左側のメニューから「Get API key」を選択します。
3.  新しいAPIキーを作成するか、既存のAPIキーを選択します。

詳細については、[Google AI for Developers のドキュメント](https://ai.google.dev/gemini-api/docs?hl=ja) を参照してください。

## インストール手順

1. ROS 2環境がセットアップされていることを確認してください。
2. このパッケージをROS 2ワークスペースの `src` ディレクトリにクローンします。
   ```bash
   cd ~/colcon_ws/src
   git clone https://github.com/TeamSOBITS/gemini_ros.git
   ```
3. 依存関係スクリプトの実行
   ```bash
   cd gemini_ros
   bash install.sh
   ```

4. ワークスペースをビルドします。
   ```bash
   cd ~/colcon_ws
   colcon build
   ```
5. ROS 2環境をソースします。
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

- **使用例**
  ```bash
  ros2 run gemini_ros gemini_service_client
  
  === Gemini Service Client ===
  利用可能なモード:
    1: LLM (テキスト)
    2: VLM (画像)
    3: STT (音声)
    q: 終了

  モードを選択してください (1/2/3/q): 3
  音声ファイルパスを入力してください: /home/sobits/colcon_ws/src/gemini_ros/gemini_ros/data_prompts/audio.mp3 
  音声に対するプロンプトを入力してください: この曲を知っていますか？

  [応答]:
  はい、この曲は「大きな古時計」ですね。平井堅さんのカバーバージョンが特に有名ですね。

  モードを選択してください (1/2/3/q): 1
  プロンプトを入力してください: 作曲されたのはいつですか？

  [応答]:
  「大きな古時計」は、アメリカの作曲家ヘンリー・クレイ・ワークによって1876年に作曲されました。
  ```

## 設定

- **モデル名**: 使用するGeminiモデルを指定します (`model_name`パラメータ)。デフォルトは `gemini-2.0-flash` です

- **APIキー**: Google CloudのGemini APIキーを設定します (`api_key`パラメータ)

- **最大出力トークン数**: Geminiモデルからの応答の最大トークン数を設定します (`max_output_tokens`パラメータ)

- **CRISPEの使用**: CRISPEフレームワークを使用するかどうかを設定します (`use_crispe`パラメータ)

- **対話履歴の使用**: 対話履歴を保存し使用するかどうかを設定します (`use_history`パラメータ)

- **履歴の長さ**: 保存する過去の対話の最大数を設定します (`history_length`パラメータ)

- **CRISPEコンテキスト**: `config/crispe_framework.yaml` ファイルで、CRISPEフレームワークの各要素 (capacity, role, insight, statement, personality, experiment) を設定できます

## 注意事項

- Gemini APIの利用にはAPIキーが必要です。安全にAPIキーを管理してください。launchファイルに直接記述してください(環境変数のほうが良かったかも…)。
- VLMモードおよびSTTモードでは、指定された画像ファイルまたは音声ファイルが存在することを確認してください。
