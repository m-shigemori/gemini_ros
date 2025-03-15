# Gemini ROS2

このリポジトリは、Google Gemini APIを利用して、音声認識（STT）、画像認識（VLM）、テキスト生成（LLM）を行うROS2インターフェースを提供します。これにより、音声や画像データを入力として、AIモデルによるテキスト生成や解析が可能になります。

---

## サービスの概要

- **LLM (Large Language Model)**:  
  テキストを入力として受け取り、その内容に基づいて生成されたレスポンスを返します。

- **STT (Speech-to-Text)**:  
  音声を録音し、それをテキストに変換しレスポンスを返します。

- **VLM (Vision-Language Model)**:  
  カメラからの画像を入力として受け取り、それに基づいたテキストを生成します。

## 依存関係のインストール

まず、必要なパッケージをインストールします。これには音声データの録音、Google Gemini APIとの接続、および画像データ処理に必要なライブラリが含まれます。

### 必要なパッケージのインストール

```bash
#!/bin/bash
sudo apt update
sudo apt install -y portaudio19-dev
sudo apt install -y pavucontrol

pip3 install sounddevice
pip3 install -q -U google-genai
```

これにより、音声録音用の`portaudio`、音声データ処理用の`sounddevice`、およびGoogle Gemini APIを利用するための`google-genai`ライブラリがインストールされます。

## サービスの立ち上げ方法

以下に、各サービスを起動するための`launch`ファイルを使用した方法を示します。

### 1. Gemini LLM サービス（テキスト生成）

#### ノードの起動

`gemini_llm_launch.py`という`launch`ファイルを使って、LLMサービスを起動します。このファイルを使うことで、直接コマンドでノードを実行することなく、ROS2で簡単にサービスを起動できます。

```bash
ros2 launch gemini_ros gemini_llm_launch.py
```

これにより、LLMサービスが立ち上がり、リクエストに基づいてテキスト生成が行えるようになります。

#### サービス呼び出し方法

`gemini_llm_service`に対して、テキストリクエストを送信し、生成されたレスポンスを受け取ります。

```bash
ros2 service call /gemini_llm_service gemini_interface/srv/GeminiRequest "input: 'テキストに基づいて生成したい内容'"
```

#### 使用例

```bash
ros2 service call /gemini_llm_service gemini_interface/srv/GeminiRequest "input: 'おすすめ商品を教えてください'"
```

### 2. Gemini STT サービス（音声認識）

#### ノードの起動

`gemini_stt_launch.py`という`launch`ファイルを使って、STTサービスを起動します。この`launch`ファイルを使って、音声データを録音し、テキストに変換するサービスを簡単に立ち上げます。

```bash
ros2 launch gemini_ros gemini_stt_launch.py
```

これにより、STTサービスが立ち上がり、音声認識が可能になります。

#### サービス呼び出し方法

`gemini_stt_service`に対して、音声データを録音し、その音声をGoogle Gemini APIで処理してテキストに変換します。音声録音後、生成されたテキストがレスポンスとして返されます。

```bash
ros2 service call /gemini_stt_service gemini_interface/srv/GeminiRequest "input: '音声データに基づいて処理を行う内容'"
```

#### 使用例

```bash
ros2 service call /gemini_stt_service gemini_interface/srv/GeminiRequest "input: '会話を続けてください'"
```

### 3. Gemini VLM サービス（画像認識）

#### ノードの起動

`gemini_vlm_launch.py`という`launch`ファイルを使って、VLMサービスを起動します。この`launch`ファイルを使って、カメラから画像を取得し、その画像に基づいてテキストを生成するサービスを簡単に立ち上げます。

```bash
ros2 launch gemini_ros gemini_vlm_launch.py
```

これにより、VLMサービスが立ち上がり、画像認識が可能になります。

#### サービス呼び出し方法

`gemini_vlm_service`に対して、カメラから自動的に画像を取得し、その画像に基づいたテキストを生成します。

```bash
ros2 service call /gemini_vlm_service gemini_interface/srv/GeminiRequest "input: '画像に関連するテキスト生成の指示'"
```

#### 使用例

```bash
ros2 service call /gemini_vlm_service gemini_interface/srv/GeminiRequest "input: 'この画像を見て、何が写っているか教えて'"
```

## 注意点

- **会話のコンテキスト維持ができていない**:  
  現在の実装では、各リクエストが独立して処理されるため、会話の文脈を維持することができません。連続的な対話を行いたい場合は、文脈を管理する追加の実装が必要です。

- **STTの録音は5秒に制限されている**:  
  現在、STTサービスでの音声録音は最大5秒間に制限されています。長時間の音声認識が必要な場合は、録音時間の延長や別の音声認識処理の追加を検討する必要があります。