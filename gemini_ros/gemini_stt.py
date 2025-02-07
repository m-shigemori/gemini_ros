#!/usr/bin/env python3
import os
from google import genai
import sounddevice as sd
import scipy.io.wavfile as wav

api_key = os.environ.get('GEMINI_API_KEY')
client = genai.Client(api_key=api_key)

file_name = 'output.wav'
SAMPLERATE = 44100
DURATION = 5

audio_data = sd.rec(int(SAMPLERATE * DURATION), samplerate=SAMPLERATE, channels=1)
sd.wait()
wav.write('output.wav', SAMPLERATE, audio_data)

sound_file = client.files.upload(file=file_name)

response = client.models.generate_content(
    model='gemini-2.0-flash',
    contents=['注文した商品を読み上げて', sound_file]
)

print(response.text)