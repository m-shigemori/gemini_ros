#!/usr/bin/env python3
import os
from google import genai
import PIL.Image

api_key = os.environ.get('GEMINI_API_KEY')
client = genai.Client(api_key=api_key)

image = PIL.Image.open('test.jpg')

response = client.models.generate_content(
    model="gemini-2.0-flash",
    contents=["これはどういう画像?", image],
    config={
        # "max_output_tokens": 10,
    }
)

print(response.text)