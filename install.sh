#!/bin/bash
sudo apt update
sudo apt install -y portaudio19-dev
sudo apt install -y pavucontrol

pip3 install sounddevice
pip3 install -q -U google-genai

echo "export GEMINI_API_KEY=AIzaSyCg-8enfQnTDdiK4txTXqr0fEH7ThASKG4 >> ~/.bashrc"