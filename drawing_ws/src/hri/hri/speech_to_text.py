#!/usr/bin/env python3

import numpy as np
import torch
torch.set_num_threads(1)
import torchaudio
torchaudio.set_audio_backend("soundfile")
import pyaudio
from threading import Thread
import whisper
from unidecode import unidecode

from gtts import gTTS
import playsound
import pyttsx3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

say_voice = "com.apple.speech.synthesis.voice.paulina"



class VAD_Text_to_Speech(Node):
    
    def __init__(self, silero_model,silero_utils, whisper_model, say_engine=None) -> None:
        self.silero_model = silero_model
        self.whisper_model = whisper_model
        self.utils = silero_utils
        self.say_engine = say_engine
        self.speaking = False
        
        # Initialize the node
        print("Initializing VAD_Text_to_Speech")
        super().__init__('vad_text_to_speech')
        self.text_publisher_ = self.create_publisher(String, 'whisper_text', 10)
        
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        SAMPLE_RATE = 16000
        CHUNK = int(SAMPLE_RATE / 10)

        audio = pyaudio.PyAudio()
        
        self.sample_time = 750 #ms
        self.num_samples = int(SAMPLE_RATE * self.sample_time / 1000)

        self.stream = audio.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=SAMPLE_RATE,
                            input=True,
                            frames_per_buffer=CHUNK,
                            )
        self.get_logger().info('VAD_Text_to_Speech node is running')    
        
        self.listen()
    
    def speak(self,text):
        self.speaking = True
        tts = gTTS(text=text, lang='es')
        filename = 'voice.mp3'
        tts.save(filename)
        playsound.playsound(filename)
        self.speaking = False


    def listen(self):
        # Taken from utils_vad.py
        def validate(self,model,
                    inputs: torch.Tensor):
            with torch.no_grad():
                outs = model(inputs)
            return outs

        # Provided by Alexander Veysov
        def int2float(sound):
            abs_max = np.abs(sound).max()
            sound = sound.astype('float32')
            if abs_max > 0:
                sound *= 1/32768
            sound = sound.squeeze()  # depends on the use case
            return sound
        data = []
        
        print("Saying...")
        self.speak("¿Qué palabra quieres escribir?")
        print("Started listening")
        recorded_audio = []

        while (self.speaking):
            print()
            # wait for the speaking to finish
        
        while True:
            audio_chunk = self.stream.read(self.num_samples, exception_on_overflow = False)
        
            # in case you want to save the audio later
            data.append(audio_chunk)
            
            audio_int16 = np.frombuffer(audio_chunk, np.int16);

            audio_float32 = int2float(audio_int16)
            
            # get the confidences and add them to the list to plot them later
            confidence = self.silero_model(torch.from_numpy(audio_float32), 16000).item()
            if confidence > 0.75:
                if len(recorded_audio) == 0:
                    print("Started recording")
                recorded_audio.append(audio_chunk)
            else:
                if len(recorded_audio) > 0:
                    if len(recorded_audio) * self.sample_time < 1000:
                        print("Recording too short")
                        recorded_audio = []
                        continue
                    self.process_audio(recorded_audio)
                    recorded_audio = []
        
    def process_audio(self, recorded_audio):
        recorded_audio = b"".join(recorded_audio)
        audio_np = np.frombuffer(recorded_audio, dtype=np.int16).astype(np.float32) / 32768.0
        thread = Thread(target=self.whisper_inference, args=(audio_np,))
        thread.start()
        print("Done")
    
    def whisper_inference(self, audio_np):
        print(f"Processing Whisper...")
        result = self.whisper_model.transcribe(audio_np, language="es", fp16=torch.cuda.is_available())
        text = result['text'].strip()
        print(f"Whisper: {text}")
        text_msg = String()
        text_msg.data = text
        self.text_publisher_.publish(text_msg)

if __name__ == "__main__" :
    say_engine = pyttsx3.init()
    say_engine.setProperty('voice', say_voice)
    print("Loading Silero VAD...")
    silero_model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False)
    WHISPER_MODEL = "small"
    whisper_model = whisper.load_model(WHISPER_MODEL)
    print("Whisper initialized, starting rclpy...")
    rclpy.init()
    print("rclpy initialized, starting VAD...")
    vad = VAD_Text_to_Speech(silero_model, utils, whisper_model, say_engine=say_engine)
    rclpy.spin(vad)
    vad.destroy_node()
    rclpy.shutdown()