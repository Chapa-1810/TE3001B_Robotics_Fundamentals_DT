import io
import numpy as np
import torch
torch.set_num_threads(1)
import torchaudio
import matplotlib
import matplotlib.pylab as plt
torchaudio.set_audio_backend("soundfile")
import pyaudio
import os
from threading import Thread
import whisper

silero_model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=True)
WHISPER_MODEL = "base"


class VAD_Text_to_Speech:
    
    def __init__(self, silero_model,silero_utils, whisper_model) -> None:
        self.silero_model = silero_model
        self.whisper_model = whisper_model
        self.utils = silero_utils
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        SAMPLE_RATE = 16000
        CHUNK = int(SAMPLE_RATE / 10)

        audio = pyaudio.PyAudio()
        
        sample_time = 500 #ms
        self.num_samples = int(SAMPLE_RATE * sample_time / 1000)

        self.stream = audio.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=SAMPLE_RATE,
                            input=True,
                            frames_per_buffer=CHUNK)
    
    

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
        voiced_confidences = []

        frames_to_record = 50

        print("Started Recording")
        
        recorded_audio = []

        while True:
            audio_chunk = self.stream.read(self.num_samples)
            
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
                    if len(recorded_audio) < 3:
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
        pass

if __name__ == "__main__" :
    whisper_model = whisper.load_model(WHISPER_MODEL)
    vad = VAD_Text_to_Speech(silero_model, utils, whisper_model)
    vad.listen()