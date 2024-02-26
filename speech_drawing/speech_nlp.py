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
import socket
import sys
import spacy

silero_model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', force_reload=False)
WHISPER_MODEL = "base"
spacy_model_name = "es_dep_news_trf"
HOSTNAME = '127.0.0.1'  # The server's hostname or IP address
PORT = 65432      # The port used by the server


class VAD_Text_to_Speech:
    
    def __init__(self, silero_model,silero_utils, whisper_model, spacy_model, text_socket=None, socket_type=None) -> None:
        self.silero_model = silero_model
        self.whisper_model = whisper_model
        self.spacy_model = spacy_model
        self.utils = silero_utils
        self.socket = text_socket
        self.socket_type = socket_type
        if self.socket is not None:
            if socket_type == "client":
                self.socket["socket"].connect((self.socket["hostname"], self.socket["port"]))
            elif socket_type == "server":
                try:
                    self.socket["socket"].bind((self.socket["hostname"], self.socket["port"]))
                except Exception as e:
                    self.socket["socket"].close()
                    self.socket["socket"] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket["socket"].bind((self.socket["hostname"], self.socket["port"]))
                self.socket["socket"].listen(1)
                print(f"Listening on {self.socket['hostname']}:{self.socket['port']}")
                print("Waiting for connection...")
                self.client, self.addr = self.socket["socket"].accept()
                print(f"Connected to {self.addr}")
        else:
            print("No socket provided")
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
                            frames_per_buffer=CHUNK,
                            )
    
    

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

        print("Started listening")
        
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
        
        target_word, target_face = self.get_target_word_and_face(text)
        
        print(f"Target word is {target_word}, target face is {target_face}")
        
        if self.socket is not None:
            #uppercase
            target_word = target_word.text.upper()
            target_face = target_face.text.upper()
            msg = bytes(f"{target_word};{target_face};", "utf-8")
            if self.socket_type == "server":
                try:
                    self.client.sendall(msg)
                except socket.error as e:
                    self.client.close()
                    self.socket["socket"].listen(1)
                    self.client, self.addr = self.socket["socket"].accept()
                    print(f"Connected to {self.addr}")
                    self.client.sendall(msg)
                pass
            else:
                self.socket["socket"].sendall(msg)
    
    def get_target_word_and_face(self, text):
        doc = self.spacy_model(text)
        verb_pos = 0
        face_num = 0
        target_num = 0

        for pos, token in enumerate(doc):
            if token.pos_ == "VERB":
                verb_pos = pos

        for pos, token in enumerate(doc[verb_pos:]):
            if ((token.pos_ == "NOUN") or (token.pos_ == "PROPN")) and token.text != "palabra": 
                target_num = pos
                break

        for pos, token in enumerate(doc[verb_pos:]):
            if token.is_digit or token.like_num:
                face_num = pos

        return doc[verb_pos+target_num], doc[face_num+verb_pos]

if __name__ == "__main__" :
    text_socket = {
        "hostname": HOSTNAME,
        "port": PORT,
        "socket": socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    }
    spacy_model = spacy.load(spacy_model_name)
    whisper_model = whisper.load_model(WHISPER_MODEL)
    vad = VAD_Text_to_Speech(silero_model, utils, whisper_model, spacy_model, text_socket, socket_type="client")
    #vad = VAD_Text_to_Speech(silero_model, utils, whisper_model, spacy_model)
    vad.listen()