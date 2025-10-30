# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import speech_recognition as sr
import sounddevice as sd
import numpy as np
from kokoro import KPipeline
import warnings
import time
import os
import torch

warnings.filterwarnings("ignore")
os.environ["PYTHONWARNINGS"] = "ignore"
os.environ["ALSA_CARD"] = "default"

KOKORO_VOICE = "pf_dora"  # voz PT-BR
KOKORO_SPEED = 1.3       # velocidade da fala

class Talk_Node(Node):
    def __init__(self):
        super().__init__('talk_node')

        # --- ROS Setup ---
        self.publisher_ = self.create_publisher(String, '/transcript', 10)
        self.subscription = self.create_subscription(
            String, '/tts_command', self.tts_callback, 10
        )

        # --- STT / TTS Setup ---
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.tts_pipeline = KPipeline(lang_code="p", repo_id="hexgrad/Kokoro-82M")

        self.get_logger().info("Nó Fala iniciado (STT + TTS com Kokoro).")

        # --- Ajuste de ruído ambiente ---
        with self.microphone as source:
            self.get_logger().info("Calibrando microfone (1s)...")
            self.recognizer.adjust_for_ambient_noise(source, duration=2.0)

        # --- Inicia Threads ---
        threading.Thread(target=self.listen_loop, daemon=True).start()

    # --- Loop de escuta contínua (STT) ---
    def listen_loop(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Ouvindo...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=8)
                # self.get_logger().info("Reconhecendo fala...")

                # Google Web Speech API em PT-BR
                text = self.recognizer.recognize_google(audio, language="pt-BR")

                if text.strip():
                    msg = String()
                    msg.data = text.strip()
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"[STT] Você disse: {text.strip()}")

            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                # self.get_logger().warn("Não entendi o que foi dito.")
                continue
            except Exception as e:
                self.get_logger().error(f"Erro no STT: {e}")
                time.sleep(1)

    # --- Callback TTS ---
    def tts_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"[TTS] Falando: {text}")
        threading.Thread(target=self.speak, args=(text,), daemon=True).start()

    # --- Execução do TTS Kokoro (bloqueante) ---
    def speak(self, text: str):
        try:
            gen = self.tts_pipeline(text, voice=KOKORO_VOICE, speed=KOKORO_SPEED)
            for _, _, chunk in gen:
                if chunk is None or len(chunk) == 0:
                    continue
                else:
                    audio_np = chunk.cpu().numpy().astype("float32")
                    sd.play(audio_np, 24000, blocking=True)

        except Exception as e:
            self.get_logger().error(f"Erro no TTS: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Talk_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
