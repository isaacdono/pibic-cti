# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import speech_recognition as sr
import pyttsx3
import time
import os

os.environ["PYTHONWARNINGS"] = "ignore"
os.environ["ALSA_CARD"] = "default"


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
        self.tts_engine = pyttsx3.init()
        self.get_logger().info("Nó Fala iniciado (STT + TTS).")

        # --- Ajuste de ruído ambiente ---
        with self.microphone as source:
            self.get_logger().info("Calibrando microfone (1s)...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        # --- Inicia Threads ---
        threading.Thread(target=self.listen_loop, daemon=True).start()

    # --- Loop de escuta contínua (STT) ---
    def listen_loop(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Ouvindo...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=8)
                self.get_logger().info("Reconhecendo fala...")

                # Usa o reconhecimento padrão (Google Web Speech API)
                text = self.recognizer.recognize_google(audio, language="pt-BR")

                if text.strip():
                    msg = String()
                    msg.data = text.strip()
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"[STT] Você disse: {text.strip()}")

            except sr.WaitTimeoutError:
                continue  # Nenhum som detectado
            except sr.UnknownValueError:
                self.get_logger().warn("Não entendi o que foi dito.")
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

    # --- Execução do TTS (não bloqueante) ---
    def speak(self, text: str):
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"Erro no TTS: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Talk_Node()
    try:
        rclpy.spin(node)  # mantém o nó ativo (para callbacks do ROS)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
