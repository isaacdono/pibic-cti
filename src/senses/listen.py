# stt_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import speech_recognition as sr
import os
import time
import warnings

# (Opcional, mas recomendado)
warnings.filterwarnings("ignore")
os.environ["PYTHONWARNINGS"] = "ignore"
# os.environ["ALSA_CARD"] = "default"

class STT_Node(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.publisher_ = self.create_publisher(String, '/transcript', 10)
        
        self.get_logger().info("Nó STT (Ouvido) iniciado.")

        try:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            with self.microphone as source:
                self.get_logger().info("Calibrando microfone (2s)...")
                self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
        except Exception as e:
            self.get_logger().fatal(f"Falha ao iniciar microfone: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info("Microfone calibrado. Iniciando loop de escuta.")
        threading.Thread(target=self.listen_loop, daemon=True).start()

    def listen_loop(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Ouvindo...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=8)

                # Reconhece (usando Google, mas poderia ser Whisper)
                text = self.recognizer.recognize_google(audio, language="pt-BR")

                if text.strip():
                    msg = String()
                    msg.data = text.strip()
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"[STT] Publicado: {text.strip()}")

            except sr.WaitTimeoutError:
                continue # Ninguém falou, loop normal
            except sr.UnknownValueError:
                continue # Não entendeu, loop normal
            except Exception as e:
                self.get_logger().error(f"Erro no STT: {e}")
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = STT_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()