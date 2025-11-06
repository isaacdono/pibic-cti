# tts_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
from kokoro import KPipeline
import warnings
import os

warnings.filterwarnings("ignore")
os.environ["PYTHONWARNINGS"] = "ignore"
os.environ["ALSA_CARD"] = "default"

KOKORO_VOICE = "pf_dora"
KOKORO_SPEED = 1.3

class TTS_Node(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String, '/tts_command', self.tts_callback, 10
        )
        
        try:
            self.get_logger().info("Carregando modelo TTS (Kokoro)...")
            self.tts_pipeline = KPipeline(lang_code="p", repo_id="hexgrad/Kokoro-82M")
            self.get_logger().info("Nó TTS (Voz) iniciado e pronto.")
        except Exception as e:
            self.get_logger().fatal(f"Falha ao carregar Kokoro: {e}")
            self.destroy_node()
            rclpy.shutdown()
            return

    def tts_callback(self, msg: String):
        """
        Callback bloqueante. Como este nó SÓ fala, não há problema
        em bloquear o 'spin' enquanto a fala está sendo gerada.
        """
        text = msg.data.strip()
        if not text:
            return
            
        self.get_logger().info(f"[TTS] Recebido: {text}")
        
        try:
            # Não precisamos mais de thread aqui!
            gen = self.tts_pipeline(text, voice=KOKORO_VOICE, speed=KOKORO_SPEED)
            
            # O stream do Kokoro
            for _, _, chunk in gen:
                if chunk is None or len(chunk) == 0:
                    continue
                else:
                    audio_np = chunk.cpu().numpy().astype("float32")
                    # sd.play é bloqueante, o que é OK aqui.
                    sd.play(audio_np, 24000, blocking=True)
                    
            self.get_logger().info(f"[TTS] Fala concluída.")

        except Exception as e:
            self.get_logger().error(f"Erro no TTS: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TTS_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()