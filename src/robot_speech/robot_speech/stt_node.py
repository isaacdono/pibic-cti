import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import threading

class STTNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, '/transcript', 10)
        self.get_logger().info('Nó STT (Whisper offline) iniciado.')

        # Carrega o modelo Whisper offline
        self.model = whisper.load_model("base")  # tiny/base/small/medium/large
        self.get_logger().info('Modelo Whisper carregado.')

        # Thread de captura de áudio
        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()

    def listen_loop(self):
        while rclpy.ok():
            self.get_logger().info('Ouvindo...')
            # Captura 5s de áudio do microfone
            audio = sd.rec(int(5*16000), samplerate=16000, channels=1)
            sd.wait()
            audio = np.squeeze(audio)

            # Transcreve com Whisper offline
            result = self.model.transcribe(audio, language='pt')
            text = result['text']
            self.get_logger().info(f'Texto reconhecido: "{text}"')

            msg = String()
            msg.data = text
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
