#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage  # Usando Pillow
import os

class VisionSaverNode(Node):
    def __init__(self):
        super().__init__('vision_saver_node_pil')
        self.subscription = self.create_subscription(
            Image, '/image', self.callback, 10)
        self.file_path = "/tmp/robot_latest_view.jpg"
        self.get_logger().info(f"Salvando frames em: {self.file_path} (usando PIL)")

    def callback(self, msg: Image):
        try:
            # Garante que o encoding seja o esperado para conversão direta
            if msg.encoding != 'rgb8':
                self.get_logger().warn(f"Encoding inesperado: {msg.encoding}. Esperado 'rgb8'.")
                return

            # Cria a imagem PIL diretamente dos bytes da mensagem
            pil_image = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
            
            # Inverte a imagem verticalmente (problema comum com simuladores)
            flipped_image = pil_image.transpose(PILImage.FLIP_TOP_BOTTOM)
            
            # Salva a imagem no disco, sobrescrevendo a anterior
            # O método save do PIL infere o formato pelo nome do arquivo (.jpg)
            flipped_image.save(self.file_path)

        except Exception as e:
            self.get_logger().error(f"Falha ao salvar imagem com PIL: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Limpa o arquivo temporário ao sair
        if os.path.exists("/tmp/robot_latest_view.jpg"):
            os.remove("/tmp/robot_latest_view.jpg")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()