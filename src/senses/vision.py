#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import os

class VisionSaverNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.callback, 10)
        
        # Arquivo de sinal: quando existe, a visão deve ser capturada
        self.vision_request_file = "/tmp/vision_request"
        
        # Caminhos para salvar a imagem
        self.final_file_path = "/tmp/robot_latest_view.jpg"
        self.temp_file_path = "/tmp/robot_latest_view.jpg.tmp"
        
        self.get_logger().info(f"Vision Node iniciado. Aguardando sinal em: {self.vision_request_file}")

    def callback(self, msg: Image):
        try:
            # Só processa se houver requisição de visão
            if not os.path.exists(self.vision_request_file):
                return
            
            if msg.encoding != 'rgb8':
                self.get_logger().warn(f"Encoding inesperado: {msg.encoding}.")
                return

            pil_image = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
            
            # 1. Salva a imagem no arquivo temporário
            pil_image.save(self.temp_file_path, format='JPEG')
            
            # 2. Renomeia atomicamente para o arquivo final
            os.rename(self.temp_file_path, self.final_file_path)
            
            # 3. Remove o sinal de requisição (consomiu a requisição)
            if os.path.exists(self.vision_request_file):
                os.remove(self.vision_request_file)
            
            self.get_logger().info(f"Imagem capturada e salva: {self.final_file_path}")

        except Exception as e:
            self.get_logger().error(f"Falha ao salvar imagem: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Limpa arquivos ao sair
        if os.path.exists(node.final_file_path):
            os.remove(node.final_file_path)
        if os.path.exists(node.temp_file_path):
            os.remove(node.temp_file_path)
        if os.path.exists(node.vision_request_file):
            os.remove(node.vision_request_file)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

