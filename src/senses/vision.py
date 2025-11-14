#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import os

class VisionSaverNode(Node):
    def __init__(self):
        super().__init__('vision_saver_node_pil')
        self.subscription = self.create_subscription(
            Image, '/image', self.callback, 10)
        
        # Constrói o caminho para a pasta 'temp'
        script_dir = os.path.dirname(os.path.realpath(__file__))
        project_root = os.path.dirname(os.path.dirname(script_dir))
        output_dir = os.path.join(project_root, "temp")
        os.makedirs(output_dir, exist_ok=True)
        
        # Define o caminho do arquivo final e do arquivo temporário
        self.final_file_path = os.path.join(output_dir, "robot_latest_view.jpg")
        self.temp_file_path = os.path.join(output_dir, "robot_latest_view.jpg.tmp")
        
        self.get_logger().info(f"Salvando frames em: {self.final_file_path}")

    def callback(self, msg: Image):
        try:
            if msg.encoding != 'rgb8':
                self.get_logger().warn(f"Encoding inesperado: {msg.encoding}.")
                return

            pil_image = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
            flipped_image = pil_image.transpose(PILImage.FLIP_TOP_BOTTOM)
            
            # 1. Salva a imagem no arquivo temporário, especificando o formato
            flipped_image.save(self.temp_file_path, format='JPEG')
            
            # 2. Renomeia atomicamente o arquivo temporário para o final
            # Esta operação é instantânea e evita o bloqueio do arquivo.
            os.rename(self.temp_file_path, self.final_file_path)

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
        # Limpa ambos os arquivos ao sair, se existirem
        if os.path.exists(node.final_file_path):
            os.remove(node.final_file_path)
        if os.path.exists(node.temp_file_path):
            os.remove(node.temp_file_path)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()