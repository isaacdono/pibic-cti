#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardSTTNode(Node):
    """
    Nó ROS2 que lê o input do teclado e o publica no tópico /transcript.
    Simula a saída de um módulo de Speech-to-Text (STT).
    """
    def __init__(self):
        super().__init__('keyboard_stt_node')
        # Cria o publisher para enviar comandos para o nó de pensamento
        self.publisher_ = self.create_publisher(String, '/transcript', 10)
        self.get_logger().info('Pronto para enviar comandos. Digite "sair" para encerrar.')

    def send_input_loop(self):
        """
        Loop principal que lê o input do teclado e publica a mensagem.
        """
        while rclpy.ok():
            try:
                # Lê a entrada do usuário
                user_input = input("Diga algo para Sandra: ")

                if user_input.lower().strip() in ['sair', 'exit', 'quit']:
                    self.get_logger().info('Comando de saída recebido. Encerrando.')
                    break

                # Publica a mensagem
                msg = String()
                msg.data = user_input
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publicado no /transcript: "{user_input}"')

            except KeyboardInterrupt:
                self.get_logger().info('Encerrando loop de input.')
                break
            except EOFError:
                # Caso o terminal feche inesperadamente
                break
            except Exception as e:
                self.get_logger().error(f'Erro durante a publicação: {e}')
                break

def main(args=None):
    rclpy.init(args=args)
    
    # Cria a instância do nó
    node = KeyboardSTTNode()

    # Inicia o loop de input
    try:
        node.send_input_loop()
    except Exception as e:
        node.get_logger().error(f'Falha no nó principal: {e}')
    finally:
        # Garante que o nó e o ROS sejam desligados corretamente
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()