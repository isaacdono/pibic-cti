import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading
import queue

class TTSNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        self.subscription = self.create_subscription(
            String,
            '/tts_command',
            self.listener_callback,
            10)
        self.get_logger().info('Nó TTS (pyttsx3) iniciado. Aguardando comandos.')
        
        # Fila para desacoplar o callback do ROS da engine de fala (que é bloqueante)
        self.speak_queue = queue.Queue()
        
        # Inicia o loop de fala em um thread separado
        self.speak_thread = threading.Thread(target=self.speak_loop, daemon=True)
        self.speak_thread.start()

    def listener_callback(self, msg):
        # O callback é super rápido: apenas adiciona o texto à fila.
        self.get_logger().info(f'Recebido para falar: "{msg.data}"')
        self.speak_queue.put(msg.data)

    def speak_loop(self):
        # Inicializa o pyttsx3 DENTRO do thread
        engine = pyttsx3.init()
        # Tenta carregar vozes em PT-BR (se instaladas no sistema)
        try:
            voices = engine.getProperty('voices')
            pt_voice = next((v for v in voices if 'brazil' in v.id.lower() or 'pt_br' in v.id.lower()), None)
            if pt_voice:
                engine.setProperty('voice', pt_voice.id)
                self.get_logger().info(f'Voz PT-BR encontrada: {pt_voice.id}')
            else:
                self.get_logger().warn('Voz PT-BR não encontrada, usando padrão.')
        except Exception as e:
             self.get_logger().warn(f'Não foi possível carregar vozes: {e}')

        while rclpy.ok():
            # 'get()' é bloqueante, o thread dorme aqui até a fila ter itens
            text = self.speak_queue.get()
            
            if text is None: # Sinal para parar
                break
            
            # 'runAndWait()' é bloqueante, por isso está em um thread
            engine.say(text)
            engine.runAndWait()
            self.speak_queue.task_done()

    def destroy_node(self):
        self.speak_queue.put(None) # Envia sinal de parada para o thread
        self.speak_thread.join() # Espera o thread terminar
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node) # Mantém o nó vivo
    tts_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()