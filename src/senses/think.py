# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from langchain_ollama import ChatOllama  # você pode trocar o backend
from langchain_core.messages import HumanMessage
import threading


class Think_Node(Node):
    def __init__(self):
        super().__init__('think_node')

        # --- ROS Setup ---
        self.subscription = self.create_subscription(String, '/transcript', self.callback_stt, 10)
        self.publisher_ = self.create_publisher(String, '/tts_command', 10)

        # --- LLM Setup ---
        self.llm = ChatOllama(model="gemma3:1b")  # ou "mistral", "phi3", etc.
        self.get_logger().info("Nó Cérebro iniciado e ouvindo /transcript.")

    # Callback de fala (texto vindo do STT)
    def callback_stt(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return

        self.get_logger().info(f"[CÉREBRO] Usuário disse: {user_text}")
        threading.Thread(target=self.process_input, args=(user_text,), daemon=True).start()

    # Gera resposta com o LLM e publica
    def process_input(self, text: str):
        try:
            resposta = self.llm.invoke([HumanMessage(content=f"Você é um robô recepcionista. Responda em tom amigável: {text}")])
            reply = resposta.content.strip()
            msg = String()
            msg.data = reply
            self.publisher_.publish(msg)
            self.get_logger().info(f"[CÉREBRO] Respondeu: {reply}")
        except Exception as e:
            self.get_logger().error(f"Erro ao gerar resposta: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Think_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
