#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from dotenv import load_dotenv

# LangChain
from langchain.agents import create_agent
from langgraph.checkpoint.memory import InMemorySaver
from langchain_google_genai import ChatGoogleGenerativeAI
from src.graph.tools import all_tools

load_dotenv(override=True)

SYSTEM_PROMPT = """
Você é uma robô inteligente chamada Sandra, parte do CTI Renato Archer.
Você recebe comandos de voz do usuário, processa-os e responde de forma adequada.
Use as ferramentas disponíveis para ajudá-la a responder.
Responda de forma concisa e direta, não use emojis
"""

class Think_Node(Node):
    def __init__(self):
        super().__init__('think_node')

        # ROS
        self.sub_stt = self.create_subscription(String, '/transcript', self.callback_stt, 10)
        self.pub_tts = self.create_publisher(String, '/tts_command', 10)

        # LLM + agent (create_agent com checkpointer)
        self.llm = ChatGoogleGenerativeAI(model="gemini-2.5-flash-lite")
        tools = all_tools
        self.checkpointer = InMemorySaver()
        self.agent = create_agent(self.llm, tools, system_prompt=SYSTEM_PROMPT, checkpointer=self.checkpointer)

        self.get_logger().info("Think_Node iniciado com create_agent.")

    def callback_stt(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"[CÉREBRO] Usuário: {text}")
        threading.Thread(target=self.process_input, args=(text,), daemon=True).start()

    def process_input(self, user_text: str):
        # monta mensagens
        messages = [{"role": "user", "content": user_text}]

        try:
            result = self.agent.invoke(
                {"messages": messages},
                config={"configurable": {"thread_id": "main"}}
            )

            # Corrige aqui: pega a resposta textual
            ai_msgs = [m for m in result["messages"] if m.type == "ai"]
            reply = ai_msgs[-1].content if ai_msgs else "Desculpe, não consegui formar uma resposta."

            msg = String()
            msg.data = reply
            self.pub_tts.publish(msg)
            self.get_logger().info(f"[CÉREBRO] -> /tts_command: {reply}")

        except Exception as e:
            self.get_logger().error(f"Erro no agent.invoke: {e}")
            err = String()
            err.data = "Desculpe, ocorreu um erro ao processar sua solicitação."
            self.pub_tts.publish(err)


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
