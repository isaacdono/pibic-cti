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

SYSTEM_PROMPT = os.getenv(
    "ROBOT_SYSTEM_PROMPT",
    "Você é Sandra, uma recepcionista virtual amigável e prestativa. Responda em português (pt-BR) de forma concisa e educada. Não use emojis em suas repostas."
)

class Think_Node(Node):
    def __init__(self):
        super().__init__('think_node')

        # ROS
        self.sub_stt = self.create_subscription(String, '/transcript', self.callback_stt, 10)
        self.sub_vision = self.create_subscription(String, '/detected_objects', self.callback_vision, 10)
        self.pub_tts = self.create_publisher(String, '/tts_command', 10)

        # estado local
        self.lock = threading.Lock()
        self.last_seen_object = None


# import time
# from google.api_core.exceptions import ResourceExhausted

# def call_llm_safe(prompt, retries=5):
#     for attempt in range(retries):
#         try:
#             return llm.invoke(prompt)
#         except ResourceExhausted:
#             wait = 2 ** attempt
#             print(f"[WARN] LLM saturada, tentando novamente em {wait}s...")
#             time.sleep(wait)
#     print("[ERRO] Não foi possível contatar a LLM após várias tentativas.")
#     return None


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

    def callback_vision(self, msg: String):
        detected = msg.data.strip()
        if not detected:
            return
        with self.lock:
            self.last_seen_object = detected
        # self.get_logger().info(f"[VISÃO] {detected}")

    def process_input(self, user_text: str):
        # consome contexto de visão
        with self.lock:
            vision = self.last_seen_object
            self.last_seen_object = None

        # monta mensagens
        messages = []
        if vision:
            messages.append({"role": "user", "content": f"[Visão detectou: {vision}]\n\n{user_text}"})
        else:
            messages.append({"role": "user", "content": user_text})

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
            self.get_logger().info(f"[CÉREBRO] -> /tts_command: {reply[:200]}")

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
