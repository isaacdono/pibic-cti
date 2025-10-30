#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from langchain_ollama import ChatOllama
from langchain_core.messages import SystemMessage, HumanMessage
import threading
from collections import deque

# --- Configurações ---
SYSTEM_PROMPT = os.getenv("ROBOT_SYSTEM_PROMPT", (
    "Você é Sandra, uma recepcionista virtual amigável e prestativa. "
    "Você percebe o ambiente ao seu redor e pode comentar sobre o que vê. "
    "Responda em português (pt-BR) de forma concisa e educada, sem emojis."
))
PROMPT_TEMPLATE = (
    "Contexto recente:\n{context}\n\n"
    "Instrução: Responda ao usuário abaixo, considerando o contexto e o que foi percebido pelo sistema de visão.\n\n"
    "Usuário: {user_input}\n\n"
    "Se for apenas cumprimento, responda curto. Se for algo relacionado à visão, comente brevemente o que viu."
)

MAX_HISTORY = 6

class Think_Node(Node):
    def __init__(self):
        super().__init__('think_node')

        # --- ROS Subscriptions / Publications ---
        self.subscription_stt = self.create_subscription(String, '/transcript', self.callback_stt, 10)
        self.subscription_vision = self.create_subscription(String, '/detected_objects', self.callback_vision, 10)
        self.publisher_tts = self.create_publisher(String, '/tts_command', 10)

        # --- Modelo ---
        self.llm = ChatOllama(model="gemma3:1b", temperature=0)
        self.get_logger().info("Nó Think iniciado — ouvindo /transcript e /detected_objects.")

        # --- Estado ---
        self.history = deque(maxlen=MAX_HISTORY * 2)
        self.lock = threading.Lock()
        self.last_seen_object = None

    # --- Callback: Fala do usuário ---
    def callback_stt(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return
        self.get_logger().info(f"[CÉREBRO] Usuário: {user_text}")
        threading.Thread(target=self.process_input, args=(user_text,), daemon=True).start()

    # --- Callback: Dados de visão ---
    def callback_vision(self, msg: String):
        detected = msg.data.strip()
        if not detected:
            return
        with self.lock:
            self.last_seen_object = detected
            self.history.append(("vision", f"O sistema de visão detectou: {detected}"))
        # self.get_logger().info(f"[VISÃO] Detectado: {detected}")

    # --- Constrói o contexto completo ---
    def build_context(self):
        parts = []
        for role, content in list(self.history):
            if role == "user":
                parts.append(f"Usuário: {content}")
            elif role == "assistant":
                parts.append(f"Sandra: {content}")
            elif role == "vision":
                parts.append(f"[Percepção Visual] {content}")
        return "\n".join(parts) if parts else "Nenhum contexto prévio."

    # --- Gera resposta LLM ---
    def process_input(self, text: str):
        with self.lock:
            self.history.append(("user", text))
            context = self.build_context()
            prompt = PROMPT_TEMPLATE.format(context=context, user_input=text)

        try:
            self.get_logger().info("Gerando resposta com LLM...")
            resp = self.llm.invoke([SystemMessage(content=SYSTEM_PROMPT), HumanMessage(content=prompt)])
            reply = getattr(resp, "content", str(resp)).strip()

            with self.lock:
                self.history.append(("assistant", reply))

            # Publica fala
            msg = String()
            msg.data = reply
            self.publisher_tts.publish(msg)
            self.get_logger().info(f"[CÉREBRO] -> /tts_command: {reply[:200]}")

        except Exception as e:
            self.get_logger().error(f"Erro no LLM: {e}")
            msg = String()
            msg.data = "Desculpe, ocorreu um erro ao processar sua solicitação."
            self.publisher_tts.publish(msg)

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
