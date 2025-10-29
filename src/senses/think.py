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

# --- Configurações (pode vir do .env se preferir) ---
SYSTEM_PROMPT = os.getenv("ROBOT_SYSTEM_PROMPT", (
    "Você é Sandra, uma recepcionista virtual amigável e prestativa. "
    "Responda de forma concisa em português (pt-BR), a menos que o usuário peça o contrário." \
    "Não use emojis em suas respostas."
))
PROMPT_TEMPLATE = (
    "Context:\n{context}\n\n"
    "Instruction: Responda ao usuário abaixo de forma educada e direta.\n\n"
    "User: {user_input}\n\n"
    "Se for apenas cumprimento, responda curto. Se precisar de ação, descreva qual ação e confirme."
)

MAX_HISTORY = 6  # número de trocas (user+assistant) a manter

class Think_Node(Node):
    def __init__(self):
        super().__init__('think_node')

        # ROS
        self.subscription = self.create_subscription(String, '/transcript', self.callback_stt, 10)
        self.publisher_ = self.create_publisher(String, '/tts_command', 10)

        # LLM (ajuste o modelo/backend conforme seu ambiente)
        self.llm = ChatOllama(model=os.getenv("OLLAMA_MODEL", "gemma3:1b"))
        self.get_logger().info("Nó Cérebro iniciado e ouvindo /transcript.")

        # Estado / contexto
        self.history = deque(maxlen=MAX_HISTORY * 2)  # guarda alternância user/assistant
        self.lock = threading.Lock()

    def callback_stt(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return

        self.get_logger().info(f"[CÉREBRO] Usuário: {user_text}")
        # processa em thread para não bloquear o callback ROS
        threading.Thread(target=self.process_input, args=(user_text,), daemon=True).start()

    def build_context(self):
        """Constrói um contexto resumido a partir do histórico"""
        # Junta entradas do histórico em linhas, mantendo ordem
        parts = []
        for role, content in list(self.history):
            prefix = "Usuário: " if role == "user" else "Assistente: "
            parts.append(f"{prefix}{content}")
        return "\n".join(parts) if parts else "Nenhum contexto prévio."

    def process_input(self, text: str):
        """Gera resposta com o LLM e publica em /tts_command"""
        with self.lock:
            # atualiza histórico com a entrada do usuário
            self.history.append(("user", text))

            # monta prompt com contexto
            context = self.build_context()
            prompt = PROMPT_TEMPLATE.format(context=context, user_input=text)

            # constrói mensagens para o LLM (system + conversa)
            messages = [SystemMessage(content=SYSTEM_PROMPT), HumanMessage(content=prompt)]

        # chama o LLM fora do lock (pode demorar)
        try:
            self.get_logger().info("Gerando resposta com LLM...")
            resp = self.llm.invoke(messages)  # mantém seu .invoke(...) atual
            raw = getattr(resp, "content", str(resp)).strip()
            # limpa e pega só a parte final (opcional)
            reply = raw

            # registra resposta no histórico
            with self.lock:
                self.history.append(("assistant", reply))

            # publica para TTS
            msg = String()
            msg.data = reply
            self.publisher_.publish(msg)
            self.get_logger().info(f"[CÉREBRO] Resposta publicada para /tts_command: {reply[:200]}")

        except Exception as e:
            self.get_logger().error(f"Erro ao gerar resposta: {e}")
            # feedback simples pro usuário
            err_msg = String()
            err_msg.data = "Desculpe, ocorreu um erro ao processar sua solicitação."
            self.publisher_.publish(err_msg)

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
