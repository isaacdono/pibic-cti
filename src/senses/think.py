#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from dotenv import load_dotenv
import base64
import time # Importar time

# LangChain
from langchain.agents import create_agent
from langgraph.checkpoint.memory import InMemorySaver
from langchain_google_genai import ChatGoogleGenerativeAI
from src.graph.tools import all_tools
from src.graph.tools.others import get_current_view

load_dotenv(override=True)

SYSTEM_PROMPT = """
Você é uma robô ROS2 inteligente chamada Sandra, parte do CTI Renato Archer.
Você recebe comandos de voz do usuário, processa-os e responde de forma adequada.
Use as ferramentas disponíveis para ajudá-la a responder.
Responda de forma concisa e direta, não use emojis.
"""

class Think_Node(Node):
    def __init__(self):
        super().__init__('think_node')

        # ROS
        self.sub_stt = self.create_subscription(String, '/transcript', self.callback_stt, 10)
        self.pub_tts = self.create_publisher(String, '/tts_command', 10)

        # LLM + agent
        self.llm = ChatGoogleGenerativeAI(model="gemini-2.5-pro")
        self.checkpointer = InMemorySaver()
        self.agent = create_agent(self.llm, all_tools, system_prompt=SYSTEM_PROMPT, checkpointer=self.checkpointer)

        self.get_logger().info("Think_Node iniciado com create_agent.")

    def callback_stt(self, msg: String):
        text = msg.data.strip()
        if not text: return
        self.get_logger().info(f"[CÉREBRO] Usuário: {text}")
        threading.Thread(target=self.process_input, args=(text,), daemon=True).start()

    def process_input(self, user_text: str):
        messages = [{"role": "user", "content": user_text}]

        try:
            # 1. Primeira Invocação
            result = self.agent.invoke(
                {"messages": messages},
                config={"configurable": {"thread_id": "main"}}
            )

            # 2. Verifica se a LLM pediu para usar a ferramenta de visão
            last_message = result["messages"][-1]
            if last_message.tool_calls and any(tc["name"] == "get_current_view" for tc in last_message.tool_calls):
                self.get_logger().info("Agente solicitou 'get_current_view'. Processando imagem multimodal...")
                
                # Passa um timestamp para evitar cache e força leitura de imagem fresca
                tool_input = {"cache_buster": str(time.time())}
                tool_result = get_current_view.invoke(tool_input)
                
                # Se a ferramenta retornou sucesso, a imagem já está em base64
                if isinstance(tool_result, dict) and tool_result.get("status") == "success":
                    try:
                        image_base64 = tool_result.get("image_base64")
                        
                        # IMPORTANTE: Primeiro responde à tool call com sucesso
                        tool_response = {
                            "type": "tool",
                            "name": "get_current_view",
                            "content": "Imagem capturada com sucesso.",
                            "tool_call_id": last_message.tool_calls[0]["id"],
                        }
                        
                        # Depois adiciona a imagem como uma nova mensagem do usuário
                        messages_with_tool_and_image = result["messages"] + [
                            tool_response,
                            {
                                "role": "user",
                                "content": [
                                    {"type": "text", "text": "Aqui está a imagem que você solicitou. Descreva o que você vê:"},
                                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}}
                                ]
                            }
                        ]
                        
                        self.get_logger().info(f"Imagem adicionada como mensagem do usuário ({len(image_base64)} chars)")
                        
                        result = self.agent.invoke(
                            {"messages": messages_with_tool_and_image},
                            config={"configurable": {"thread_id": "main"}}
                        )
                        
                    except Exception as e:
                        self.get_logger().error(f"Erro ao processar imagem: {e}")
                        # Em caso de erro, responde normalmente à tool call
                        tool_response = {
                            "type": "tool",
                            "name": "get_current_view",
                            "content": f"Erro ao processar imagem: {e}",
                            "tool_call_id": last_message.tool_calls[0]["id"],
                        }
                        messages_with_error = result["messages"] + [tool_response]
                        result = self.agent.invoke(
                            {"messages": messages_with_error},
                            config={"configurable": {"thread_id": "main"}}
                        )
                else:
                    # Se houve erro na captura, responde à tool call com erro
                    tool_response = {
                        "type": "tool",
                        "name": "get_current_view",
                        "content": tool_result.get("message", "Erro desconhecido ao capturar visão."),
                        "tool_call_id": last_message.tool_calls[0]["id"],
                    }
                    messages_with_error = result["messages"] + [tool_response]
                    result = self.agent.invoke(
                        {"messages": messages_with_error},
                        config={"configurable": {"thread_id": "main"}}
                    )

            # 4. Publica a resposta final
            ai_msgs = [m for m in result["messages"] if m.type == "ai"]
            reply_content = ai_msgs[-1].content if ai_msgs else "Desculpe, não consegui formar uma resposta."

            if isinstance(reply_content, list):
                text_parts = [part["text"] for part in reply_content if isinstance(part, dict) and part.get("type") == "text"]
                reply = " ".join(text_parts)
            else:
                reply = reply_content

            msg = String()
            msg.data = reply
            self.pub_tts.publish(msg)
            self.get_logger().info(f"[CÉREBRO] -> /tts_command: {reply}")

        except Exception as e:
            self.get_logger().error(f"Erro no agent.invoke: {e}")
            err = String(); err.data = "Desculpe, ocorreu um erro."; self.pub_tts.publish(err)


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
