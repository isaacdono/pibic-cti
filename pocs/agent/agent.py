import os
import uuid
from typing import Annotated, Literal
from typing_extensions import TypedDict

# --- Dependências do LangChain ---
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.tools import tool
from langchain_core.messages import AnyMessage, HumanMessage, ToolMessage, AIMessage
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langgraph.graph import StateGraph, END
from langgraph.checkpoint.memory import MemorySaver
from langgraph.prebuilt import ToolNode

# --- CONFIGURAÇÃO DA API KEY ---
from dotenv import load_dotenv
load_dotenv(override=True)

# =============================================================================
# 1. FERRAMENTAS MOCKADAS (Simulando o Robô)
# =============================================================================

@tool
def tool_ros_speak(text: str) -> str:
    """
    Simula o robô falando. Use esta ferramenta para dar a resposta final ao usuário.
    :param text: O texto que o robô deve falar.
    """
    print(f"\n🤖 [ROBÔ FALA]: {text}\n")
    return "O robô terminou de falar."

@tool
def tool_ros_navigate(destination: str) -> str:
    """
    Simula o robô navegando para um destino (ex: 'sala 5', 'cozinha').
    :param destination: O nome do local para onde navegar.
    """
    print(f"\n🤖 [ROBÔ NAVEGA]: Iniciando navegação para {destination}...")
    return f"Navegação para {destination} iniciada com sucesso."

@tool
def tool_describe_scene() -> str:
    """
    Simula o VLM (Vision Language Model) descrevendo a cena atual.
    Use isso se o usuário perguntar o que o robô está vendo.
    """
    print(f"\n🤖 [VISÃO VLM]: Processando a cena atual...")
    # Resposta mockada de um VLM
    return "Eu estou vendo uma sala de estar com um sofá e uma pessoa sentada."

@tool
def tool_call_rag(query: str) -> str:
    """
    Simula uma busca no RAG (Base de Conhecimento) para responder perguntas
    factuais (ex: 'onde fica a sala 5?').
    :param query: A pergunta para a base de conhecimento.
    """
    print(f"\n🧠 [RAG]: Buscando por '{query}'...")
    knowledge_db = {
        "onde fica a sala 5": "A Sala 5 fica no segundo andar, ao lado da TI.",
        "qual o evento de hoje": "O 'Tech Demo Day' começa às 15h no auditório."
    }
    
    response = "Desculpe, não encontrei essa informação."
    for key in knowledge_db:
        if key in query.lower():
            response = knowledge_db[key]
            break
    return response

# =============================================================================
# 2. DEFINIÇÃO DO GRAFO (O Cérebro)
# =============================================================================

# --- O Estado da Memória ---
class AgentState(TypedDict):
    messages: Annotated[list[AnyMessage], "add_messages"]

# --- O PROMPT DE SISTEMA (A CORREÇÃO CRÍTICA) ---
# Este prompt FORÇA o LLM a não entrar em loop e a usar a ferramenta de fala.
system_prompt = """Você é o cérebro de um robô.
Seu objetivo é responder ao usuário. Você tem 4 ferramentas:

1.  `tool_call_rag`: Para buscar informações factuais (ex: "Onde fica...").
2.  `tool_describe_scene`: Para descrever o que você vê (ex: "O que você vê?").
3.  `tool_ros_navigate`: Para se mover (ex: "Vá para...").
4.  `tool_ros_speak`: Para falar com o usuário.

**REGRA MAIS IMPORTANTE:**
Após usar uma ferramenta de coleta de dados (RAG, VLM, Navegação), você DEVE usar a 
ferramenta `tool_ros_speak` para comunicar o resultado ao usuário. 
NÃO chame a mesma ferramenta de dados duas vezes.

Fluxo de exemplo:
Usuário: "O que você vê?"
Você: (Chama `tool_describe_scene`)
Tool: "Vejo um sofá."
Você: (Chama `tool_ros_speak` com o texto "Eu estou vendo um sofá.")
"""

# --- Configuração do LLM e Ferramentas ---

# CORREÇÃO: Passe o system_prompt DIRETAMENTE para o modelo aqui
llm = ChatGoogleGenerativeAI(
    model="gemini-2.5-flash", # Ou o modelo Gemini que você estiver usando
)

tools = [tool_ros_speak, tool_ros_navigate, tool_describe_scene, tool_call_rag]
tool_node = ToolNode(tools) # Nó que executa as ferramentas


# --- O Cérebro (Prompt + LLM + Ferramentas) ---

# CORREÇÃO: Simplifique o template do prompt.
# Ele só precisa lidar com o histórico de mensagens.
prompt = ChatPromptTemplate.from_messages(
    [
        MessagesPlaceholder(variable_name="messages"), # <-- SÓ ISSO
    ]
)

llm_with_tools = prompt | llm.bind_tools(tools)


# --- NÓS DO GRAFO ---

def llm_router_node(state: AgentState):
    """O nó principal que chama o LLM para decidir o que fazer."""
    print("--- \n🧠 [CÉREBRO]: Pensando...")
    response = llm_with_tools.invoke(state) # Passa o estado (com prompt)
    return {"messages": [response]}

def should_continue(state: AgentState) -> Literal["call_tool", "__end__"]:
    """Decide se o grafo deve chamar uma ferramenta ou se a conversa terminou."""
    last_message = state["messages"][-1]
    
    if isinstance(last_message, AIMessage) and last_message.tool_calls:
        print(f"🧠 [CÉREBRO]: Decidi usar: {[tc['name'] for tc in last_message.tool_calls]}")
        return "call_tool"
    
    # Se o LLM não chamou uma ferramenta, o prompt de sistema falhou
    # ou é uma conversa simples que não precisa de 'tool_ros_speak'.
    print("🧠 [CÉREBRO]: Decidi encerrar o turno (resposta final).")
    return "__end__"


# --- CONSTRUÇÃO DO GRAFO ---

def build_graph():
    workflow = StateGraph(AgentState)
    
    workflow.add_node("llm_router", llm_router_node)
    workflow.add_node("call_tool", tool_node)

    workflow.set_entry_point("llm_router")

    workflow.add_conditional_edges(
        "llm_router",
        should_continue,
        {
            "call_tool": "call_tool",
            "__end__": END
        }
    )
    workflow.add_edge("call_tool", "llm_router")

    memory = MemorySaver()
    print("✅ Cérebro (Grafo) compilado com sucesso!")
    return workflow.compile(checkpointer=memory)

# =============================================================================
# 3. SCRIPT DE TESTE (Execução)
# =============================================================================

if __name__ == "__main__":
    app = build_graph()
    
    convo_id = str(uuid.uuid4())
    config = {"configurable": {"thread_id": convo_id}}

    print("\n=======================================================")
    print("Cérebro do Robô Mockado (Versão Estável)")
    print("Fale com o robô. Digite 'sair' para fechar.")
    print("=======================================================")

    while True:
        user_input = input("🙂 [VOCÊ]: ")
        if user_input.lower() == "sair":
            print("🤖 [ROBÔ]: Desligando...")
            break
        
        # Envia a mensagem humana para o grafo
        events = app.stream(
            {"messages": [HumanMessage(content=user_input)]},
            config,
            stream_mode="values"
        )
        
        # O stream executa o loop (LLM -> Tool -> LLM -> ...)
        # As próprias ferramentas já imprimem suas ações
        for event in events:
            # Apenas monitoramos o evento final
            last_message = event["messages"][-1]
            if isinstance(last_message, AIMessage) and not last_message.tool_calls:
                # Se o LLM respondeu sem chamar o 'speak' (violando a regra)
                if last_message.content:
                     print(f"🤖 [ROBÔ FALA - fallback]: {last_message.content}")
                pass