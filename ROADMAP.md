## ROADMAP 

### Meta
Robô navega, reage a pessoas e **descreve o que vê** (VLM) sob demanda.

### Utilidades
https://docs.langchain.com/oss/python/integrations/chat
source /opt/ros/jazzy/setup.bash
https://console.groq.com/docs/model/meta-llama/llama-4-scout-17b-16e-instruct 

### Planejamento

1.  **Cérebro (LangGraph Puro, Mockado):**
    * Criar `robot_agent` (grafo LangGraph, RAG, APIs).
    * **Testar lógica de IA em terminal de texto.**
    * *Mock (Simulação) das Ferramentas/Eventos ROS:*
        * `tool_ros_speak(text)` $\rightarrow$ `print(f"[FALA]: {text}")`
        * `tool_ros_navigate(dest)` $\rightarrow$ `print(f"[NAVEGA]: Indo para {dest}")`
        * `on_face_detected(id)` (Evento) $\rightarrow$ `print(f"[VISÃO-RÁPIDA]: Vi a pessoa {id}")`
        * `tool_describe_scene()` (VLM) $\rightarrow$ `print(f"[VISÃO-VLM]: Estou vendo uma sala...")`

2.  **Sentidos Auditivos (STT/TTS Reais):**
    * Criar `robot_speech` (nós reais de STT e TTS).
    * **Meta:** Testar o ciclo de áudio (Falar $\rightarrow$ `/transcript` $\rightarrow$ `/tts_command` $\rightarrow$ Ouvir).

3.  **Corpo e Sentidos Visuais (Simulação + Percepção):**
    * Criar a cena no CoppeliaSim (com câmera), o URDF e a *bridge* (`robot_sim`).
    * Configurar o Nav2 (`robot_bringup`).
    * Criar `robot_perception` contendo **dois nós de visão**:
        1.  `face_detector_node`: Nó leve (OpenCV) que publica no tópico `/detected_faces` (para reações rápidas).
        2.  `vlm_node`: Nó pesado (LLaVA, etc.) que expõe um **Serviço ROS 2** (ex: `/describe_scene`).
    * **Meta:** 1) Robô navega via comando manual do RViz. 2) O tópico `/detected_faces` funciona. 3) O serviço `/describe_scene` responde com uma descrição quando chamado manualmente (`ros2 service call...`).

4.  **Integração (Cérebro $\leftrightarrow$ Sentidos):**
    * Conectar o `agent_node` (Passo 1) aos `robot_speech` (Passo 2) e `robot_perception` (Passo 3).
    * **Audição:** Substituir o `print()` da `tool_ros_speak` pela publicação real no `/tts_command` e subscrever ao `/transcript`.
    * **Visão Passiva:** O `agent_node` subscreve ao `/detected_faces` para reações (substituindo o mock `on_face_detected`).
    * **Visão Ativa (VLM):** Substituir o `print()` da `tool_describe_scene` por um **Cliente de Serviço ROS 2** que chama o serviço `/describe_scene`.
    * **Meta:** Robô reage a rostos E responde por áudio a perguntas como "O que você está vendo?".

5.  **Integração Final (Cérebro $\rightarrow$ Corpo):**
    * Conectar o `agent_node` (Passo 1) ao Nav2 (Passo 3).
    * Substituir o `print()` da `tool_ros_navigate` pela chamada real ao *Action Server* `/navigate_to_pose`.
    * **Meta:** Atingir o objetivo final (Falar, ver, descrever e navegar no CoppeliaSim).