#!/usr/bin/env python3
"""
Script para visualizar a arquitetura de nós ROS2 do projeto RAIA.
Gera um diagrama detalhado mostrando os dois fluxos principais:
1. Fluxo de Visão (Cenário 1): Percepção visual com LLM multimodal
2. Fluxo de Navegação (Cenário 2): Comando semântico para navegação
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle

# Configuração da figura
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 14))
fig.suptitle('Arquitetura ROS2 - Projeto RAIA (CTI Renato Archer)', fontsize=20, fontweight='bold')

# Cores por tipo de componente
COLOR_SENSOR = "#90EE90"      # Verde claro - Sensores
COLOR_PERCEPTION = "#87CEEB"   # Azul claro - Percepção
COLOR_COGNITION = "#FFD700"    # Dourado - Cognição
COLOR_TOOL = "#FFA07A"         # Salmão - Ferramentas
COLOR_ACTION = "#DDA0DD"       # Roxo claro - Atuação
COLOR_TOPIC = "#F0F0F0"        # Cinza claro - Tópicos
COLOR_EXTERNAL = "#FFB6C1"     # Rosa claro - Sistemas externos

def draw_node(ax, x, y, width, height, label, sublabel, color, style='round'):
    """Desenha um nó ROS2"""
    if style == 'round':
        box = FancyBboxPatch((x, y), width, height, boxstyle="round,pad=0.1",
                            facecolor=color, edgecolor='black', linewidth=2)
    else:
        box = FancyBboxPatch((x, y), width, height,
                            facecolor=color, edgecolor='black', linewidth=2)
    ax.add_patch(box)
    ax.text(x + width/2, y + height/2 + 0.15, label, 
           ha='center', va='center', fontsize=10, fontweight='bold')
    ax.text(x + width/2, y + height/2 - 0.15, sublabel, 
           ha='center', va='center', fontsize=8, style='italic')

def draw_topic(ax, x, y, width, height, label):
    """Desenha um tópico ROS2"""
    box = FancyBboxPatch((x, y), width, height, boxstyle="round,pad=0.05",
                        facecolor=COLOR_TOPIC, edgecolor='black', 
                        linewidth=1.5, linestyle='--')
    ax.add_patch(box)
    ax.text(x + width/2, y + height/2, label, 
           ha='center', va='center', fontsize=9, family='monospace')

def draw_arrow(ax, x1, y1, x2, y2, label='', color='black', style='solid', width=2):
    """Desenha uma seta conectando componentes"""
    arrow = FancyArrowPatch((x1, y1), (x2, y2),
                           arrowstyle='->', mutation_scale=20, 
                           linewidth=width, color=color, linestyle=style)
    ax.add_patch(arrow)
    if label:
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mid_x, mid_y, label, ha='center', va='bottom', 
               fontsize=7, color=color, fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

def draw_step_number(ax, x, y, number):
    """Desenha número de etapa do fluxo"""
    circle = Circle((x, y), 0.25, color='red', zorder=10)
    ax.add_patch(circle)
    ax.text(x, y, str(number), ha='center', va='center', 
           fontsize=10, fontweight='bold', color='white', zorder=11)

# ==================== CENÁRIO 1: FLUXO DE VISÃO ====================
ax1.set_title('Cenário 1: Descrição Visual do Ambiente\n("Sandra, me mostre o que você vê")', 
              fontsize=14, fontweight='bold', pad=20)
ax1.set_xlim(0, 10)
ax1.set_ylim(0, 12)
ax1.axis('off')

# Etapa 1: Percepção - Entrada de Áudio
draw_node(ax1, 0.5, 10.5, 1.8, 0.8, 'talk.py', 'STT_Node', COLOR_PERCEPTION)
draw_step_number(ax1, 0.4, 10.9, 1)

# Tópico /transcript
draw_topic(ax1, 3, 10.5, 1.5, 0.8, '/transcript')
draw_arrow(ax1, 2.3, 10.9, 3, 10.9, 'String\n(texto)', 'blue')

# Etapa 2: Cognição - LLM Agent
draw_node(ax1, 5, 9.5, 2, 1.5, 'think.py', 'Think_Node (LLM)', COLOR_COGNITION)
draw_step_number(ax1, 4.9, 11.2, 2)
draw_arrow(ax1, 4.5, 10.9, 5, 10.5, '', 'blue')

# Ferramentas do Agent
ax1.text(6, 10.5, '🔧 Tools:', fontsize=9, fontweight='bold')
ax1.text(6, 10.2, '• get_current_view', fontsize=7)
ax1.text(6, 9.95, '• calculate', fontsize=7)
ax1.text(6, 9.7, '• search_web', fontsize=7)

# Etapa 3: Execução de Ferramenta - Visão
draw_node(ax1, 7.5, 7.5, 2, 1, 'others.py', 'get_current_view', COLOR_TOOL)
draw_step_number(ax1, 7.4, 8.6, 3)
draw_arrow(ax1, 6, 9.5, 7.5, 8.3, 'invoke tool', 'orange', 'dashed')

# Sinal de requisição de visão
draw_topic(ax1, 7.5, 6, 2, 0.6, '/tmp/vision_request')
draw_arrow(ax1, 8.5, 7.5, 8.5, 6.6, 'cria sinal', 'orange', 'dotted', 1.5)

# Nó de visão (captura sob demanda)
draw_node(ax1, 5.5, 4.5, 2, 1, 'vision.py', 'VisionSaverNode', COLOR_SENSOR)
draw_arrow(ax1, 8.5, 6, 7.5, 5.3, 'detecta', 'green', 'dotted', 1.5)

# Tópico de câmera (externo - Gazebo/Hardware)
draw_node(ax1, 2, 4.5, 2.5, 1, 'ROS2 Camera', '/camera/color/\nimage_raw', COLOR_EXTERNAL)
draw_arrow(ax1, 4.5, 5, 5.5, 5, 'Image msg', 'green')

# Arquivo temporário com imagem
draw_topic(ax1, 7.5, 3.2, 2, 0.6, '/tmp/robot_latest_view.jpg')
draw_arrow(ax1, 6.5, 4.5, 7.8, 3.8, 'salva JPEG', 'green')

# Retorno da imagem para o tool
draw_arrow(ax1, 8.5, 3.8, 8.5, 7.5, 'base64', 'orange', 'dashed', 1.5)

# Etapa 4: Análise Multimodal
ax1.text(3.5, 7.5, '📸 Imagem + Texto', fontsize=9, fontweight='bold',
         bbox=dict(boxstyle='round,pad=0.4', facecolor='yellow', alpha=0.7))
draw_step_number(ax1, 3.2, 7.5, 4)
draw_arrow(ax1, 7.5, 8, 7, 8, '', 'orange', 'dashed')
draw_arrow(ax1, 5.5, 9.5, 4.5, 8, 'user msg\n+ image_url', 'purple', 'dashed', 2)

# LLM Multimodal (Gemini)
draw_node(ax1, 2, 6, 2, 1, 'Gemini 2.0', 'Vision Model', COLOR_EXTERNAL)
draw_arrow(ax1, 4, 7.3, 4, 7, '', 'purple', 'dotted', 1.5)
draw_arrow(ax1, 4, 6.5, 5, 9, 'descrição\nsemântica', 'purple', 'dotted', 1.5)

# Etapa 5: Síntese de Resposta
draw_topic(ax1, 5, 2, 1.5, 0.8, '/tts_command')
draw_step_number(ax1, 4.9, 2.9, 5)
draw_arrow(ax1, 6, 9.5, 5.7, 2.8, 'String\n(resposta)', 'red')

# Nó de síntese de voz
draw_node(ax1, 7.5, 1.2, 2, 1, 'speech.py', 'TTS_Node', COLOR_ACTION)
draw_arrow(ax1, 6.5, 2.4, 7.5, 2, '', 'red')

# Motor TTS (Kokoro)
draw_node(ax1, 7.5, 0, 2, 0.8, 'Kokoro TTS', 'Audio Output', COLOR_EXTERNAL)
draw_arrow(ax1, 8.5, 1.2, 8.5, 0.8, '🔊 áudio', 'red')

# Legenda Cenário 1
legend_y = -0.8
ax1.text(0.5, legend_y, '📊 Fluxo de Dados:', fontsize=10, fontweight='bold')
ax1.text(0.5, legend_y - 0.4, '1→ Entrada de áudio   2→ Análise LLM   3→ Captura imagem', fontsize=8)
ax1.text(0.5, legend_y - 0.7, '4→ Análise visual   5→ Resposta falada', fontsize=8)

# ==================== CENÁRIO 2: FLUXO DE NAVEGAÇÃO ====================
ax2.set_title('Cenário 2: Navegação via Comando Semântico\n("Vá até o meio do mapa")', 
              fontsize=14, fontweight='bold', pad=20)
ax2.set_xlim(0, 10)
ax2.set_ylim(0, 12)
ax2.axis('off')

# Etapa 1: Entrada de comando
draw_node(ax2, 0.5, 10.5, 1.8, 0.8, 'talk.py', 'STT_Node', COLOR_PERCEPTION)
draw_step_number(ax2, 0.4, 10.9, 1)

draw_topic(ax2, 3, 10.5, 1.5, 0.8, '/transcript')
draw_arrow(ax2, 2.3, 10.9, 3, 10.9, 'String', 'blue')

# Etapa 2: Processamento de Intenção
draw_node(ax2, 5, 9.5, 2, 1.5, 'think.py', 'Think_Node (LLM)', COLOR_COGNITION)
draw_step_number(ax2, 4.9, 11.2, 2)
draw_arrow(ax2, 4.5, 10.9, 5, 10.5, '', 'blue')

# Análise de intenção
ax2.text(6, 10.5, '🧠 Análise:', fontsize=9, fontweight='bold')
ax2.text(6, 10.2, '• Detecta navegação', fontsize=7)
ax2.text(6, 9.95, '• Mapeia semântica', fontsize=7)
ax2.text(6, 9.7, '  "meio"→(x,y)', fontsize=7)

# Etapa 3: Ferramenta de Navegação
draw_node(ax2, 7.5, 7.5, 2, 1.2, 'ros2.py', 'navigate_to_goal', COLOR_TOOL)
draw_step_number(ax2, 7.4, 8.8, 3)
draw_arrow(ax2, 6.5, 9.5, 7.5, 8.5, 'invoke tool\n(x, y, θ)', 'orange', 'dashed')

# Parâmetros da ferramenta
ax2.text(8.5, 7.9, 'Params:', fontsize=8, fontweight='bold')
ax2.text(8.5, 7.6, '• x: 0.0', fontsize=7, family='monospace')
ax2.text(8.5, 7.35, '• y: 0.0', fontsize=7, family='monospace')
ax2.text(8.5, 7.1, '• θ: 0.0', fontsize=7, family='monospace')

# Action Server Nav2
draw_node(ax2, 5.5, 5.5, 3, 1.2, 'Nav2 Stack', '/navigate_to_pose', COLOR_EXTERNAL)
draw_arrow(ax2, 8.5, 7.5, 7.5, 6.7, 'Goal msg', 'green', 'solid', 2)

# Componentes do Nav2
ax2.text(7, 6.1, '📍 Subsistemas Nav2:', fontsize=8, fontweight='bold')
ax2.text(7, 5.85, '• Planner Server', fontsize=7)
ax2.text(7, 5.65, '• Controller Server', fontsize=7)

# SLAM e Mapa
draw_node(ax2, 1, 5.5, 2.5, 1.2, 'SLAM/Mapping', 'map, /odom', COLOR_EXTERNAL)
draw_arrow(ax2, 3.5, 6.1, 5.5, 6.1, 'localização', 'purple', 'dotted')

# Etapa 4: Execução Motora - Gazebo
draw_node(ax2, 4, 3.5, 4, 1.5, 'Gazebo Simulator', 'TurtleBot3 + Ambiente', COLOR_EXTERNAL)
draw_step_number(ax2, 3.9, 5.1, 4)
draw_arrow(ax2, 7, 5.5, 6.5, 5, 'cmd_vel', 'green', 'solid', 2)

# Sensores do robô
ax2.text(6, 4.5, '🤖 Sensores:', fontsize=8, fontweight='bold')
ax2.text(6, 4.25, '• LiDAR → /scan', fontsize=7)
ax2.text(6, 4, '• Câmera → /camera/...', fontsize=7)
ax2.text(6, 3.75, '• Odometria → /odom', fontsize=7)

# Feedback para SLAM
draw_arrow(ax2, 4, 5, 2.5, 6, 'sensor data', 'purple', 'dotted', 1.5)

# Feedback para o usuário
draw_topic(ax2, 5, 1.8, 1.5, 0.8, '/tts_command')
draw_arrow(ax2, 6, 9.5, 5.7, 2.6, 'confirmação', 'red', 'dashed')

draw_node(ax2, 7.5, 1, 2, 1, 'speech.py', 'TTS_Node', COLOR_ACTION)
draw_arrow(ax2, 6.5, 2.2, 7.5, 1.8, '', 'red')

draw_node(ax2, 7.5, -0.2, 2, 0.8, 'Kokoro TTS', 'Audio Output', COLOR_EXTERNAL)
draw_arrow(ax2, 8.5, 1, 8.5, 0.6, '🔊 "Indo..."', 'red')

# Status da navegação
draw_topic(ax2, 1, 3, 2, 0.6, '/goal_status')
draw_arrow(ax2, 5.5, 3.5, 3, 3.3, 'feedback', 'orange', 'dotted', 1.5)
draw_arrow(ax2, 3, 3.3, 5, 9.5, 'atualização', 'orange', 'dotted', 1.5)

# Legenda Cenário 2
legend_y = -0.8
ax2.text(0.5, legend_y, '📊 Fluxo de Dados:', fontsize=10, fontweight='bold')
ax2.text(0.5, legend_y - 0.4, '1→ Comando voz   2→ Interpretação semântica   3→ Mapeamento (x,y)', fontsize=8)
ax2.text(0.5, legend_y - 0.7, '4→ Execução motora   Feedback contínuo: SLAM ↔ Nav2 ↔ Gazebo', fontsize=8)

# ==================== LEGENDA GLOBAL ====================
fig.text(0.5, 0.02, 'Legenda de Cores:', ha='center', fontsize=12, fontweight='bold')

legends = [
    mpatches.Patch(color=COLOR_SENSOR, label='Sensores (Entrada)'),
    mpatches.Patch(color=COLOR_PERCEPTION, label='Percepção (STT)'),
    mpatches.Patch(color=COLOR_COGNITION, label='Cognição (LLM+Agent)'),
    mpatches.Patch(color=COLOR_TOOL, label='Ferramentas (Tools)'),
    mpatches.Patch(color=COLOR_ACTION, label='Atuação (TTS)'),
    mpatches.Patch(color=COLOR_EXTERNAL, label='Sistemas Externos'),
    mpatches.Patch(color=COLOR_TOPIC, label='Tópicos/Arquivos', hatch='//'),
]

fig.legend(handles=legends, loc='lower center', ncol=7, fontsize=10, 
          frameon=True, fancybox=True, shadow=True, bbox_to_anchor=(0.5, -0.02))

plt.tight_layout(rect=[0, 0.05, 1, 0.98])
plt.savefig('/home/isaac/Documents/pibic-cti/projeto-raia/arquitetura_ros2_detalhada.png', 
            dpi=300, bbox_inches='tight', facecolor='white')
print("✅ Diagrama detalhado salvo em: arquitetura_ros2_detalhada.png")
print("📊 Mostrando os dois cenários principais:")
print("   1. Fluxo de Visão (Percepção Visual)")
print("   2. Fluxo de Navegação (Comando Semântico)")
plt.show()
