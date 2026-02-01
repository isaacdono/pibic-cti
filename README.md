# Projeto de Iniciação Científica

Integração de Inteligência Artificial e Robótica Autônoma

Este é um projeto de iniciação científica brasileiro open-source focado no desenvolvimento de uma arquitetura cognitiva para robôs. O objetivo atual é validar um "cérebro" híbrido (IA Generativa + Sistemas de Controle Clássicos) capaz de operar tanto em ambientes simulados quanto em plataformas físicas.

Atualmente, o projeto foca no robô Sandra, um agente autônomo operando em ambiente de armazém (Warehouse) simulado, capaz de obedecer comandos de voz e realizar navegação semântica.

## Estado Atual do Projeto

O sistema encontra-se em fase de Validação de Arquitetura (POC).

- **Simulador:** Migrado para Gazebo Harmonic (Ambiente Tugbot Warehouse).
- **Middleware:** ROS 2 Jazzy Jalisco.
- **Navegação:** Nav2 + SLAM Toolbox integrados.
- **Cognição:** Agente ReAct implementado com LangChain e Google Gemini, capaz de usar ferramentas ROS 2.

## Tecnologias e Dependências

- **Linguagem:** Python 3.10+
- **Gerenciador de Pacotes:** uv (Rust-based)
- **ROS 2:** Jazzy
- **Simulação:** Gazebo Harmonic + ros_gz_bridge

## Instalação

Certifique-se de ter o ROS 2 Jazzy e o Gazebo Harmonic instalados no sistema.

1. Clone o repositório:

```bash
git clone <url-do-repositorio>
cd pibic-cti
```

2. Instale as dependências Python via uv:

```bash
# Instala o uv (caso não tenha)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Sincroniza o ambiente virtual com o lockfile
uv sync
```

3. Compile os pacotes ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

4. Configure as chaves de API: Crie um arquivo `.env` na raiz e adicione sua chave do Gemini:

```
GOOGLE_API_KEY="sua-chave-aqui"
```

## Como Executar (Passo a Passo)

A execução do sistema é **modular**. Você precisará de múltiplos terminais. Lembre-se de sempre ativar o ambiente uv, `source .venv/bin/activate`, e carregar o ambiente do ROS 2 (`source /opt/ros/jazzy/setup.bash`)em cada um.


### Terminal 1: RViz

Lança a interface de visualização, esteja no diretório `pocs/rviz`.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch simulation demo.launch.py
```

### Terminal 2: Simulacao (Gazebo)

Inicia o ambiente físico simulado e o robô Tugbot, esteja na raíz do repositório.

```bash
source /opt/ros/jazzy/setup.bash
gz sim -r pocs/rviz/resource/tugbot_warehouse.sdf
```

### Terminal 3: Localizacao e Mapeamento (SLAM)

Inicia o SLAM Toolbox para criar o mapa em tempo real ou localizar o robô.  
Preferencialmente na raíz do repositório.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

### Terminal 4: Stack de Navegacao (Nav2)

Carrega o planejador de trajetórias e os servidores de controle.  
Preferencialmente na raíz do repositório.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

### Terminal 5: Operador de Robo (Teleop)

Controla o robô manualmente para testes.  
Preferencialmente na raíz do repositório.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Contribuição e Licença

Este projeto é construído pela comunidade. Leia nosso **[Guia de Contribuição](./CONTRIBUTING.md)** para mais detalhes.

Distribuído sob a licença Apache 2.0. Veja o arquivo **[LICENSE](./LICENSE)** para mais detalhes.