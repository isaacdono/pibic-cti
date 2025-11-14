# src/graph/tools/__init__.py

import inspect
import sys
from langchain_core.tools import BaseTool
from typing import List

# --- 1. Importe Explicitamente Seus Módulos de Ferramentas ---
# (Assim como o ROSA faz com 'from . import calculation, log, system')
#
# Adicione qualquer novo arquivo .py (ex: 'vision.py') a esta lista.
from . import calculation
from . import external
from . import log
from . import ros2
from . import others


# --- 2. Liste os Módulos Importados ---
# (Isso nos permite iterar sobre eles)
_tool_modules = [
    calculation,
    external,
    log,
    ros2,
    others,
]

def load_all_tools():
    """
    Itera sobre os módulos de ferramentas importados (como 'calculation', 'ros2')
    e coleta todas as funções decoradas com @tool.

    Este é o padrão usado pelo 'ROSA' e é muito mais robusto
    do que 'pkgutil' ou 'os.listdir'.
    """
    tools = []
    print("[Loader] Carregando ferramentas dos módulos importados...")

    for module in _tool_modules:
        try:
            # Itera sobre todos os atributos do módulo (funções, classes, etc.)
            for item_name in dir(module):
                # Pula atributos privados/mágicos
                if item_name.startswith("_"):
                    continue

                # Pega o objeto (a própria função/tool)
                obj = getattr(module, item_name)

                # Verifica se é uma LangChain Tool (decorada com @tool)
                if isinstance(obj, BaseTool):
                    tools.append(obj)
                    
        except Exception as e:
            print(f"[Loader-ERRO] Falha ao inspecionar o módulo {module.__name__}: {e}", file=sys.stderr)

    if not tools:
        print("[Loader-WARN] Nenhuma tool foi encontrada.")
    else:
        print(f"[Loader-INFO] {len(tools)} tools carregadas: {[t.name for t in tools]}")

    return tools

# --- 3. (Opcional) Pré-carregue as ferramentas ---
# Isso executa a função acima assim que 'graph.tools' é importado.
# O seu 'cerebro_node.py' pode agora simplesmente fazer:
# from graph.tools import all_tools
all_tools = load_all_tools()