from langchain_core.tools import tool
import os
import time

@tool
def get_current_view() -> bytes | str:
    """
    Lê a última imagem capturada pelo robô do sistema de arquivos.
    Retorna os bytes da imagem em caso de sucesso, ou uma string de erro.
    """
    # Constrói o caminho dinamicamente para ser mais robusto
    # __file__ -> .../tools/others.py
    # dirname -> .../tools
    # dirname -> .../graph
    # dirname -> .../src
    # dirname -> .../projeto-raia
    try:
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
        path = os.path.join(project_root, "temp", "robot_latest_view.jpg")
    except NameError:
        # Fallback para quando __file__ não está definido (ex: alguns notebooks)
        path = "temp/robot_latest_view.jpg"

    # Tenta ler algumas vezes caso o arquivo esteja sendo escrito
    for _ in range(3):
        try:
            # Verifica se o arquivo existe e não está vazio
            if os.path.exists(path) and os.path.getsize(path) > 0:
                # Abre o arquivo em modo de leitura de bytes ('rb')
                with open(path, "rb") as f:
                    image_bytes = f.read()
                return image_bytes
        except Exception:
            # Se houver um erro (ex: arquivo bloqueado), espera um pouco e tenta de novo
            time.sleep(0.05)
            
    return f"Erro: O sistema de visão não está salvando imagens ou o arquivo '{path}' não pôde ser lido."