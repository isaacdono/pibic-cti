from langchain_core.tools import tool
import os
import time

@tool
def get_current_view(cache_buster: str) -> bytes | str:
    """
    Lê a última imagem capturada pelo robô do sistema de arquivos.
    O argumento 'cache_buster' é usado para garantir que a imagem mais recente seja sempre lida.
    Retorna os bytes da imagem em caso de sucesso, ou uma string de erro.
    """
    # O argumento cache_buster não é usado, mas sua presença com um valor
    # diferente a cada chamada (como um timestamp) impede o cache do resultado.
    
    path = "/tmp/robot_latest_view.jpg"

    # Tenta ler algumas vezes para evitar condição de corrida
    for _ in range(3):
        try:
            if os.path.exists(path) and os.path.getsize(path) > 0:
                with open(path, "rb") as f:
                    image_bytes = f.read()
                return image_bytes
        except Exception:
            time.sleep(0.05)
            
    return f"Erro: O sistema de visão não está salvando imagens ou o arquivo '{path}' não pôde ser lido."