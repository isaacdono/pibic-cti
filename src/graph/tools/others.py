from langchain_core.tools import tool
import os
import time
import base64

@tool
def get_current_view(cache_buster: str) -> dict:
    """
    Solicita a captura de uma imagem, aguarda e retorna como base64.
    O argumento 'cache_buster' é usado para garantir que a imagem mais recente seja sempre lida.
    Retorna um dicionário com a imagem codificada em base64 e metadados para processamento multimodal.
    """
    
    path = "/tmp/robot_latest_view.jpg"
    signal_file = "/tmp/vision_request"
    
    # 1. Cria o sinal para que vision.py capture a próxima imagem
    try:
        with open(signal_file, "w") as f:
            f.write(str(time.time()))
    except Exception as e:
        return {
            "status": "error",
            "message": f"Erro ao criar sinal de visão: {e}"
        }
    
    # 2. Aguarda a imagem ser salva (com timeout de 2 segundos)
    timeout = 2.0
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        try:
            if os.path.exists(path) and os.path.getsize(path) > 0:
                # Aguarda um pouco para garantir que o arquivo foi escrito completamente
                time.sleep(0.1)
                
                with open(path, "rb") as f:
                    image_bytes = f.read()
                
                # Converte para base64
                image_base64 = base64.b64encode(image_bytes).decode("utf-8")
                
                return {
                    "status": "success",
                    "image_base64": image_base64,
                    "size": len(image_bytes),
                    "format": "JPEG",
                    "message": "Imagem capturada com sucesso."
                }
        except Exception as e:
            time.sleep(0.05)
            continue
    
    # 3. Se expirou o timeout, tenta ler mesmo assim
    try:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            with open(path, "rb") as f:
                image_bytes = f.read()
            image_base64 = base64.b64encode(image_bytes).decode("utf-8")
            return {
                "status": "success",
                "image_base64": image_base64,
                "size": len(image_bytes),
                "format": "JPEG",
                "message": "Imagem capturada (com timeout)."
            }
    except Exception:
        pass
            
    return {
        "status": "error",
        "message": f"Timeout: O sistema de visão não capturou a imagem em {timeout} segundos."
    }