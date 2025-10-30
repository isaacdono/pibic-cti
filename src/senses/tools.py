from dotenv import load_dotenv 
from langchain_core.tools import tool
from langchain_community.utilities import GoogleSerperAPIWrapper

# --- Configuração da API ---
load_dotenv(override=True)

# --- Instanciação do wrapper ---
search_wrapper = GoogleSerperAPIWrapper()

# --- Definição da tool ---
@tool("web_search")
def google_serper_search(query: str) -> str:
    """Realiza uma pesquisa na internet usando o Serper e retorna informações relevantes resumidas."""
    try:
        result = search_wrapper.run(query)
        return result or "Nenhum resultado encontrado."
    except Exception as e:
        return f"Erro ao pesquisar: {e}"
