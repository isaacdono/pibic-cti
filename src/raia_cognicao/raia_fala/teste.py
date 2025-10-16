# teste_fonetica.py
import os


os.environ['PHONEMIZER_ESPEAK_LIBRARY'] = "C:/Program Files/eSpeak NG/libespeak-ng.dll"
print("--- Iniciando teste direto do Phonemizer ---")

try:
    # Importa as bibliotecas necessárias
    from phonemizer.backend import EspeakBackend

    # Tenta inicializar o backend do Espeak para português.
    # Esta é a camada que falha no seu script principal.
    print("Tentando inicializar o backend Espeak com language='pt'...")
    
    # Usamos 'pt' porque é o código de idioma padrão que o espeak espera.
    backend = EspeakBackend(language='pt')

    print("✅ Backend inicializado com sucesso!")

    # Se a linha acima funcionou, vamos tentar usar.
    texto = ['olá', 'mundo', 'teste', 'de', 'fonemas']
    phonemes = backend.phonemize(texto)
    
    print(f"✅ Resultado da fonemização: {phonemes}")
    print("\n--- Teste concluído com SUCESSO. ---")
    print("Se você vê esta mensagem, o 'phonemizer' e o 'espeak-ng' estão funcionando corretamente.")


except Exception as e:
    print("\n--- OCORREU UM ERRO NO TESTE DO PHONEMIZER: ---")
    print(f"Erro: {e}")
    print("\nIsso confirma que o problema está na comunicação entre a biblioteca 'phonemizer' e o 'espeak-ng' no seu ambiente.")