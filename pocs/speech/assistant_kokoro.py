# -*- coding: utf-8 -*-
import os
import re
from time import time # Import 'time' from 'time' for timestamp
import warnings
import requests
import numpy as np
import sounddevice as sd
import soundfile as sf # Import a biblioteca soundfile
import speech_recognition as sr
from dotenv import load_dotenv

# --- CONFIGURAÇÃO INICIAL ---
# Força o caminho da DLL do eSpeak-NG para evitar erros no Windows
ESPEAK_DLL_PATH = "C:/Program Files/eSpeak NG/libespeak-ng.dll"
if os.path.exists(ESPEAK_DLL_PATH):
    os.environ['PHONEMIZER_ESPEAK_LIBRARY'] = ESPEAK_DLL_PATH

from kokoro import KPipeline

warnings.filterwarnings("ignore")
load_dotenv()

# --- PARÂMETROS GLOBAIS ---
# LLM (Groq)
GROQ_API_KEY = os.getenv("GROQ_API_KEY", "").strip()
GROQ_MODEL = "llama-3.3-70b-versatile"
SYSTEM_MSG = (
    "You are Rosana, a friendly and helpful virtual assistant. "
    "If you need to reason, place your reasoning between <think> and </think>. "
    "THE FINAL ANSWER MUST COME AFTER </think>, in plain text without any markers."
)

# TTS (Kokoro)
KOKORO_VOICE = "af_heart"

# STT (Whisper)
WHISPER_MODEL_SIZE = os.getenv("WHISPER_MODEL", "small")

# --- FUNÇÕES PRINCIPAIS ---

def listen(recognizer, source, whisper_model): # Alterado: recebe 'source' diretamente
    """Ouve o microfone e transcreve a fala para texto."""
    print("\n🎤 Ouvindo...")
    try:
        audio_data = recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0)
    except sr.WaitTimeoutError:
        return ""

    try:
        if whisper_model:
            wav_path = "captura_temporaria.wav"
            with open(wav_path, "wb") as f:
                f.write(audio_data.get_wav_data())

            segments, _ = whisper_model.transcribe(wav_path, language="en", vad_filter=True)
            transcribed_text = " ".join(s.text for s in (segments or [])).strip()
            return transcribed_text
            
        else:
            return recognizer.recognize_whisper(audio_data, model=WHISPER_MODEL_SIZE, language="en").strip()
        
    except (sr.UnknownValueError, Exception) as e:
        if not isinstance(e, sr.UnknownValueError):
            print(f"Erro na transcrição: {e}")
        return ""

def think_and_clean(prompt: str) -> tuple[str, str]:
    """Chama a API da Groq e limpa a resposta para o TTS."""
    headers = {"Authorization": f"Bearer {GROQ_API_KEY}"}
    payload = {
        "model": GROQ_MODEL,
        "messages": [{"role": "system", "content": SYSTEM_MSG}, {"role": "user", "content": prompt}],
        "temperature": 0.6, "top_p": 0.9, "max_tokens": 256
    }
    try:
        r = requests.post("https://api.groq.com/openai/v1/chat/completions", headers=headers, json=payload, timeout=15)
        r.raise_for_status()
        raw_text = r.json()["choices"][0]["message"]["content"]
    except requests.RequestException as e:
        print(f"Erro de conexão com a API: {e}")
        raw_text = "Desculpe, estou com problemas de conexão."

    # Lógica de limpeza integrada
    spoken_text = re.split(r"</think\s*>", raw_text, flags=re.IGNORECASE)[-1].strip()
    spoken_text = re.sub(r"[*#_`|/\[\]()]", "", spoken_text)
    spoken_text = re.sub(r"\s{2,}", " ", spoken_text).strip()
    
    return raw_text, spoken_text or "Não sei bem o que dizer sobre isso."

def speak(text: str, tts_pipeline):
    """Converte o texto limpo em áudio e o reproduz."""
    gen = tts_pipeline(text, voice=KOKORO_VOICE, speed=1.3)
    chunks = [audio for _, _, audio in gen]
    if chunks:
        audio = np.concatenate(chunks).astype("float32")
        try:
            sd.play(audio, 24000, blocking=False)
            sd.wait()
        except KeyboardInterrupt:
            print("\n🔇 Reprodução interrompida pelo usuário.")
            sd.stop()
            raise
        except Exception as e:
            print(f"Erro ao reproduzir áudio: {e}")
    else:
        print("⚠️ O TTS não gerou áudio para o texto.")

# --- BLOCO DE EXECUÇÃO PRINCIPAL ---

if __name__ == "__main__":
    # --- VERIFICAÇÃO E INICIALIZAÇÃO ---
    if not GROQ_API_KEY:
        print("❌ GROQ_API_KEY não configurada no arquivo .env. Encerrando.")
        exit()

    print("--- Iniciando Assistente de Voz ---")
    print("⏳ Carregando modelos de IA... (pode demorar na primeira vez)")

    # Inicializa TTS
    tts_pipeline = KPipeline(lang_code="a", repo_id="hexgrad/Kokoro-82M")
    print("✅ TTS (Kokoro) carregado.")

    # Inicializa STT
    whisper_model = None
    try:
        from faster_whisper import WhisperModel
        # CORREÇÃO: Alterado para 'float32' para maior compatibilidade de hardware.
        whisper_model = WhisperModel(WHISPER_MODEL_SIZE, device="auto", compute_type="float32")
        print(f"✅ STT (faster-whisper '{WHISPER_MODEL_SIZE}') carregado.")
    except Exception as e:
        print(f"⚠️ STT: faster-whisper não disponível, usando a versão padrão. Erro: {e}")

    # Calibra Microfone
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    with microphone as source:
        print("🔧 Calibrando microfone...")
        recognizer.adjust_for_ambient_noise(source, duration=3.0)
        recognizer.energy_threshold = max(recognizer.energy_threshold, 300)
        recognizer.energy_threshold = min(recognizer.energy_threshold, 500)
        print(f"✅ Microfone calibrado (limiar: {recognizer.energy_threshold:.0f}).")

        # --- LOOP PRINCIPAL ---
        print("\n✅ Assistente pronto! Diga 'sair' para encerrar.")
        try:
            while True:
                # Alterado: Passa 'source' diretamente para a função listen
                user_text = listen(recognizer, source, whisper_model)
                if not user_text:
                    continue
                
                print(f"Você: {user_text}")

                # Comandos de controle
                if user_text.lower().strip() in {"sair", "parar"}:
                    print("👋 Encerrando...")
                    speak("Até logo!", tts_pipeline)
                    break
                if "silêncio" in user_text.lower():
                    sd.stop()
                    print("🔇 Fala interrompida.")
                    continue

                # Ciclo principal: Pensar -> Falar
                raw_response, spoken_text = think_and_clean(user_text)
                # print(f"Rosana (bruto): {raw_response}")
                print(f"Rosana (fala): {spoken_text}")
                speak(spoken_text, tts_pipeline)

        except KeyboardInterrupt:
            print("\n👋 Encerrando (Ctrl+C).")
        finally:
            sd.stop()