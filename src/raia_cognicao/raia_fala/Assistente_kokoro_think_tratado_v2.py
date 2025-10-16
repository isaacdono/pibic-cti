

# -*- coding: utf-8 -*-
"""
Assistente de Voz (Groq + Whisper + Kokoro)
- Escuta o microfone, transcreve (STT), pergunta à LLM (Groq) e fala (Kokoro).
- Mantém <think> no log bruto, mas o TTS NUNCA fala <think>.
"""

import os
import re
import time
import threading
import warnings
import requests
import numpy as np
import soundfile as sf
import sounddevice as sd
import speech_recognition as sr
from dotenv import load_dotenv

# Define a variável de ambiente para que o phonemizer encontre a DLL (ainda necessário!)
os.environ['PHONEMIZER_ESPEAK_LIBRARY'] = "C:/Program Files/eSpeak NG/libespeak-ng.dll"

from kokoro import KPipeline


# ---------------------- Flags de comportamento ----------------------
KEEP_THINK_IN_LOG = True        # mostra o bruto com <think> no console
USE_THINK_IN_TTS = False        # não falar <think> (recomendado)
MAX_LEN_CHARS = 220             # limite pós-limpeza
FALLBACK_ENABLED = True

# Configura o caminho da DLL do eSpeak NG para o phonemizer
# os.environ['PHONEMIZER_ESPEAK_LIBRARY'] = "C:/Program Files/eSpeak NG/libespeak-ng.dll"

# ---------------------- Carregar .env ----------------------
load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY", "").strip()

# ---------------------- Config LLM (Groq) ----------------------
GROQ_URL = "https://api.groq.com/openai/v1/chat/completions"
# Id do modelo na Groq (use minúsculas e hífens)
# ou "llama-3.1-8b-instant" p/ menor latência
# "llama-3.3-70b-versatile"  # +"deepseek-r1-distill-llama-70b" "meta-llama/llama-4-scout-17b-16e-instruct"
GROQ_MODEL = "llama-3.1-8b-instant"

SYSTEM_MSG = (
    "Você é uma assistente virtual em português do Brasil. Muito sompática, educada e prestativa. "
    "Se precisar raciocinar, coloque apenas o raciocínio entre <think> e </think>. "
    "A RESPOSTA FINAL DEVE VIR APÓS </think>, em texto simples, sem marcadores nem símbolos (* # _ ` > | / [] ()). "
    "Nunca envie apenas <think> sem a resposta final. "
    "Responda em até 1000 palavras, com linguagem clara, simples e natural. "
    "llama-3.1-8b-instant"
    "Seu nome é Rosana"
)

# ---------------------- Config TTS (Kokoro) ----------------------
KOKORO_LANG = "a"          # pt-BR
KOKORO_VOICE = "af_heart"   # pf_dora | pm_alex | pm_santa
KOKORO_SPEED = 1.3
KOKORO_SR = 24000

# ---------------------- Config STT/listening ----------------------
LISTEN_TIMEOUT = 3.0
PHRASE_TIME_LIMIT = 5.0
PAUSE_THRESHOLD = 0.5
ENERGY_BOOST = 50  # suba para 400–600.

# ---------------------- Flags globais ----------------------
_stop_playback = threading.Event()
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=FutureWarning)

# ---------------------- Utils ----------------------


def _script_dir() -> str:
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return os.getcwd()


def extract_final_answer(text: str) -> str:
    """
    Escolhe o que o TTS deve falar:
    - Se existir </think>, usa tudo após o ÚLTIMO </think>.
    - Se existir <think sem fechar, retorna string vazia (para acionar fallback).
    - Caso não haja <think>, usa o texto inteiro.
    """
    parts = re.split(r"</think\s*>", text, flags=re.IGNORECASE)
    if len(parts) >= 2:
        candidate = parts[-1].strip()
        if candidate:
            return candidate

    if re.search(r"<think\b", text, flags=re.IGNORECASE):
        # só pensamento, sem resposta -> força fallback
        return ""

    return text.strip()


def normalize_for_tts(text: str) -> str:
    """Limpa símbolos e normaliza espaços/linhas. (o <think> já foi excluído nesta fase)"""
    text = re.sub(r"[*#_>`|/]", "", text)
    text = re.sub(r"[\[\]\(\)]", "", text)
    text = re.sub(r"(?m)^\s*[-•]\s+", "", text)
    text = re.sub(r"\s*\n\s*\n+", ". ", text)
    text = re.sub(r"\s*\n\s*", " ", text)
    text = re.sub(r"\s{2,}", " ", text).strip()

    if len(text) > MAX_LEN_CHARS:
        cut = text[:MAX_LEN_CHARS]
        if "." in cut[-40:]:
            cut = cut[: cut.rfind(".") + 1]
        elif " " in cut[-40:]:
            cut = cut[: cut.rfind(" ")]
        text = cut.strip()

    return text


def fallback_from_prompt(prompt: str) -> str:
    """Fallback curto e seguro, caso o modelo não envie resposta final fora do <think>."""
    p = prompt.lower()
    if re.search(r"\b(oi|olá|ola|bom dia|boa tarde|boa noite)\b", p):
        return "Olá! Como posso ajudar?"
    return "Certo. Pode especificar melhor para eu ajudar?"


def prepare_tts_text(raw: str, prompt: str) -> str:
    """Escolhe a parte falada respeitando as flags e aplica fallback se necessário."""
    text = raw
    if not USE_THINK_IN_TTS:
        text = extract_final_answer(text)

    text = normalize_for_tts(text)

    # Se ainda vier vazio ou sobrou <think> (por segurança), aplica fallback
    if (not text or "<think" in text.lower()) and FALLBACK_ENABLED:
        text = fallback_from_prompt(prompt)

    return text or "Tudo pronto por aqui."


def ask_groq(prompt: str):
    if not GROQ_API_KEY:
        return "Configure a GROQ_API_KEY no .env.", "Configure a GROQ_API_KEY no .env.", 0.0
    t0 = time.time()
    headers = {"Content-Type": "application/json",
               "Authorization": f"Bearer {GROQ_API_KEY}"}
    payload = {
        "model": GROQ_MODEL,
        "messages": [
            {"role": "system", "content": SYSTEM_MSG},
            {"role": "user", "content": prompt}
        ],
        "temperature": 0.6,
        "top_p": 0.9,
        "max_tokens": 256
    }
    r = requests.post(GROQ_URL, headers=headers, json=payload, timeout=30)
    r.raise_for_status()
    data = r.json()
    raw = data["choices"][0]["message"]["content"]
    spoken = prepare_tts_text(raw, prompt)
    return raw, spoken, time.time() - t0


# ---------------------- TTS (Kokoro) ----------------------
KOKORO_PIPELINE = KPipeline(lang_code=KOKORO_LANG,
                            repo_id="hexgrad/Kokoro-82M")


def _kokoro_generate_audio(text: str) -> np.ndarray:
    gen = KOKORO_PIPELINE(text, voice=KOKORO_VOICE,
                          speed=KOKORO_SPEED, split_pattern=r"\n+")
    chunks = [audio for _, _, audio in gen]
    if not chunks:
        return np.zeros((0,), dtype="float32")
    return np.concatenate(chunks).astype("float32")


def stop_speaking():
    _stop_playback.set()
    try:
        sd.stop()
    except Exception:
        pass


def speak_kokoro(text: str, save_wav: bool = False):
    audio = _kokoro_generate_audio(text)
    if audio.size == 0:
        print("⚠️ Kokoro não gerou áudio.")
        return
    if save_wav:
        out_path = os.path.join(_script_dir(), "resposta.wav")
        sf.write(out_path, audio, KOKORO_SR)
    _stop_playback.clear()
    try:
        sd.play(audio, KOKORO_SR, blocking=True)
    except Exception as e:
        print("Erro ao reproduzir áudio:", e)


# ---------------------- STT (Whisper) ----------------------
_use_faster = False
_faster_model = None


def _init_faster_whisper():
    global _use_faster, _faster_model
    try:
        from faster_whisper import WhisperModel
        model_size = os.getenv("WHISPER_MODEL", "small")
        device = "cuda" if os.getenv("FORCE_CPU", "0") != "1" else "cpu"
        compute_type = "float16" if device == "cuda" else "int8"
        _faster_model = WhisperModel(
            model_size, device=device, compute_type=compute_type)
        _use_faster = True
        print(
            f"🎙️ STT: faster-whisper carregado ({model_size}, {device}, {compute_type}).")
    except Exception as e:
        _use_faster = False
        _faster_model = None
        print("ℹ️ STT padrão (speech_recognition Whisper). Motivo:", e)


_init_faster_whisper()

_recognizer = sr.Recognizer()
_microphone = sr.Microphone()

with _microphone as source:
    print("🔧 Calibrando ruído ambiente (1s)...")
    _recognizer.dynamic_energy_threshold = True
    _recognizer.pause_threshold = PAUSE_THRESHOLD
    _recognizer.adjust_for_ambient_noise(source, duration=1.0)
    _recognizer.energy_threshold = max(
        _recognizer.energy_threshold, ENERGY_BOOST)
    print(
        f"✅ Calibração feita. energy_threshold={_recognizer.energy_threshold:.0f}")


def _stt_with_faster_whisper(audio_data: sr.AudioData) -> str:
    wav_path = os.path.join(_script_dir(), "captura.wav")
    try:
        with open(wav_path, "wb") as f:
            f.write(audio_data.get_wav_data())
    except PermissionError:
        alt_path = os.path.join(
            _script_dir(), f"captura_{int(time.time())}.wav")
        with open(alt_path, "wb") as f:
            f.write(audio_data.get_wav_data())
        wav_path = alt_path

    segments, _info = _faster_model.transcribe(
        wav_path, language="en", vad_filter=True, vad_parameters=dict(min_silence_duration_ms=300)
    )
    return " ".join(seg.text.strip() for seg in segments).strip()


def listen_once() -> str:
    with _microphone as source:
        print("🎤 Fale agora...")
        try:
            audio_data = _recognizer.listen(
                source, timeout=LISTEN_TIMEOUT, phrase_time_limit=PHRASE_TIME_LIMIT)
        except sr.WaitTimeoutError:
            return ""
    try:
        if _use_faster and _faster_model is not None:
            text = _stt_with_faster_whisper(audio_data)
        else:
            text = _recognizer.recognize_whisper(
                audio_data, model="small", language="en")
        return (text or "").strip()
    except sr.UnknownValueError:
        return ""
    except Exception as e:
        print("Erro no STT:", e)
        return ""

# ---------------------- Loop Principal ----------------------


def main():
    if not GROQ_API_KEY:
        print("❌ GROQ_API_KEY não encontrada. Configure no arquivo .env")
        return

    print("=== Assistente de Voz (Groq + Whisper + Kokoro) ===")
    print(
        "Dicas: diga '[SILENCIO]' para interromper; 'sair' ou 'parar' para encerrar.")

    try:
        while True:
            user_text = listen_once()
            if not user_text:
                continue

            low = user_text.lower().strip()
            print(f"Você: {user_text}")

            if "[silencio]" in low:
                stop_speaking()
                print("🔇 Silenciado.")
                continue
            if low in {"sair", "parar", "exit", "quit"}:
                print("👋 Encerrando...")
                break

            raw, spoken, t_llm = ask_groq(user_text)

            # Log: mostra bruto (com <think>) se desejar, e também o que será falado
            if KEEP_THINK_IN_LOG:
                print(f"Assistente bruto ({t_llm:.2f}s): {raw}")
            print(f"Assistente ({t_llm:.2f}s): {spoken}")

            speak_kokoro(spoken, save_wav=False)

    except KeyboardInterrupt:
        print("\n👋 Encerrando (Ctrl+C).")
    finally:
        stop_speaking()


if __name__ == "__main__":
    main()
