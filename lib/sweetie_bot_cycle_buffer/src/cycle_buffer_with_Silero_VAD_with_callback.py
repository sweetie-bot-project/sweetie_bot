import pyaudio
import numpy as np
import speech_recognition as sr
import collections
import audioop
import torch
import threading

# Параметры аудио
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 512

# Длина циклического буфера в секундах
BUFFER_LENGTH = 5

# Создаем распознаватель и аудио-систему
recognizer = sr.Recognizer()
audio_interface = pyaudio.PyAudio()

# Циклический буфер для хранения аудио данных
buffer_size = RATE * BUFFER_LENGTH
audio_buffer = collections.deque(maxlen=buffer_size)

model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                              model='silero_vad',
                              force_reload=True)

def int2float(sound):
    abs_max = np.abs(sound).max()
    sound = sound.astype('float32')
    if abs_max > 0:
        sound *= 1 / 32768
    sound = sound.squeeze()
    return sound

# Для отрисовки на графике
voiced_confidences = []

# Callback функция для обработки аудио
def audio_callback(in_data, frame_count, time_info, status):
    global audio_buffer

    audio_int16 = np.frombuffer(in_data, np.int16)
    audio_float32 = int2float(audio_int16)

    new_confidence = model(torch.from_numpy(audio_float32), 16000).item()
    voiced_confidences.append(new_confidence)

    # Ограничиваем размер voiced_confidences
    if len(voiced_confidences) > 100:
        voiced_confidences.pop(0)

    # Добавляем данные в audio_buffer
    audio_buffer.extend(audio_int16)

    audio_data = np.array(audio_buffer, dtype=np.int16).tobytes()
    energy = audioop.rms(audio_data, 2)

    if new_confidence > 0.5:
        print(f"Аудио обнаружено, <энергия>: {energy} <уверенность>: {new_confidence}")
        # Здесь можно добавить код для распознавания речи

    return (None, pyaudio.paContinue)

# Открываем поток с использованием callback
stream = audio_interface.open(format=FORMAT,
                               channels=CHANNELS,
                               rate=RATE,
                               input=True,
                               frames_per_buffer=CHUNK,
                               stream_callback=audio_callback)

# Запуск потока
stream.start_stream()

def run_stream():
    try:
        while True:
                # audio_segment = sr.AudioData(audio_data, RATE, 2)
                # print(datetime.datetime.now())

                # try:
                #     text = recognizer.recognize_google(audio_segment, language="ru-RU")
                #     print("Распознанный текст: ", text)
                # except sr.UnknownValueError:
                #     print("Не удалось распознать речь")
                # except sr.RequestError as e:
                #     print(f"Ошибка службы распознавания: {e}")
            pass  # Основной поток просто ждет
    except KeyboardInterrupt:
        print("Остановка записи...")

# Запускаем поток для обработки аудио
stream_thread = threading.Thread(target=run_stream)
stream_thread.start()

# Ждем завершения потока
stream_thread.join()

# Остановка потока
stream.stop_stream()
stream.close()
audio_interface.terminate()
print("Запись остановлена.")
