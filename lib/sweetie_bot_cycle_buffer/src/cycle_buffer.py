import pyaudio
import numpy as np
import speech_recognition as sr
import collections
import threading
import audioop

# Параметры аудио
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024

# Длина циклического буфера в секундах
BUFFER_LENGTH = 5
SILENCE_THRESHOLD = 2000  # Порог энергии для распознавания

# Создаем распознав и аудио-систему
recognizer = sr.Recognizer()
audio_interface = pyaudio.PyAudio()

# Циклический буфер для хранения аудио данных
buffer_size = RATE * BUFFER_LENGTH
audio_buffer = collections.deque(maxlen=buffer_size)

# Флаг для остановки записи
is_recording = True

# Функция для захвата аудио
def record_audio():
    global is_recording
    stream = audio_interface.open(format=FORMAT,
                                  channels=CHANNELS,
                                  rate=RATE,
                                  input=True,
                                  frames_per_buffer=CHUNK)

    while is_recording:
        data = stream.read(CHUNK, exception_on_overflow=False)
        audio_buffer.extend(np.frombuffer(data, dtype=np.int16))
    
    stream.stop_stream()
    stream.close()

# Функция для распознавания речи
def recognize_audio():
    global ising
    while is_recording:
        if len(audio_buffer) == buffer_size:
            audio_data = np.array(audio_buffer, dtype=np.int16).tobytes()

            # Вычисление RMS (корня среднего квадрата) энергии аудиосигнала
            energy = audioop.rms(audio_data, 2)
            
            # Проверка, превосходит ли энергия установленный порог
            if energy > SILENCE_THRESHOLD:
                print("Аудио обнаружено, энергия: ", energy)
                audio_segment = sr.AudioData(audio_data, RATE, 2)
                
                try:
                    text = recognizer.recognize_google(audio_segment, language="ru-RU")
                    print("Распознанный текст: ", text)
                except sr.UnknownValueError:
                    print("Не удалось распознать речь")
                except sr.RequestError as e:
                    print(f"Ошибка службы распознавания: {e}")
            else:
                print("Энергия недостаточна: ", energy)

# Запускаем запись и распознавание речи в отдельных потоках
recording_thread = threading.Thread(target=record_audio, name='record_audio_thred')
recognition_thread = threading.Thread(target=recognize_audio, name='recognize_audio_thred')

# Запуск потоков
recording_thread.start()
recognition_thread.start()
try:
    while True:
            #
            pass
except KeyboardInterrupt:
    # Останавливаем запись при нажатии Ctrl+C
    is_recording = False
    recording_thread.join()
    recognition_thread.join()
    audio_interface.terminate()
    print("Программа завершена.")
    
    

