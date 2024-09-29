import pyaudio
import numpy as np
import speech_recognition as sr
import collections
import audioop
import torch
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
import datetime

# Параметры аудио
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = int(RATE / 10)

# Длина циклического буфера в секундах
BUFFER_LENGTH = 5
SILENCE_THRESHOLD = 2000

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
# fig, ax = plt.subplots()
# line, = ax.plot([], [], lw=2)
# ax.set_ylim(0, 1)  # Установите в зависимости от диапазона значений confidence
# ax.set_xlim(0, 100)  # Установите в зависимости от максимального количества значений

# def update_plot(frame):  # Добавлен аргумент frame
#     if len(voiced_confidences) > 0:
#         line.set_ydata(voiced_confidences)
#         line.set_xdata(np.arange(len(voiced_confidences)))
#         ax.relim()
#         ax.autoscale_view()
#         fig.canvas.draw()  # Принудительное обновление графика
#         fig.canvas.flush_events()  # Обработка событий

# Установите save_count в None или в конкретное значение
# ani = animation.FuncAnimation(fig, update_plot, blit=False, save_count=100)

# plt.show(block=False)

# Функция для захвата и распознавания аудио
def process_audio():
    global audio_buffer
    stream = audio_interface.open(format=FORMAT,
                                  channels=CHANNELS,
                                  rate=RATE,
                                  input=True,
                                  frames_per_buffer=CHUNK)

    try:
        while True:
            audio_chunk = stream.read(512)
            audio_int16 = np.frombuffer(audio_chunk, np.int16)
            audio_float32 = int2float(audio_int16)

            new_confidence = model(torch.from_numpy(audio_float32), 16000).item()
            voiced_confidences.append(new_confidence)

            # Ограничиваем размер voiced_confidences
            if len(voiced_confidences) > 100:
                voiced_confidences.pop(0)
            
            # update_plot(0)  # Обновление графика, передаем 0 как аргумент
            
            # Добавляем данные в audio_buffer
            audio_buffer.extend(audio_int16)
            
            audio_data = np.array(audio_buffer, dtype=np.int16).tobytes()
            energy = audioop.rms(audio_data, 2)

            if new_confidence>0.5:
                print(f"Аудио обнаружено, <энергия>: {energy} <уверенность>: {new_confidence}")
                # audio_segment = sr.AudioData(audio_data, RATE, 2)
                # print(datetime.datetime.now())

                # try:
                #     text = recognizer.recognize_google(audio_segment, language="ru-RU")
                #     print("Распознанный текст: ", text)
                # except sr.UnknownValueError:
                #     print("Не удалось распознать речь")
                # except sr.RequestError as e:
                #     print(f"Ошибка службы распознавания: {e}")

            # update_p
            # lot(0)  # Обновление графика, передаем 0 как аргумент

    except KeyboardInterrupt:
        print("Остановка записи...")
    finally:
        stream.stop_stream()
        stream.close()
        audio_interface.terminate()
        print("Запись остановлена.")

# Запуск обработки аудио
process_audio()
