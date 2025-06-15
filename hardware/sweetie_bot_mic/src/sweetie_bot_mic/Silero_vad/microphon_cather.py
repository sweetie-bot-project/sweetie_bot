import logging
import socket
import numpy as np
import speech_recognition as sr
import collections
import audioop
import torch
import threading
# import matplotlib.pyplot as plt
import struct

# Настройка форматирования логов
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Создаем форматтер
formatter = logging.Formatter('%(asctime)s.%(msecs)03d [cather] %(levelname)s: %(message)s', datefmt='%M:%S')

# Создаем обработчик для файла
file_handler = logging.FileHandler('cather.log')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# Отключаем вывод в консоль
logger.propagate = False

# Настройки сокета
HOST = '0.0.0.0'  # Адрес сервера
PORT = 1234        # Порт для TCP подключения

# Флаг для контроля работы потоков
running = True

# Длина циклического буфера в секундах
BUFFER_LENGTH = 5

# Циклический буфер для хранения аудио данных
buffer_size = 16000 * BUFFER_LENGTH  # 16000 - частота дискретизации
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
# voiced_confidences = []
# energies = []

# Функция для обработки аудио
def process_audio(data, client_socket):
    global audio_buffer

    audio_int16 = np.frombuffer(data, np.int16)
    audio_float32 = int2float(audio_int16)

    new_confidence = model(torch.from_numpy(audio_float32), 16000).item()
    # voiced_confidences.append(new_confidence)

    audio_buffer.extend(audio_int16)

    audio_data = np.array(audio_buffer, dtype=np.int16).tobytes()
    energy = audioop.rms(audio_data, 2)
    # energies.append(energy)

    # if len(voiced_confidences) > 100:
    #     voiced_confidences.pop(0)
        
    # if len(energies) > 100:
    #     energies.pop(0)

    # Отправляем уверенность клиенту
    
    
    try:
        client_socket.send(struct.pack('f', new_confidence))
    except:
        pass
    now = datetime.datetime.now()
    if new_confidence > 0.5:
        print(f"{now.strftime('%S.%f')[:-3]} Аудио обнаружено, <энергия>: {energy} <уверенность>: {new_confidence:.6f}")
    else:
        print(f"{now.strftime('%S.%f')[:-3]} ")
def handle_client(client_socket, client_address):
    global running
    try:
        print(f"Подключен клиент: {client_address}")
        while running:
            data = client_socket.recv(1024)
            if not data:
                break
            process_audio(data, client_socket)
    except Exception as e:
        print(f"Ошибка при обработке клиента: {e}")
    finally:
        client_socket.close()
        print(f"Клиент отключен: {client_address}")

def main():
    global running
    
    # Создание TCP сокета
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Слушаем на {HOST}:{PORT}")

    # # Настройка графика в главном потоке
    # plt.ion()
    # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    # line1, = ax1.plot([], [], 'b-', label='Уверенность в голосе')
    # ax1.set_ylim(0, 1)
    # ax1.set_title('Уверенность в наличии голоса')
    # ax1.legend()
    # ax2.set_title('Энергия аудио')
    # ax2.legend()

    try:
        while running:
            client_socket, client_address = server_socket.accept()
            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
            client_thread.start()

            # if len(voiced_confidences) > 0:
            #     line1.set_data(range(len(voiced_confidences)), voiced_confidences)
            #     ax1.relim()
            #     ax1.autoscale_view(scalex=True)
                
            # if len(energies) > 0:
            #     ax2.relim()
            #     ax2.autoscale_view()
                
            # plt.draw()
            # plt.pause(0.25)
            
    except KeyboardInterrupt:
        print("\nЗавершение работы...")
    finally:
        running = False
        server_socket.close()
        # plt.ioff()
        # plt.close()
        print("Программа завершена.")

if __name__ == "__main__":
    main()
