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
import datetime

# Logging formatting setup
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Create formatter
formatter = logging.Formatter('%(asctime)s.%(msecs)03d [cather] %(levelname)s: %(message)s', datefmt='%M:%S')

# Create file handler
file_handler = logging.FileHandler('cather.log')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# Disable console output
logger.propagate = False

# Socket settings
HOST = '0.0.0.0'  # Server address
PORT = 1234        # Port for TCP connection

# Flag to control thread operation
running = True

# Circular buffer length in seconds
BUFFER_LENGTH = 5

# Circular buffer for storing audio data
buffer_size = 16000 * BUFFER_LENGTH  # 16000 - sample rate
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

# For plotting
# voiced_confidences = []
# energies = []

# Function for processing audio
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

    # Send confidence to client
    
    
    try:
        client_socket.send(struct.pack('f', new_confidence))
    except:
        pass
    now = datetime.datetime.now()
    if new_confidence > 0.5:
        print(f"{now.strftime('%S.%f')[:-3]} Audio detected, <energy>: {energy} <confidence>: {new_confidence:.6f}")
    else:
        print(f"{now.strftime('%S.%f')[:-3]} ")
def handle_client(client_socket, client_address):
    global running
    try:
        print(f"Client connected: {client_address}")
        while running:
            data = client_socket.recv(1024)
            if not data:
                break
            process_audio(data, client_socket)
    except Exception as e:
        print(f"Error while handling client: {e}")
    finally:
        client_socket.close()
        print(f"Client disconnected: {client_address}")

def main():
    global running
    
    # Create TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Listening on {HOST}:{PORT}")

    # # Plot setup in main thread
    # plt.ion()
    # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    # line1, = ax1.plot([], [], 'b-', label='Voice confidence')
    # ax1.set_ylim(0, 1)
    # ax1.set_title('Voice presence confidence')
    # ax1.legend()
    # ax2.set_title('Audio energy')
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
        print("\nShutting down...")
    finally:
        running = False
        server_socket.close()
        # plt.ioff()
        # plt.close()
        print("Program terminated.")

if __name__ == "__main__":
    main()
