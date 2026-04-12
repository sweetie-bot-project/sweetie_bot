import socket
import numpy as np
import torch
import threading
import argparse
import struct

# constants
DATA_CHUNK_SIZE = 1024 #TODO extract from model

# parse command line arguments
parser = argparse.ArgumentParser(
                    prog='Silero VAD Server',
                    description='Provides access to the Silero VAD model via a TCP socket'
                )

parser.add_argument('-t', '--host', type=str, default='0.0.0.0', help='The host to listen on')
parser.add_argument('-p', '--port', type=int, default=1234, help='The port to listen on')
parser.add_argument('-c', '--model-cache-path', type=str,  help='The path to cache the Silero VAD model')
parser.add_argument('-f', '--force-model-download', action='store_true', help='Force the model to be downloaded')
parser.add_argument('-r', '--model-repo', type=str, default='snakers4/silero-vad', help='Repository with Silero VAD model')

args = parser.parse_args()

# load the model
if args.model_cache_path is not None:
    torch.hub.set_dir(args.model_cache_path)
model, _ = torch.hub.load(repo_or_dir=args.model_repo, model='silero_vad', source='github', 
                          force_reload=args.force_model_download, verbose=True)


def estimate_speech_confidence(data):
    # convert to float
    audio_int16 = np.frombuffer(data, np.int16)
    audio_float32 = audio_int16.astype(np.float32) / 32768
    # apply model
    confidence = model(torch.from_numpy(audio_float32), 16000).item()
    # logging
    # now = datetime.datetime.now()
    # print(f"{now.strftime('%S.%f')[:-3]} Speech <confidence>: {confidence:.6f}")
    # return result 
    return confidence

def recv_exact(sock, size):
    data = b''
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            # Connection closed or error
            raise ConnectionError("Connection lost while receiving data")
        data += chunk
    return data

def handle_client(client_socket, client_address):
    print(f"Client connected: {client_address}")
    try:
        while True:
            # receive data
            data = recv_exact(client_socket, DATA_CHUNK_SIZE)
            # estimate speech confidence
            confidence = estimate_speech_confidence(data)
            # send result
            client_socket.send(struct.pack('f', confidence))
    except (ConnectionError, TimeoutError) as e:
        print(f"Error while handling client: {e}")
    finally:
        client_socket.close()
        print(f"Client disconnected: {client_address}")

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((args.host, args.port))
    server_socket.listen(1)
    print(f"Listening on {args.host}:{args.port}")
    try:
        while True:
            client_socket, client_address = server_socket.accept()
            client_socket.settimeout(5.0)
            threading.Thread(target=handle_client, args=(client_socket, client_address)).start()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server_socket.close()
        print("Program terminated.")

if __name__ == "__main__":
    main()
