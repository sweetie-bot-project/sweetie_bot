import logging
import socket
import struct
import threading
import time

# Настройка форматирования логов
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Создаем форматтер
formatter = logging.Formatter('%(asctime)s.%(msecs)03d [gstrmr] %(levelname)s: %(message)s', datefmt='%M:%S')

# Создаем обработчик для файла
file_handler = logging.FileHandler('gstrmr.log')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# Отключаем вывод в консоль
logger.propagate = False

# Настройки сокета
HOST = '192.168.3.186'  # Адрес сервера
HOST = '127.0.0.1'
PORT = 1234             # Порт для TCP подключения
RECONNECT_DELAY = 5     # Задержка перед повторным подключением в секундах

# Глобальная переменная для сокета
sock = None
is_connected = False

def create_socket():
    """Создание нового сокета"""
    global sock
    try:
        if sock is not None:
            sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)  # Таймаут на операции сокета
        return True
    except Exception as e:
        logger.error(f"Ошибка при создании сокета: {e}")
        return False

def connect_to_server():
    """Подключение к серверу с повторными попытками"""
    global sock, is_connected
    while True:
        try:
            if not create_socket():
                time.sleep(RECONNECT_DELAY)
                continue

            logger.info(f"Попытка подключения к {HOST}:{PORT}")
            sock.connect((HOST, PORT))
            is_connected = True
            logger.info("Успешное подключение к серверу")
            return True
        except Exception as e:
            logger.error(f"Ошибка подключения: {e}")
            is_connected = False
            time.sleep(RECONNECT_DELAY)

def ensure_connection():
    """Проверка и восстановление соединения при необходимости"""
    global is_connected
    if not is_connected:
        return connect_to_server()
    return True

def send_data(data):
    """Отправка данных с обработкой ошибок"""
    global is_connected
    try:
        if not ensure_connection():
            return False
        sock.send(data)
        return True
    except Exception as e:
        logger.error(f"Ошибка при отправке данных: {e}")
        is_connected = False
        return False

def receive_confidence():
    """Получение значения confidence с обработкой ошибок"""
    global is_connected
    while True:
        try:
            if not ensure_connection():
                time.sleep(RECONNECT_DELAY)
                continue

            # Получаем 4 байта (размер float32)
            data = sock.recv(4)
            if not data:
                logger.warning("Получены пустые данные, переподключение...")
                is_connected = False
                continue

            # Распаковываем значение confidence
            confidence = struct.unpack('f', data)[0]
            return confidence
        except socket.timeout:
            logger.warning("Таймаут при получении данных")
            continue
        except Exception as e:
            logger.error(f"Ошибка при получении confidence: {e}")
            is_connected = False
            time.sleep(RECONNECT_DELAY)

try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst, GstApp, GObject
except ModuleNotFoundError as e:
    logger.error(f"Exception: {e}")
    logger.error(
        "In order to use GStreamer, you need to `pip install pipecat-ai[gstreamer]`. Also, you need to install GStreamer in your system."
    )
    raise Exception(f"Missing module: {e}")

def appsink_audio_new_sample(appsink: GstApp.AppSink):
    buffer = appsink.pull_sample().get_buffer()
    (_, info) = buffer.map(Gst.MapFlags.READ)
    
    try:
        # Отправляем аудио данные
        if send_data(info.data):
            # Получаем значение confidence
            confidence = receive_confidence()
            if confidence is not None:
                if confidence > 0.6:
                    logger.info(f"Обнаружен голос, confidence: {confidence:.6f}")
                elif confidence < 0.4:
                    logger.info(f"Нет голоса, confidence: {confidence:.6f}")
                else:
                    logger.info(f"Пауза, confidence: {confidence:.6f}")
            
    except Exception as e:
        logger.error(f"Ошибка при обработке данных: {e}")

    buffer.unmap(info)
    return Gst.FlowReturn.OK

# Initialize Gstreamer
Gst.init(None)

# Create pipeline
pipeline = Gst.Pipeline.new("rtp_pipeline")

# Create elements
udpsrc = Gst.ElementFactory.make("udpsrc", "udpsrc")
udpsrc.set_property("port", 5004)
udpsrc.set_property("caps", Gst.caps_from_string("application/x-rtp"))
queue = Gst.ElementFactory.make("queue", "queue")
queue.set_property("max-size-bytes", 1024)
rtppcmudepay = Gst.ElementFactory.make("rtppcmudepay", "rtppcmudepay")
mulawdec = Gst.ElementFactory.make("mulawdec", "mulawdec")
audioconvert = Gst.ElementFactory.make("audioconvert", "audioconvert")
audiobuffersplit = Gst.ElementFactory.make("audiobuffersplit", "audiobuffersplit")
audiobuffersplit.set_property("output-buffer-duration", Gst.Fraction(8, 125))
audiobuffersplit.set_property("strict-buffer-size", True)

appsink_audio = Gst.ElementFactory.make("appsink", None)
appsink_audio.set_property("emit-signals", True)
appsink_audio.set_property("sync", False)
appsink_audio.connect("new-sample", appsink_audio_new_sample)

# Add elements to pipeline
pipeline.add(udpsrc)
pipeline.add(queue)
pipeline.add(rtppcmudepay)
pipeline.add(mulawdec)
pipeline.add(audioconvert)
pipeline.add(audiobuffersplit)
pipeline.add(appsink_audio)

# Link elements
udpsrc.link(queue)
queue.link(rtppcmudepay)
rtppcmudepay.link(mulawdec)
mulawdec.link(audioconvert)
audioconvert.link(audiobuffersplit)
audiobuffersplit.link(appsink_audio)

# Start pipeline
pipeline.set_state(Gst.State.PLAYING)

# Start the GTK main loop
GObject.MainLoop().run()


