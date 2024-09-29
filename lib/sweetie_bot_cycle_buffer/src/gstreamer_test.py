import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Инициализация GStreamer
Gst.init(None)

# Создание конвейера
pipeline = Gst.parse_launch('udpsrc port=5004 ! application/x-rtp ! decodebin ! autovideosink')

# Запуск конвейера
pipeline.set_state(Gst.State.PLAYING)

# Ожидание завершения
bus = pipeline.get_bus()
bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)

# Остановка конвейера
pipeline.set_state(Gst.State.NULL)
