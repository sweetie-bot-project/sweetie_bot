import os
import pyaudio

# suppress error messages from ALSA
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
# https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i
@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield

class RespeakerAudio(object):

    def __init__(self, on_audio, mic_name = '', sample_rate = 16000, buffer_size = 1024, channels=None, suppress_error=True):
        self.on_audio = on_audio
        with ignore_stderr(enable=suppress_error):
            self.pyaudio = pyaudio.PyAudio()
        self.available_channels = None
        self.channels = channels
        self.device_index = None
        self.rate = sample_rate
        self.sample_width = 2
        self.bit_depth = 16

        # find device
        count = self.pyaudio.get_device_count()
        rospy.logdebug(f"{count} audio devices found")
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = info["name"]
            chan = info["maxInputChannels"]
            rospy.logdebug(f" - {i}: {name} ({chan} channels)")
            if name.startswith(mic_name):
                self.available_channels = chan
                self.device_index = i
                rospy.loginfo(f"Found pyaudio device id={i} name='{name}' (channels: {chan})")
                break

        if self.device_index is None:
            msg = f"Failed to find pyaudio device by name '{mic_name}'. Available devices:\n"
            for i in range(count):
                info = self.pyaudio.get_device_info_by_index(i)
                msg += f" - '{info['name']}' (input channels: {info['maxInputChannels']}, output channels: {info['maxOutputChannels']})\n"
            raise RuntimeError(msg)

        # selected channels
        if self.channels is None:
            self.channels = range(self.available_channels)
        elif any(c < 0 or c >= self.available_channels for c in self.channels):
            raise RuntimeError("Invalid channels {list(self.channels)}. (Available channels are {self.available_channels}).")
        rospy.loginfo(f"Using channels {list(self.channels)}")

        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=int(buffer_size),
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

    def __del__(self):
        try:
            self.stop()
            self.stream.close()
        except:
            pass
        finally:
            self.stream = None
        try:
            self.pyaudio.terminate()
        except:
            pass

    def stream_callback(self, in_data, frame_count, time_info, status):
        # split channel
        data = np.frombuffer(in_data, dtype=np.int16)
        chunk_per_channel = len(data) // self.available_channels
        data_channels = np.reshape(data, (chunk_per_channel, self.available_channels))
        # select channels
        selected_channels = data_channels[:, self.channels]
        # invoke callback
        self.on_audio(selected_channels)
        # continue function
        return None, pyaudio.paContinue

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if self.stream.is_active():
            self.stream.stop_stream()

