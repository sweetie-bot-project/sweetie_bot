import rospy
import PyKDL
from enum import IntEnum

from sweetie_bot_text_msgs.msg import SoundEvent
from std_msgs.msg import Bool

class VoiceActivity(IntEnum):
    SILENCE = 0,   # not speech fragment
    VOICED = 1,    # voice fragment
    UNVOICED = 2,  # pause between voiced fragments
    VOICED_END = 3,# end of speech (first SILENCE fragment after VOICED/UNVOICED fragments)

class VoiceActivityDetector:
    ''' Voice activity detecton 

    Base functions:
    * Classification of speech fragment (SILENCE, VOICED, UNVOICED, VOICED_END)
    * Agregate speech fragment into buffer for consequent translation to speech.
    * Avergae direction to speech source. Direction is provided in stationary frame.

    '''
    _subclass_map = {}

    def __new__(cls, type, **kwargs):
        cls = VoiceActivityDetector._subclass_map.get(type)
        if cls is None:
            raise ValueError(f"VoiceActivityDetector: unknown detector type: {type}")
        return super(VoiceActivityDetector, cls).__new__(cls)

    def __init_subclass__(cls, **kwargs):
        super(VoiceActivityDetector, cls).__init_subclass__(**kwargs)
        if cls._detector_type in cls._subclass_map:
            raise ValueError(f"VoiceActivityDetector: detector type '{cls._detector_type}' is already registered.")
        cls._subclass_map[cls._detector_type] = cls

    def __init__(self, type):
        self._speech_direction = PyKDL.Vector()
        self._speech_direction_intensity = 0.0
    
    def update(audio_data, doa_direction, intensity):
        ''' Update detector state.

        Arguments:
        ---------
        audio_data: array_like
            Next fragment of audio stream.
        doa_direction: KDL.Vector
            Direction of sound arrival. It is assumed that frame is stationary.
        intensity: float
            Sound intensity

        Returns:
        -------
        result: VoiceActivity
            Classification result: VOICED, UNVOICED, SILENCE, VOICED_END
        speech_probability: float
            Probablilty that fragment is VOICED.
        speech_data: array_like or None
            Data fragment with speech accumulated over VOICED/UNVOICED period. 
            it is not None only if result is VOICED_END.
        speech_direction: PyKDL.Vector
            Averaged direction to speech source.
        '''
        raise NotImplementedError

    def debug_plots(self):
        ''' Return array of sweetie_bot_plot.msgs.Subplot object with debug information. '''
        return ()

    def _average_speech_direction(self, doa_direction, intensity):
        ''' Average speech source direction using intensity as weight. '''
        self._speech_direction += doa_direction * intensity
        self._speech_direction_intensity += intensity

    def _reset_speech_direction(self):
        ''' Reset averaged direction. '''
        self._speech_direction = PyKDL.Vector()
        self._speech_direction_intensity = 0.0

    @property
    def speech_direction(self):
        ''' Get curernt averaged direction to speech source. '''
        if self._speech_direction_intensity > 0.0:
            return (1.0/self._speech_direction_intensity) * self._speech_direction
        else:
            return PyKDL.Vector()

class VoiceActivityDetectorNone(VoiceActivityDetector):
    _detector_type = 'none'

    def update(self, sound_event, audio_data):
        return VoiceActivity.SILENCE, 0.0, None, PyKDL.Vector()

class VoiceActivityDetectorButton(VoiceActivityDetector):
    _detector_type = 'button'

    def __init__(self, **kwargs):
        super(VoiceActivityDetectorButton, self).__init__(**kwargs)
        # listen: mic button is pressed
        self._sub_speech_indicator = rospy.Subscriber("mic_button", Bool, self._on_speech_indicator)
        self._speech_indicator = False
        self._speeeh_indicator_on_previous_update = False
        self._speech_audio_buffer = bytearray()
        rospy.loginfo(f"VAD type: button.")

    def update(self, audio_data, doa_direction, intensity):
        if self._speech_indicator:
            # button is pressed so we are hearing speech so add segment to audio buffer
            self._speech_audio_buffer.extend(audio_data.tobytes())
            self._average_speech_direction(doa_direction, intensity)
            self._speeeh_indicator_on_previous_update = True
            return VoiceActivity.VOICED, 1.0, None, self.speech_direction
        else: 
            # button is not pressed, 
            if self._speeeh_indicator_on_previous_update == True:
                # end of speech episode 
                self._speeeh_indicator_on_previous_update = False
                return VoiceActivity.VOICED_END, 0.0, self._speech_audio_buffer, self.speech_direction
            else:
                # reset state
                self._speech_audio_buffer.clear()
                self._reset_speech_direction()
                return VoiceActivity.SILENCE, 0.0, None, PyKDL.Vector()

    def _on_speech_indicator(self, msg):
        self._speech_indicator = msg.data

class VoiceActivityDetectorThresholdBase(VoiceActivityDetector):

    def __init__(self, threshold, timeout, frame_size, sample_rate, **kwargs):
        super(VoiceActivityDetectorTresholdBase, self).__init__(**kwargs)
        # get paramter
        if threshold < 0.0 or timeout < 0.0:
            raise ValueError('VoiceActivityDetectorTresholdBase: threshold and timeout paramters must be positive')
        self._threshold = threshold
        self._timeout = timeout
        if frame_size < 0 or frame_size != int(frame_size):
            raise ValueError('VoiceActivityDetectorTresholdBase: threshold and timeout paramters must be positive')
        self._frame_size = frame_size
        if sample_rate < 0:
            raise ValueError('VoiceActivityDetectorTresholdBase: sample_rate must be positive')
        self._sample_rate = sample_rate
        # Detector state
        self._state = VoiceActivity.SILENCE
        self._last_voiced_time = 0.0
        self._time = 0.0
        self._speech_buffer = bytearray()
        self._debug_subplot = None

    def update(self, audio_data, doa_direction, intensity):
        frame_size = self._frame_size
        # check data length
        n_frames, rem = divmod(len(audio_data), frame_size)
        if rem != 0:
            raise ValueError(f'VoiceActivityDetectorTresholdBase: length of audio data {len(audio_data)} must be divisible by frame size {frame_size}.')
        # process data
        voiced_end = False
        result_state = VoiceActivity.SILENCE
        result_value = 0.0
        for index in range(0, len(audio_data), frame_size):
            # process frame
            value = self._update(audio_data[index:index+frame_size])
            # evaluate result
            if self._state = VoiceActivity.SILENCE:
                # SILENCE state
                if len(self._speech_buffer) > 0:
                    self._speech_buffer.clear()
                    self._reset_speech_direction()
                if value > self._threshold:
                    self._state = VoiceActivity.VOICED
                    self._last_voiced_time = self._time
                    self._speech_buffer.extend(audio_data[index:index+frame_size])
            elif self._state == VoiceActivity.UNVOICED:
                # UNVOICED state
                if value > self._threshold:
                    self._state = VoiceActivity.VOICED
                    self._last_voiced_time = self._time
                else:
                    if (self._time - self._last_voiced_time) >= self._timeout:
                        self._state = VoiceActivity.SILENCE
                        voiced_end = True
                self._speech_buffer.extend(audio_data[index:index+frame_size])
            elif self._state == VoiceActivity.VOICED:
                # VOICED state
                if value > self._threshold:
                    self._last_voiced_time = self._time
                else:
                    self._state = VoiceActivity.UNVOICED
                self._speech_buffer.extend(audio_data[index:index+frame_size])
            # result state
            result_state = max(result_state, self._state)
            result_value += value
            # increase time
            self._time += self._frame_size / self._sample_rate
        # result value and durection estimate
        if result_state == VoiceActivity.SILENCE
            result_value = 0.0
        else:
            result_value /= n_frames
            self._average_speech_direction(doa_direction, intensity * result_value)
        # return result
        if voiced_end:
            return VoiceActivity.VOICED_END, 0.0, self._speech_buffer, self.speech_direction
        else:
            return result_state, result_value, None, self.speech_direction

