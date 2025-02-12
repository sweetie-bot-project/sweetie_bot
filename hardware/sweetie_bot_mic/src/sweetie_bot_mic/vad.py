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
        print(cls, cls._detector_type)
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
        result: 
            Classification result: 'silence', 'pause', 'speech'.
        speech_data: array_like
            Fragment with detected speech or None.
        speech_direction: KDL.Vector
            Direction of sound arrival.
        '''
        raise NotImplementedError

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
        return VoiceActivity.SILENCE, None, PyKDL.Vector()


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
            return VoiceActivity.VOICED, None, self.speech_direction
        else: 
            # button is not pressed, 
            if self._speeeh_indicator_on_previous_update == True:
                # end of speech episode 
                self._speeeh_indicator_on_previous_update = False
                return VoiceActivity.VOICED_END, self._speech_audio_buffer, self.speech_direction
            else:
                # reset state
                self._speech_audio_buffer.clear()
                self._reset_speech_direction()
                return VoiceActivity.SILENCE, None, PyKDL.Vector()

    def _on_speech_indicator(self, msg):
        self._speech_indicator = msg.data


