import rospy
import PyKDL
from enum import IntEnum
import numpy as np

from sweetie_bot_text_msgs.msg import SoundEvent
from std_msgs.msg import Bool

class VoiceActivity(IntEnum):
    SILENCE = 0,   # not speech fragment
    UNVOICED = 1,    # pause between voiced fragments
    VOICED = 2,  # voice fragment

class VoiceActivityDetector:
    ''' Voice activity detecton 

     Classification of speech fragment (SILENCE, VOICED, UNVOICED).

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
        pass
    
    def update(audio_data):
        ''' Update detector state.

        Arguments:
        ---------
        audio_data: array_like
            Next fragment of audio stream.

        Returns:
        -------
        result: VoiceActivity
            Classification result: VOICED, UNVOICED, SILENCE
        speech_probability: float
            Probablilty or likehood that fragment is VOICED.
        '''
        raise NotImplementedError

    def debug_plots(self, t):
        ''' Return array of sweetie_bot_plot.msgs.Subplot object with debug information.

        Arguments
        ---------
        t: float
            Current plot time.
        Returns
        -------
        result: tuple(sweetie_bot_plot.msg.Subplot)
            Subplot to add.
        '''
        return ()

class VoiceActivityDetectorNone(VoiceActivityDetector):
    ''' Always return that speech is not detected. '''
    _detector_type = 'none'

    def update(self, sound_event, audio_data):
        return VoiceActivity.SILENCE, 0.0

class VoiceActivityDetectorButton(VoiceActivityDetector):
    ''' External event as published on ROS topic message as speech indicator. '''
    _detector_type = 'button'

    def __init__(self, **kwargs):
        # listen: mic button is pressed
        self._sub_speech_indicator = rospy.Subscriber("mic_button", Bool, self._on_speech_indicator)
        self._speech_indicator = False
        rospy.loginfo(f"VAD type: button.")

    def update(self, audio_data):
        if self._speech_indicator:
            return VoiceActivity.VOICED, 1.0
        else: 
            return VoiceActivity.SILENCE, 0.0

    def _on_speech_indicator(self, msg):
        self._speech_indicator = msg.data


class VoiceActivityDetectorIntensity(VoiceActivityDetector):
    _detector_type = 'intensity'

    def __init__(self, **kwargs):
        self._threshold = kwargs.get('threshold')
        if not isinstance(self._threshold, (int,float)) or self._threshold <= 0.0:
            raise RuntimeError("VoiceActivityDetectorIntensity: 'threshold' parameter must present and be positive numeric value.")
        # detector state
        self._speech_indicator = False
        # node constructed
        rospy.loginfo(f'VAD type: sound intesity threshold.')

    def update(self, audio_data):
        intensity = np.sqrt(np.sum(audio_data.astype(np.float32)**2) / len(audio_data)) 
        if intensity > self._threshold:
            return VoiceActivity.VOICED, 1.0
        else:
            return VoiceActivity.UNVOICED, 1.0


