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

class VoiceActivityDetectorSilero_vad(VoiceActivityDetector):
    ''' By silero_vad ROS topic message as speech indicator. '''
    _detector_type = 'silero_vad'

    def __init__(self, **kwargs):
        # listen: mic button is pressed
        self._sub_speech_indicator = rospy.Subscriber("mic_button", Bool, self._on_speech_indicator)
        self._speech_indicator = False
        rospy.loginfo(f"VAD type: silero_vad.")

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
        self._intesity = 0.0
        # debug plot
        self._debug_subplot = None
        self._plot_xlength = 10.0
        # node constructed
        rospy.loginfo(f'VAD type: sound intesity threshold.')

    def update(self, audio_data):
        self._intensity = np.sqrt(np.sum(audio_data.astype(np.float32)**2) / len(audio_data)) 
        if self._intensity > self._threshold:
            return VoiceActivity.VOICED, 1.0
        else:
            return VoiceActivity.UNVOICED, 1.0

    def debug_plots(self, t):
        ''' Return array of sweetie_bot_plot.msgs.Subplot object with debug information. '''
        if self._debug_subplot is None:
            from sweetie_bot_plot.msg import Plot, Subplot, Curve
            self._debug_subplot = Subplot(title = 'Speech by sound intensity', xlabel = 't', ylabel= 'intensity',
                                          curves = [ Curve(name = 'intensity', type = Curve.LINE_APPEND, xlength = self._plot_xlength, style='-', x = [ 0.0,], y = [0.0,]), 
                                                     Curve(name = 'threshold', type = Curve.LINE, x = [ -self._plot_xlength, 0.0], y = [0.0, 0.0]),  
                                                     Curve(name = 'speech flag', type = Curve.LINE_APPEND, xlength = self._plot_xlength, x = [ 0.0,], y = [0.0,]) ])
        self._debug_subplot.curves[0].x[0] = t
        self._debug_subplot.curves[0].y[0] = self._intensity
        self._debug_subplot.curves[1].x[:] = t - self._plot_xlength, t
        self._debug_subplot.curves[1].y[:] = self._threshold, self._threshold
        self._debug_subplot.curves[2].x[0] = t
        self._debug_subplot.curves[2].y[0] = 2 * self._threshold if (self._intensity > self._threshold) else 0.0
        return (self._debug_subplot,)


