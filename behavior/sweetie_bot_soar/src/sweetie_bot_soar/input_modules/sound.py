from .input_module import InputModuleFlatSoarView, register
from .bins import BinsMap

from threading import Lock
import rospy

from sweetie_bot_text_msgs.msg import SoundEvent

class SoundSpeech(InputModuleFlatSoarView):
    def __init__(self, name, config, agent):
        self._sound_event_sub = None
        # call supercalss constructor
        super(SoundSpeech, self).__init__(name, agent)
        # get configuration from parameters
        sound_event_topic = self.getConfigParameter(config, 'topic', allowed_types = (str,))
        self._speech_timeout = self.getConfigParameter(config, 'speech_timeout', default_value = 1.0)
        try:
            self._intensity_bins_map = BinsMap( config['intensity_bins_map'] )
        except KeyError:
            raise RuntimeError('sound_speech input module: "intensity_bins_map" parameter must present.')

        # subscriber    
        self._sound_event_sub = rospy.Subscriber(sound_event_topic, SoundEvent, self.newSoundEventCallback, queue_size = 10)
        # message buffers
        self._lock = Lock()
        self._last_sound_event = SoundEvent()
        self._text = None
        self._text_timestamp = rospy.Time.now()

    def newSoundEventCallback(self, msg):
        with self._lock:
            self._last_sound_event = msg
            # get text
            if self._text_timestamp < msg.header.stamp:
                if (msg.sound_flags & SoundEvent.SPEECH_DECODED) and msg.text != '':
                    self._text = (msg.text, msg.language)
                    self._text_timestamp = msg.header.stamp + rospy.Duration(self._speech_timeout)
                else:
                    self._text = None

    def update(self):
        with self._lock:
            # set status flags
            sound = bool(self._last_sound_event.sound_flags & SoundEvent.SOUND_DETECTING)
            self.updateChildWME('sound', sound) 
            speech = bool(self._last_sound_event.sound_flags & SoundEvent.SPEECH_DETECTING)
            self.updateChildWME('speech', speech) 
            # add sound intensity
            self.updateChildWME('intensity', self._intensity_bins_map(self._last_sound_event.intensity)) 
            # add text
            if self._text is not None:
                self.updateChildWME('text', self._text[0])
                self.updateChildWME('lang', self._text[1])
            else:
                self.removeChildWME('text')
                self.removeChildWME('lang')

    def __del__(self):
        # remove sensor wme and ROS subscriber
        if self._sound_event_sub:
            self._sound_event_sub.unregister()

register("sound_speech", SoundSpeech)
