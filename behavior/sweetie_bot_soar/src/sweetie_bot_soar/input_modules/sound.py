from .input_module import InputModuleFlatSoarView, register
from .bins import BinsMap
from ..nlp import SpacyInstance

from threading import Lock
import rospy

from sweetie_bot_text_msgs.msg import SoundEvent

class ParserMap:
    def __init__(self, nlp_model, map):
        self._nlp = SpacyInstance(nlp_model)
        # process map: construct key to words map
        try:
            self._map = {}
            for key, words in map.items():
                for word in words:
                    self._map[word.lower()] = key
        except (TypeError, AttributeError):
            raise TypeError('map: must be dict which maps strings to list of strings')

    def parse(self, text):
        tokens = self._nlp(text)
        # process parsed tokens and find key
        keys = []
        for token in tokens:
            key = self._map.get(token.lemma_)
            if key is not None:
                keys.append(key)
        return keys

class SoundSpeech(InputModuleFlatSoarView):
    def __init__(self, name, config, agent):
        self._sound_event_sub = None
        # call supercalss constructor
        super(SoundSpeech, self).__init__(name, agent)
        # get configuration from parameters
        sound_event_topic = self.getConfigParameter(config, 'topic', allowed_types = (str,))
        self._speech_timeout = self.getConfigParameter(config, 'speech_timeout', default_value = 1.0)
        self._lang_filter = self.getConfigParameter(config, 'lang_filter', 
            default_value = ['en', 'ru'], allowed_types = (list,), 
            check_func = lambda vals: all(isinstance(v, str) for v in vals) )
        try:
            self._intensity_bins_map = BinsMap( config['intensity_bins_map'] )
        except KeyError:
            raise RuntimeError('sound_speech input module: "intensity_bins_map" parameter must present.')
        # text analisys
        parsers = self.getConfigParameter(config, 'map_parsers', default_value = {}, allowed_types = dict)
        self._parsers = {}
        try:
            for lang, conf in parsers.items():
                self._parsers[lang] = ParserMap(**conf)
        except TypeError as e:
            raise KeyError('incorrect parser declaraition: missing or superfluous parameters (%s): error %s' % ([request.keys()], e))
        # subscriber    
        self._sound_event_sub = rospy.Subscriber(sound_event_topic, SoundEvent, self.newSoundEventCallback, queue_size = 10)
        # message buffers
        self._lock = Lock()
        self._last_sound_event = SoundEvent()
        self._text = None
        self._text_is_updated = False
        self._text_timestamp = rospy.Time.now()

    def newSoundEventCallback(self, msg):
        with self._lock:
            self._last_sound_event = msg
            if msg.sound_flags & SoundEvent.SPEECH_DECODED:
                rospy.logdebug('lang: receive msg: %s (%s)' % (msg.text, msg.language))
            # get text
            if self._text_timestamp < msg.header.stamp:
                if (msg.sound_flags & SoundEvent.SPEECH_DECODED) and msg.text != '':
                    rospy.logdebug('lang: stable message: %s (%s)' % (msg.text, msg.language))
                    self._text = (msg.text, msg.language)
                    self._text_timestamp = msg.header.stamp + rospy.Duration(self._speech_timeout)
                    self._text_is_updated = True
                else:
                    self._text_is_updated = self._text_is_updated or self._text is not None
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
            # update text if necessary
            if self._text_is_updated:
                self._text_is_updated = False
                if self._text is not None:
                    text, lang = self._text
                    rospy.logdebug('lang: speech detected: %s (%s)' % (text, lang))
                    # filter text by lang
                    if lang not in self._lang_filter:
                        return
                    # updtae text
                    self.updateChildWME('text', text)
                    self.updateChildWME('lang', lang)
                    # remove elements if present
                    while True:
                        wme_id = self._sensor_id.FindByAttribute('element', 0)
                        if wme_id is None:
                            break
                        wme_id.DestroyWME()
                    # parse and add elements WMEs
                    parser = self._parsers.get(lang)
                    if parser is not None:
                        elements = parser.parse(text)
                        # add WMEs
                        for elem in elements:
                            self._sensor_id.CreateStringWME('element', elem)
                else:
                    # remove WMEs
                    self.removeChildWME('text')
                    self.removeChildWME('lang')
                    # remove elements
                    while True:
                        wme_id = self._sensor_id.FindByAttribute('element', 0)
                        if wme_id is None:
                            break
                        wme_id.DestroyWME()

    def __del__(self):
        # remove sensor wme and ROS subscriber
        if self._sound_event_sub:
            self._sound_event_sub.unregister()

register("sound_speech", SoundSpeech)
