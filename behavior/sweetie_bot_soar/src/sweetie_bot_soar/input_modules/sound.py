from .input_module import InputModuleFlatSoarView, register
from .bins import BinsMap
from ..nlp import SpacyInstance
from .swm import SpatialWorldModel, ObjectKeyTuple

from threading import Lock
from bisect import insort
from dataclasses import dataclass

import numpy as np
from numpy.linalg import norm

import rospy
import tf
from flexbe_core.proxy import ProxyTransformListener

from sweetie_bot_text_msgs.msg import SoundEvent
from std_msgs.msg import Header

#
# NLP support
#

class Parser:
    subclass_map = {}

    def __new__(cls, type, **kwargs):
        # factory implementatipacy
        cls = Parser.subclass_map[type]
        return super(Parser, cls).__new__(cls)

    def __init__(cls, type):
        pass

class ParserLexicalTree(Parser):
    def __init__(self, nlp_model, **kwargs):
        super(ParserLexicalTree, self).__init__(**kwargs)
        self._nlp = SpacyInstance(nlp_model)

    def __call__(self, text, wme_id):
        # get tokens
        doc = self._nlp(text)
        # create token WMEs
        token_ids = []
        for token in doc:
            # add token
            token_id = wme_id.CreateIdWME('token')
            # set token attributes
            token_id.CreateStringWME('text', token.text)
            token_id.CreateStringWME('lemma', token.lemma_.lower())
            token_id.CreateStringWME('dep', token.dep_)
            if token.ent_type_ != '':
                token_id.CreateStringWME('entity-type', token.ent_type_)
            # store WME id
            token_ids.append(token_id)
        # add childrens to each token
        for token in doc:
            for child in token.children:
                token_ids[token.i].CreateSharedIdWME('child', token_ids[child.i])
        # add entities
        for entity in doc.ents:
            token_ids[entity.root.i].CreateStringWME('entity-full', entity.text)

Parser.subclass_map.update({'lexical_tree': ParserLexicalTree})

class ParserMap(Parser):
    def __init__(self, nlp_model, map, **kwargs):
        super(ParserMap, self).__init__(**kwargs)
        self._nlp = SpacyInstance(nlp_model)
        # process map: construct key to words map
        try:
            self._map = {}
            for key, words in map.items():
                for word in words:
                    self._map[word.lower()] = key
        except (TypeError, AttributeError):
            raise TypeError('map: must be dict which maps strings to list of strings')

    def __call__(self, text, wme_id):
        tokens = self._nlp(text)
        # process parsed tokens and find key
        for token in tokens:
            key = self._map.get(token.lemma_.lower())
            if key is not None:
                wme_id.CreateStringWME('element', key)

Parser.subclass_map.update({'word_map': ParserMap})

#
# speaker detection support
#

@dataclass(eq = True, frozen = True)
class EvaluationFrame:
    key : 'ObjectKeyTuple'
    intensity : float

#
# input module iplementation
#

class SoundSpeech(InputModuleFlatSoarView):
    WAITING = 0
    LISTENING = 1
    DISPLAYING = 2

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
        self._object_filter = self.getConfigParameter(config, 'object_filter', 
            default_value = ['human'], allowed_types = (list,), 
            check_func = lambda vals: all(isinstance(v, str) for v in vals) )
        self._object_max_n = self.getConfigParameter(config, 'object_max_n', default_value = 5, 
            allowed_types = (int,), check_func = lambda v: v >= 1)
        self._undetected_angle_tolerance = self.getConfigParameter(config, 'undetected_angle_tolerance', default_value = 0.5, 
            allowed_types = (float,), check_func = lambda v: v >= 0.0)
        self._undetected_intensity_accept_ratio = self.getConfigParameter(config, 'undetected_intensity_accept_ratio', default_value = 0.75, 
            allowed_types = (float,), check_func = lambda v: v >= 0.0 and v <= 1.0 )
        try:
            self._intensity_bins_map = BinsMap( config['intensity_bins_map'] )
        except KeyError:
            raise RuntimeError('sound_speech input module: "intensity_bins_map" parameter must present.')
        self._object_evaluation_alpha = self.getConfigParameter(config, 'object_evaluation_alpha', default_value = 0.85, 
            allowed_types = (float,), check_func = lambda v: v >= 0.0 and v <= 1.0 )
        # NLP analisys
        parsers = self.getConfigParameter(config, 'nlp_parsers', default_value = {}, allowed_types = dict)
        self._parsers = {}
        try:
            for lang, conf in parsers.items():
                self._parsers[lang] = Parser(**conf)
        except TypeError as e:
            raise KeyError('incorrect parser declaraition: missing or superfluous parameters (%s): error %s' % ([request.keys()], e))
        # subscriber    
        self._sound_event_sub = rospy.Subscriber(sound_event_topic, SoundEvent, self.newSoundEventCallback, queue_size = 10)
        self._tf_listener = ProxyTransformListener().listener()
        # message buffers
        self._lock = Lock()
        self._last_sound_event = SoundEvent()
        self._text = None
        self._text_is_updated = False
        self._text_timestamp = rospy.Time.now()
        self._state = SoundSpeech.WAITING
        # direction 
        self._object_intensity_map= {}

    def newSoundEventCallback(self, sound_event):
        # processing FSM
        while True:

            rospy.logdebug('sound: sound event processing: state: %s, msg: %s', self._state, sound_event)
            # print('sound: sound event processing: state: %s, msg: %s' % (self._state, sound_event))

            # waiting speech 
            if self._state == SoundSpeech.WAITING:
                
                # speech is heard, transit to LISTENING
                if sound_event.sound_flags & SoundEvent.SPEECH_DETECTING:
                    # clear evaluation map
                    self._object_intensity_map.clear()
                    # new state
                    self._state = SoundSpeech.LISTENING
                    continue

                # continue waiting speech
                break

            # listening to speech (text is not available yet)
            elif self._state == SoundSpeech.LISTENING:

                # no speech, transit to WAITING
                if not (sound_event.sound_flags & (SoundEvent.SPEECH_DETECTING | SoundEvent.SPEECH_DECODED)):
                    # new state 
                    self._state = SoundSpeech.WAITING
                    continue
                
                # possible speakers evaluation
                if (sound_event.sound_flags & SoundEvent.SPEECH_DETECTING):
                    # check if doa histogram is present in message
                    if len(sound_event.doa_values) > 0:
                        # evaluate possible speaker
                        swm = SpatialWorldModel.get_swm()
                        # get objects 
                        objects = swm.get_objects() 
                        objects = swm.get_objects(object_filter = lambda obj: obj.isVisible() and (obj.type in self._object_filter or (obj.type == 'speech' and obj.label == 'speech'))) 
                        if len(objects) > 0:
                            # last convertion from SWM frame to microphone frame as matrix44
                            sTw = self._tf_listener.asMatrix(sound_event.header.frame_id, Header(frame_id = swm.world_frame, stamp = rospy.Time()))
                            # compute object coordinates to microphone frame
                            obj_coords = np.stack([ (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, 0.0) for obj in objects.values() ], axis=1) # homogeneus coordintaes of objects in world frame (4, n_objects)
                            obj_coords = (sTw @ obj_coords)[0:3, :] # coordinates of objects in sound frame (3, n_objects)
                            # extract doa directions and values
                            doa_values = np.array(sound_event.doa_values)
                            doa_azimuth = sound_event.doa_azimuth
                            doa_colatitude = sound_event.doa_colatitude
                            if len(doa_colatitude) == 0:
                                # 2D problem: latitude is asumed to be zero
                                doa_directions = np.vstack( ( np.cos(doa_azimuth), np.sin(doa_azimuth), np.zeros(len(doa_azimuth))) ) # doa directions (3, n_doas)
                                # project object coordinates onto Oxy plane by setting z coordinate to zero
                                obj_coords[2, :] = 0
                            else:
                                # 3D problem: use colatitude values
                                doa_directions = np.vstack( ( np.cos(doa_azimuth)*np.sin(doa_colatitude), np.sin(doa_azimuth)*np.sin(doa_colatitude), np.cos(doa_colatitude) ) ) # doa directions (3, n_doas)
                            # find nearest direction for each object and corresponding doa value
                            obj_distances = np.linalg.norm(obj_coords, axis=0) # distance to objects (n_objects)
                            doa_obj_cos = doa_directions.T @ (obj_coords / obj_distances) # matrix of cos between sound doas and object directions (n_doas, n_objects)
                            obj_values = doa_values[ np.argmax(doa_obj_cos, axis=0) ] # intensity of object doas (n_objects)
                            # process object intesities
                            for k, (key_tuple, obj) in enumerate(objects.items()):
                                if key_tuple in self._object_intensity_map:
                                    self._object_intensity_map[key_tuple] = self._object_evaluation_alpha * self._object_intensity_map[key_tuple] + obj_values[k] * sound_event.intensity
                                else:
                                    self._object_intensity_map[key_tuple] = obj_values[k] * sound_event.intensity
                    #TODO: respeaker algorithm support

                elif sound_event.sound_flags & SoundEvent.SPEECH_DECODED and sound_event.text != '':
                    # speech is decoded, transit to DISPLAYING

                    # select max_n objects with maximal intensity
                    # and get speech detection which represents undetected speaker
                    obj_eval_frames = []
                    undetected_eval_frame = None
                    for key_tuple, intensity in self._object_intensity_map.items():
                        if (key_tuple.label, key_tuple.type) == ('speech', 'speech'):
                            undetected_eval_frame = EvaluationFrame(key_tuple, intensity)
                        else: 
                            insort(obj_eval_frames, EvaluationFrame(key_tuple, intensity), key=lambda p: -p.intensity)
                            del obj_eval_frames[self._object_max_n:]
                    # debug data
                    rospy.logdebug('sound: evaluated objects: %s, ubdetected: %s', obj_eval_frames, undetected_eval_frame)
                    # check if speaker is undetected
                    if undetected_eval_frame is not None:
                        # get corresponding objects
                        swm = SpatialWorldModel.get_swm()
                        objects = [ swm.get_object(eval_frame.key) for eval_frame in obj_eval_frames + [undetected_eval_frame] ] # (object_max_n + 1) objects
                        # last convertion from SWM frame to microphone frame as matrix44
                        sTw = self._tf_listener.asMatrix(sound_event.header.frame_id, Header(frame_id = swm.world_frame))
                        # convert objects coordinates to sound_event frame
                        obj_coords = np.stack([ (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, 0.0) for obj in objects ], axis=1) # homogeneus coordintaes of objects in world frame (4, n_objects)
                        obj_coords = (sTw @ obj_coords)[0:3, :] # coordinates of objects in sound frame (3, n_objects+1)
                        # for 2D DOA problem calulate angles in Oxy plane by setting z-coordinate to zero
                        if len(sound_event.doa_colatitude) == 0:
                            obj_coords[2, :] = 0 
                        # calculate directions
                        obj_directions = obj_coords / np.linalg.norm(obj_coords, axis=0) # distance to objects (n_objects+1)
                        # get cos beetween 
                        obj_cos = obj_directions[:,-1].T @ obj_directions[:,:-1] # cos between speech object (undetected speaker) and evaluated objects
                        # check angle_treshhold and intensity
                        intensity_threshold = undetected_eval_frame.intensity * self._undetected_intensity_accept_ratio
                        cos_threshold = np.cos(self._undetected_angle_tolerance)
                        if np.all( obj_cos < cos_threshold ) and all( eval_frame.intensity < intensity_threshold for eval_frame in obj_eval_frames ):
                            # add undetected to object list
                            insort(obj_eval_frames, undetected_eval_frame, key=lambda p: -p.intensity)
                            del obj_eval_frames[self._object_max_n:]
                        rospy.logdebug('sound: undetected apparaisal: intesity threshold: %f, cos threshold: %f, objects cos: %s', intensity_threshold, cos_threshold, obj_cos)
                    
                    # store speech decode result
                    with self._lock:
                        self._last_sound_event = sound_event
                        self._text = ( sound_event.text, sound_event.language, obj_eval_frames )
                        self._text_timestamp = rospy.Time.now() + rospy.Duration(self._speech_timeout)
                        # notify that update is needed
                        self._text_is_updated = True
                    # new state 
                    self._state = SoundSpeech.DISPLAYING
                    continue 
               
                break

            elif self._state == SoundSpeech.DISPLAYING:
                timestamp_now = rospy.Time.now()

                if self._text_timestamp < timestamp_now:
                    # update speech decode result
                    with self._lock:
                        self._last_sound_event = sound_event
                        self._text_is_updated = True
                        self._text = None
                    # new state
                    self._state = SoundSpeech.WAITING
                    continue

                break

        # update last message (if it was already updated nothin happens)
        self._last_sound_event = sound_event

    def remove_all_wme_by_attr(self, attr):
        while True:
            wme_id = self._sensor_id.FindByAttribute(attr, 0)
            if wme_id is None:
                return
            wme_id.DestroyWME()

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
                    text, lang, obj_eval_frames = self._text
                    rospy.loginfo('sound: speech detected: "%s" (%s), speech sources: %s', text, lang, obj_eval_frames)
                    # filter text by lang
                    if lang not in self._lang_filter:
                        return
                    # updtae text
                    self.updateChildWME('text', text)
                    self.updateChildWME('lang', lang)
                    # remove nlp results and speech sources
                    self.remove_all_wme_by_attr('nlp')
                    self.remove_all_wme_by_attr('speech-source')
                    self.remove_all_wme_by_attr('best-speech-source')
                    # NLP: apply parser
                    parser = self._parsers.get(lang)
                    if parser is not None:
                        nlp_id = self._sensor_id.CreateIdWME('nlp')
                        parser(text, nlp_id)
                    # add possible speech sources
                    swm = SpatialWorldModel.get_swm()
                    is_first = True
                    for eval_frame in obj_eval_frames:
                        soar_view = swm.get_object_soar_view(eval_frame.key)
                        if soar_view is not None:
                            source_id = self._sensor_id.CreateIdWME('speech-source')
                            source_id.CreateFloatWME('metric', eval_frame.intensity)
                            source_id.CreateSharedIdWME('swm-object', soar_view.wme_id)
                            # the first element is the best
                            if is_first:
                                self._sensor_id.CreateSharedIdWME('best-speech-source', source_id)
                                is_first = False
                else:
                    # remove WMEs
                    self.removeChildWME('text')
                    self.removeChildWME('lang')
                    # remove elements and speech sources
                    self.remove_all_wme_by_attr('nlp')
                    self.remove_all_wme_by_attr('speech-source')
                    self.remove_all_wme_by_attr('best-speech-source')

    def __del__(self):
        # remove sensor wme and ROS subscriber
        if self._sound_event_sub:
            self._sound_event_sub.unregister()

register("sound_speech", SoundSpeech)
