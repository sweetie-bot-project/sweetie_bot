#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, copy, re
import tempfile
import rospy, actionlib, roslib, rospkg
from threading import Event

from sweetie_bot_text_msgs.msg import TextCommand
from sweetie_bot_text_msgs.msg import TextActionAction as TextAction
from sweetie_bot_text_msgs.msg import TextActionGoal, TextActionFeedback, TextActionResult

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

import six
from google.cloud import translate_v2 as translate

soundhandle = SoundClient(blocking=True)
rospack = rospkg.RosPack()

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

voice_log = rospy.Publisher('voice_log', TextCommand)

class TTSInterface:
    def __init__(self, langs):
        self._langs = langs

    def is_lang_supported(self, lang):
        return lang in self._langs

    def speak(self, text, language):
        # voice log
        global voice_log
        voice_log.publish('log/voice/out/'+language, text, '')
        pass

class TTSCoquiAi(TTSInterface):
    tts = {}
    models = {}
    model_params = {}
    speak_params = {}

    def __init__(self, init_params, **kwargs):
        super(TTSCoquiAi, self).__init__(**kwargs)

        from TTS.api import TTS
        languages = init_params['languages']
        for language in languages:
            if 'params' in languages[language]:
                self.speak_params[language] = languages[language]['params']
            model_name = languages[language]['model']
            if not model_name in self.models:
                model_params = init_params['models'][model_name]
                self.models[model_name] = TTS(**model_params)
                self.model_params[model_name] = model_params
            self.tts[language] = self.models[model_name]

    def speak(self, text, language):
        super().speak(text, language)
        if not language in self.tts:
            rospy.logerr("No such language '%s'" % language)
            return False
        tts = self.tts[language]
        speak_params = self.speak_params[language] if language in self.speak_params else {}
        temp_dir = tempfile.mkdtemp()
        temp_file = os.path.join(temp_dir, "output.wav")
        text_params = { 'text':text, 'file_path':temp_file }
        speak_params = {**speak_params, **text_params}
        tts.tts_to_file(**speak_params)
        temp_file_con = ''
        if 'conversion' in self.tts:
            speak_params = self.speak_params['conversion']
            tts = self.tts['conversion']
            target_wav = os.path.join(rospack.get_path(__name__.split('.')[0]), speak_params['target_wav'])
            temp_file_con = os.path.join(temp_dir, "output_conversion.wav")
            input_params = {'source_wav':temp_file, 'target_wav':target_wav, 'file_path':temp_file_con}
            speak_params = {**speak_params, **input_params}
            tts.voice_conversion_to_file(**speak_params)
            soundhandle.playWave(temp_file_con)
            os.remove(temp_file_con)
        else:
            soundhandle.playWave(temp_file)

        os.remove(temp_file)
        os.rmdir(temp_dir)

        return True

class TTSSpeechDispatcher(TTSInterface):
    def __init__(self, module = 'rhvoice', **kwargs):
        super(TTSSpeechDispatcher, self).__init__(**kwargs)

        import speechd
        self._speechd = speechd
        self._client = speechd.SSIPClient('ros_speech_dispatcher_client')
        self._client.set_output_module(module)

    def speak(self, text, lang):
        super().speak(text, lang)
        ev = Event()
        self._client.set_language(lang)
        self._client.speak(text, callback = lambda cb_type: ev.set(), event_types=(self._speechd.CallbackType.CANCEL, self._speechd.CallbackType.END))
        ev.wait()
        return True

class TTSRhvoiceWrapper(TTSInterface):

    def __init__(self, rhvoice_params, gstreamer_pipeline_string = None, **kwargs):
        super(TTSRhvoiceWrapper, self).__init__(**kwargs)

        from rhvoice_wrapper import TTS
        # create and configure rhvoice client
        self._rhvoice = TTS(threads=2)
        self._rhvoice.set_params(**rhvoice_params)
        # create gstreamer pipeline and configure it
        pipeline_list = ['appsrc name=source ! audio/x-raw,format=S16LE,channels=1,rate=24000,layout=interleaved']
        if gstreamer_pipeline_string is not None:
            pipeline_list.append(gstreamer_pipeline_string)
        pipeline_list.append('autoaudiosink')
        self._gstreamer_pipeline = Gst.parse_launch(str.join(' ! ', pipeline_list))
        self._gstreamer_src = self._gstreamer_pipeline.get_by_name('source')
        # Important, as we will produce timestamped buffers
        self._gstreamer_pipeline.set_state(Gst.State.PLAYING)
        self._gstreamer_src.set_property('format', Gst.Format.TIME)

    def __del__(self):
        self._rhvoice = None
        if hasattr(self, '_gstreamer_pipeline'):
            self._gstreamer_pipeline.set_state(Gst.State.NULL)

    def speak(self, text, lang):
        super().speak(text, lang)
        Gst.Event.new_flush_start()

        # Get synthesized PCM sound from RHVoice server
        raw_sound = self._rhvoice.get(text, format_='pcm')

        Gst.Event.new_flush_stop(True)
        # Pipeline must be restarted exactly like this. Otherwise it will not push buffer after EOS signal
        self._gstreamer_pipeline.set_state(Gst.State.READY)
        self._gstreamer_pipeline.set_state(Gst.State.PLAYING)
        # Allocate new buffer
        buf = Gst.Buffer.new_allocate(None, len(raw_sound), None)
        buf.fill(0, raw_sound)
        # We cannot know duration of generated audio from rhvoice wrapper API
        buf.duration = Gst.CLOCK_TIME_NONE
        buf.pts = Gst.CLOCK_TIME_NONE
        # Push buffer on appsrc
        gst_flow_return = self._gstreamer_src.emit('push-buffer', buf)
        # Propagate End Of Stream event downstream to the sink
        self._gstreamer_src.emit('end-of-stream')
        if gst_flow_return != Gst.FlowReturn.OK:
            rospy.logerr('TTSRhvoiceWrapper: gstreamer flow error.')
            return False
        # wait
        bus = self._gstreamer_pipeline.get_bus()
        while True:
            msg = bus.timed_pop_filtered(100*Gst.MSECOND, (Gst.MessageType.ERROR | Gst.MessageType.EOS)) 
            if msg is not None:
                break
        self._gstreamer_pipeline.set_state(Gst.State.NULL)
        # check result message: it is ERROR or NULL
        if msg.type == Gst.MessageType.ERROR:
            err, _ = msg.parse_error()
            rospy.logerr('TTSRhvoiceWrapper: gstreamer failed: %s' % err)
            return False
        # result message was NULL
        rospy.loginfo('TTSRhvoiceWrapper: succeed.')
        return True

class PlayerGstreamer():
    def __init__(self, sound_packages, lang_prefixes):
        # List sound files
        self._sounds = {}
        for pkg in sound_packages:
            try:
                sounds_path = os.path.join(rospack.get_path(pkg), 'sounds')
            except rospkg.ResourceNotFound:
                rospy.logerr('Sound package `%s` is not found.' % pkg)
                continue

            for prefix in lang_prefixes:
                path = os.path.join(sounds_path, prefix)
       
                self._sounds.update(PlayerGstreamer._file_dict(path, 'wav'))
                self._sounds.update(PlayerGstreamer._file_dict(path, 'ogg'))
        # get ROS sound client
        self._player = Gst.ElementFactory.make("playbin", "ros_player")
        fakesink = Gst.ElementFactory.make("fakesink", "fakesink")
        self._player.set_property("video-sink", fakesink)

    def __del__(self):
        if hasattr(self, '_player'):
            self._player.set_state(Gst.State.NULL)

    def play(self, name):
        filename = self._sounds.get(name)
        if filename is None:
            rospy.logerr('Unknown play_wav sound: ' + name)
            return False
        # paly sound
        rospy.loginfo('Playing sound: {0} ({1}).'.format(name, filename))
        # invoke gstreamer pipe
        self._player.set_property("uri", "file://" + filename)
        self._player.set_state(Gst.State.READY)
        self._player.set_state(Gst.State.PLAYING)
        # wait for completion
        bus = self._player.get_bus()
        while True:
            msg = bus.timed_pop_filtered(100*Gst.MSECOND, (Gst.MessageType.ERROR | Gst.MessageType.EOS)) 
            if msg is not None:
                break
        self._player.set_state(Gst.State.NULL)
        # check result message: it is ERROR or NULL
        if msg.type == Gst.MessageType.ERROR:
            err, _ = msg.parse_error()
            rospy.logerr('PlayerGstreamer: gstreamer failed: %s' % err)
            return False
        # result message was NULL
        rospy.loginfo('PlayerGstreamer: succeed.')
        return True

    @staticmethod
    def _file_dict(directory, ext):
        ''' Return list of files with given extension.

            Paramters:
                directory (string) --- directory name,
                ext (string) --- file extension,

            Return: list of strings.
        '''
        files = {} 
        try:
            for f in os.listdir(directory):
                basename, extension = os.path.splitext(f)
                if extension.endswith(ext):
                    files[basename] = os.path.join(directory, f)
        except OSError as e:
            rospy.logwarn('Unable to list `%s` directory.' % directory)
            return []
        return files

class VoiceNode():
    def __init__(self):
        rospy.init_node('voice', anonymous = True)
        self.enable_gtranslate = rospy.get_param("~enable_gtranslate", True)

        profiles_config = rospy.get_param('~voice_profile') 
        if profiles_config is None or not isinstance(profiles_config, dict):
            rospy.logerr('Incorrect or missing "~voice_profiles" subtree')
            sys.exit(2)

        # define gender hints for translator
        self.voice_gender = rospy.get_param('~gender')
        self.gender_translation_hints = rospy.get_param('~gender_translation_hints')

        # load voice profiles
        self._voice_profile = {}
        for name, profile_config in profiles_config.items():
            print(name, profile_config)
            try:
                profile_type = profile_config.get('type')
                langs = profile_config.get('langs')
                if profile_type == 'rhvoice':
                    # apply LADSPA fix
                    VoiceNode._ladspa_fix()
                    # configure rhvoice_wrapper synthesyzer
                    gstreamer_pipeline = profile_config.get('gstreamer_pipeline')
                    rhvoice_params = profile_config.get('rhvoice_params')
                    self._voice_profile[name] = TTSRhvoiceWrapper(rhvoice_params, gstreamer_pipeline, langs = langs)
                elif profile_type == 'speechd':
                    speechd_params = copy.deepcopy(profile_config)
                    del speechd_params['type']
                    self._voice_profile[name] = TTSSpeechDispatcher(**speechd_params)
                elif profile_type == 'coqui-ai':
                    # Ensure we're running on CUDA machine
                    import torch
                    assert torch.cuda.is_available(), "CUDA is not availabe on this machine."

                    coqui_ai_params = profile_config.get('coqui_ai_params')
                    self._voice_profile[name] = TTSCoquiAi(coqui_ai_params, langs = langs)
                else:
                    rospy.logwarn('Unknown voice profile "%s" of type: %s' % (name, profile_type))
            except GLib.Error as e:
                rospy.logwarn("Profile '%s' initalization failed: %s" % (name, e.message))
            except ModuleNotFoundError as e:
                rospy.logwarn("Profile '%s' initalization failed: %s" % (name, e))
            except AssertionError as e:
                rospy.logwarn("Profile '%s' initalization failed: %s" % (name, e))

        if len(self._voice_profile) == 0:
            rospy.logerr('At least one voice profile must be specified.')
            sys.exit(2)

        # get and configure player
        sound_packages = rospy.get_param('~sound_packages', [])
        if not isinstance(sound_packages, list):
            rospy.logerr('"sound_packages" parameter must contain a list of packages\' names.')
            sys.exit(2)
        sound_packages.append('sweetie_bot_voice')
        rospy.loginfo('Sound packages: ' + repr(sound_packages))
        # Get language settings
        lang_prefixes = rospy.get_param('~lang', 'ru,en')
        if not isinstance(lang_prefixes, str):
            rospy.logerr('"lang" parameter must be a string.')
            sys.exit(2)
        lang_prefixes = str.split(lang_prefixes, ',')
        rospy.loginfo('Sound prefixes: ' + repr(lang_prefixes))
        
        self._player = PlayerGstreamer(sound_packages, lang_prefixes)

        # register ROS interface
        rospy.Subscriber('control', TextCommand, self.command_cb)
        self.pub = rospy.Publisher('mouth', TextCommand, queue_size=1)
        self._action_server = actionlib.SimpleActionServer('~syn', TextAction, execute_cb=self.action_cb, auto_start=False)
        rospy.sleep(0.2)
        self._action_server.start()
        rospy.on_shutdown(self.shutdown_cb)
            
    def _execute_text_command(self, cmd):
        # Check command type
        ret = False
        if cmd.type == 'voice/play_wav':
            # Play specified sound file
            self.pub.publish('mouth/speech', 'begin', '')
            ret = self._player.play(cmd.command)
            self.pub.publish('mouth/speech', 'end', '')
        elif cmd.type.startswith('voice/say'):
            # get lang code
            if len(cmd.type) == 12:
                lang = cmd.type[-2:]
            elif len(cmd.type) == 9:
                if cmd.options != '':
                    lang = cmd.options
                else:
                    lang = 'en'
            else:
                rospy.logerr('Bad text command type: %s' % cmd.type)
                return False

            # Publish original text before the translation as well
            voice_log.publish('log/voice/out/en', cmd.command, '')

            # translate to source lang
            if self.enable_gtranslate and lang !='en':
                gender_hint = self.gender_translation_hints[self.voice_gender]
                cmd.command = self.translate_text(lang, cmd.command, gender_hint)

            # find profile
            profile = None
            for k, p in self._voice_profile.items():
                if p.is_lang_supported(lang):
                    profile = p
                    break
            if profile is None:
                rospy.logerr('Unsupported laguage code: %s' % lang)
                return False
            # Invoke text-to-speech subsystem
            rospy.loginfo('use %s profile to say: %s (%s)' % (k, cmd.command, lang))
            self.pub.publish('mouth/speech', 'begin', '')
            ret = profile.speak(cmd.command, lang)
            self.pub.publish('mouth/speech', 'end', '')
        return ret

    @staticmethod
    def translate_text(target, text, hint=''):
        """Translates text into the target language.

        Target must be an ISO 639-1 language code.
        See https://g.co/cloud/translate/v2/translate-reference#supported_languages
        """
        translate_client = translate.Client()

        if isinstance(text, six.binary_type):
            text = text.decode("utf-8")

        rospy.loginfo(u"Text: {}".format(text))
        if hint:
            text = u'{}: """{}"""'.format(hint, text)

        # Text can also be a sequence of strings, in which case this method
        # will return a sequence of results for each text.
        result = translate_client.translate(text, target_language=target)
        translated_text = result["translatedText"]

        # Extrat text from wrapped translation with hint
        if hint:
            column_pos = translated_text.find(':')
            translated_text = translated_text[column_pos + 1:].lstrip()
            translated_text = re.sub('&quot;', '', translated_text)
            translated_text = translated_text.strip("«»\"' ")

        rospy.loginfo(u"Translation: {}".format(translated_text))
        rospy.loginfo(u"Detected source language: {}".format(result["detectedSourceLanguage"]))

        return translated_text

    def command_cb(self, cmd):
        # execute command 
        self._execute_text_command(cmd)

    def action_cb(self, goal):
        # execute command
        result = self._execute_text_command(goal.command)

        if result:
            self._action_server.set_succeeded(TextActionResult(error_code = 0, error_string = 'Succeed'))
        else:
            self._action_server.set_aborted(TextActionResult(error_code = 1, error_string = 'Failed'))

    def shutdown_cb(self):
        self._voice_profile = None
        self._player = None

    @staticmethod
    def _ladspa_fix():    
        # Get path to ladspa plugins directory
        ladspa_path = rospy.get_param('~ladspa_path', "~/.ladspa/")
        if not isinstance(ladspa_path, str):
            rospy.logerr('"ladspa_path" parameter must be a string')
            sys.exit(2)

        # Set mandatory enviroment variables for used gstreamer plugins
        os.environ['LADSPA_PATH'] = ladspa_path

        # Apply some dirty fix to make sure gstreamer will work
        os.system("rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin")


def main():
    Gst.init(None)
    node = VoiceNode()
    rospy.spin()

