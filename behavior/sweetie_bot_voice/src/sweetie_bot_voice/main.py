#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, copy, re, time
import tempfile
import rospy, actionlib, roslib, rospkg
from threading import Event
import six

from sweetie_bot_load_balancer.balancer import Balancer, BalancerError

from std_msgs.msg import Bool
from sweetie_bot_text_msgs.msg import TextCommand
from sweetie_bot_text_msgs.msg import TextActionAction as TextAction
from sweetie_bot_text_msgs.msg import TextActionGoal, TextActionFeedback, TextActionResult
from sweetie_bot_text_msgs.srv import Translate, TranslateRequest, TranslateResponse

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# @Important! If TTS imported after google/Gst imports,
#             its initialization crashes without any error messages.
from rhvoice_wrapper import TTS

soundhandle = SoundClient(blocking=True)
rospack = rospkg.RosPack()

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class TTSInterface:
    def __init__(self, langs):
        self._langs = langs

    def is_lang_supported(self, lang):
        return lang in self._langs

    def speak(self, text, language):
        raise NotImplementedError

class TTSCoquiAi(TTSInterface):
    speaker = ""
    DEFAULT_CONFIG = dict(server_choices=dict(
        local_host = {'url': 'http://127.0.0.1:5002/api/tts'},
    ))

    def __init__(self, balancer_config, init_params, **kwargs):
        super(TTSCoquiAi, self).__init__(**kwargs)
        self.speaker = init_params['speaker']
        self.balancer = Balancer(balancer_config,
            loggers={'debug': rospy.logdebug, 'warn': rospy.logwarn, 'info': rospy.loginfo},
        )

    def speak(self, text, language):
        try:
            request = { "text": text, "language_id": f'{language}-cn' if language == 'zh' else language, "speaker_id": self.speaker }
            response, _ = self.balancer.request_available_server(data=request)
        except BalancerError as e:
            rospy.logerr('TTSCoquiAi: text to speach error: %s', e)
            return False

        # TODO: rewrite using gstreamer
        temp_dir = tempfile.mkdtemp()
        temp_file = os.path.join(temp_dir, "output.wav")

        with open(temp_file, 'wb') as the_file:
            the_file.write(response.content)
            the_file.close()

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
        ev = Event()
        self._client.set_language(lang)
        self._client.speak(text, callback = lambda cb_type: ev.set(), event_types=(self._speechd.CallbackType.CANCEL, self._speechd.CallbackType.END))
        ev.wait()
        return True

class TTSRhvoiceWrapper(TTSInterface):

    def __init__(self, rhvoice_params, gstreamer_pipeline_string = None, **kwargs):
        super(TTSRhvoiceWrapper, self).__init__(**kwargs)

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
        rospy.logdebug('TTSRhvoiceWrapper: succeed.')
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
        self._player.set_property("volume", 1.0)
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
            rospy.logdebug('Unable to list `%s` directory.' % directory)
            return []
        return files

class VoiceNode():
    def __init__(self):
        rospy.init_node('voice')
        self.enable_translate = rospy.get_param("~enable_translate", True)
        if self.enable_translate:
            self.translate_srv_call = rospy.ServiceProxy('translate', Translate)

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
            # TODO Use Factory Pattern
            try:
                if not profile_config.get('enabled'):
                    continue
                del profile_config['enabled']
                    
                profile_type = profile_config.get('type')
                langs = profile_config.get('langs')
                if profile_type.startswith('rhvoice'):
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
                    coqui_ai_params = profile_config.get('coqui_ai_params')
                    balancer_config = profile_config.get('balancer_config')
                    self._voice_profile[name] = TTSCoquiAi(balancer_config, coqui_ai_params, langs=langs)
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

        rospy.loginfo(f'Voice profiles: {[self._voice_profile.keys()]}')

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
        self._voice_log_pub = rospy.Publisher('voice_log', TextCommand, queue_size=10)
        self._mouth_pub = rospy.Publisher('mouth', TextCommand, queue_size=1)
        self._voice_start_stop_pub = rospy.Publisher('voice_start_stop', Bool, queue_size=2)
        self._action_server = actionlib.SimpleActionServer('~syn', TextAction, execute_cb=self.action_cb, auto_start=False)
        rospy.sleep(0.2)
        self._action_server.start()
        rospy.on_shutdown(self.shutdown_cb)

    def _publish_voice_status(self, is_speaking):
        if is_speaking:
            self._mouth_pub.publish('mouth/speech', 'begin', '')
            self._voice_start_stop_pub.publish(True)
        else:
            self._mouth_pub.publish('mouth/speech', 'end', '')
            self._voice_start_stop_pub.publish(False)
            
    def _execute_text_command(self, cmd):
        # Check command type
        success = False
        if cmd.type == 'voice/play_wav':
            # Play specified sound file
            self._publish_voice_status(is_speaking = True)
            success = self._player.play(cmd.command)
            self._publish_voice_status(is_speaking = False)
        elif cmd.type.startswith('voice/say'):
            # get lang code
            if len(cmd.type) >= 12:
                lang = cmd.type.rsplit('/',1)[-1]
            elif len(cmd.type) == 9:
                # TODO: remove?
                if cmd.options != '':
                    lang = cmd.options
                else:
                    lang = 'en'
            else:
                rospy.logerr(f"Bad text command type: {cmd.type} ({len(cmd.type)})")
                return False

            # Publish original text before the translation as well
            self._voice_log_pub.publish('log/voice/out/en', cmd.command, '')
            rospy.loginfo(f"Text: {cmd.command}")

            # translate to source lang
            if self.enable_translate and lang !='en':
                #gender_hint = self.gender_translation_hints[self.voice_gender]
                gender_hint = ''
                translated_text = self.translate_text(cmd.command, lang, gender_hint)
                if translated_text is not None:
                    cmd.command = translated_text
                else:
                    lang = 'en' # ????

            # speech log, indicate that speech has started
            self._voice_log_pub.publish('log/voice/out/'+lang, cmd.command, '')
            self._publish_voice_status(is_speaking = True)
            # try to say text 
            for profile_name, profile in self._voice_profile.items():
                # check if language is supported
                if not profile.is_lang_supported(lang):
                    continue
                # use profile
                rospy.loginfo('use %s profile to say: %s (%s)', profile_name, cmd.command, lang)
                success = profile.speak(cmd.command, lang)
                # exit on success
                if success:
                    break
            # speech end
            self._publish_voice_status(is_speaking = False)
            # log error
            if not success:
                rospy.logerr('Text-to-speech translation failed: tried all profiles:  %s (%s)', cmd.command, lang)
            # Invoke text-to-speech subsystem
        return success

    def translate_text(self, text, target_lang, hint=''):

        if isinstance(text, six.binary_type):
            text = text.decode("utf-8")

        if hint:
            text = u'{}: """{}"""'.format(hint, text)

        try:
            result = self.translate_srv_call(text=text, source='en', target=target_lang)
            translated_text = result.text
        except rospy.ServiceException as e:
            rospy.logerr(f'Translation from "en" to "{target_lang}" failed: {e}')
            return None

        # Extrat text from wrapped translation with hint
        if hint:
            column_pos = translated_text.find(':')
            translated_text = translated_text[column_pos + 1:].lstrip()
            translated_text = re.sub('&quot;', '', translated_text)
            translated_text = translated_text.strip("«»\"' ")

        rospy.loginfo("Translation: %s", translated_text)
        rospy.loginfo("Received target language: %s", target_lang)

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

