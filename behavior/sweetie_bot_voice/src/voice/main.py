#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, copy
import rospy, actionlib, roslib, rospkg
from threading import Event

from sweetie_bot_text_msgs.msg import TextCommand
from sweetie_bot_text_msgs.msg import TextActionAction as TextAction
from sweetie_bot_text_msgs.msg import TextActionGoal, TextActionFeedback, TextActionResult

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class TTSInterface:
    def speak(self, text):
        pass

class TTSSpeechDispatcher(TTSInterface):
    def __init__(self, module = 'rhvoice', lang = 'ru'):
        import speechd
        self._client = speechd.SSIPClient('ros_speech_dispatcher_client')
        self._client.set_output_module(module)
        self._client.set_language(lang)

    def speak(self, text):
        ev = Event()
        self._client.speak(text, callback = lambda cb_type: ev.set(), event_types=(speechd.CallbackType.CANCEL, speechd.CallbackType.END))
        ev.wait()
        return True

class TTSRhvoiceWrapper(TTSInterface):

    def __init__(self, rhvoice_params, gstreamer_pipeline_string = None):
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

    def speak(self, text):
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
        rospack = rospkg.RosPack()
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

        profiles_config = rospy.get_param('~voice_profile') 
        if profiles_config is None or not isinstance(profiles_config, dict):
            rospy.logerr('Incorrect or missing "~voice_profiles" subtree')
            sys.exit(2)

        # load voice profiles
        self._voice_profile = {}
        for name, profile_config in profiles_config.items():
            try:
                profile_type = profile_config.get('type')
                if profile_type == 'rhvoice':
                    # apply LADSPA fix
                    VoiceNode._ladspa_fix()
                    # configure rhvoice_wrapper synthesyzer
                    gstreamer_pipeline = profile_config.get('gstreamer_pipeline')
                    rhvoice_params = profile_config.get('rhvoice_params')
                    self._voice_profile[name] = TTSRhvoiceWrapper(rhvoice_params, gstreamer_pipeline)
                elif profile_type == 'speechd':
                    speechd_params = copy.deepcopy(profile_config)
                    del speechd_params['type']
                    self._voice_profile[name] = TTSSpeechDispatcher(**speechd_params)
                else:
                    rospy.logwarn('Unknown voice profile "%s" of type: %s' % (name, profile_type))
            except GLib.Error as e:
                rospy.logwarn("Profile '%s' initalization failed: %s" % (name, e.message))

        if len(self._voice_profile) == 0:
            rospy.logerr('At least one voice profile must be specified.')
            sys.exit(2)

        # get default profile
        default_profile = rospy.get_param("~default_voice_profile", None)
        if default_profile == None:
            self._default_profile = next(iter(self._voice_profile.values())) # get 'first' value
        else:
            profile = self._voice_profile.get(default_profile)
            # check if profile exists
            if profile is not None:
                self._default_profile = profile
            else:
                rospy.logwarn("Profile '%s' does not exist. Use fallback." % (default_profile,))
                self._default_profile = next(iter(self._voice_profile.values())) 

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
        self.pub.publish('mouth/speech', 'begin')

        ret = False
        if cmd.type == 'voice/play_wav':
            # Play specified sound file
            ret = self._player.play(cmd.command)
        elif cmd.type == 'voice/say':
            # Invoke text-to-speech subsystem
            ret = self._default_profile.speak(cmd.command)

        self.pub.publish('mouth/speech', 'end')
        return ret

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

