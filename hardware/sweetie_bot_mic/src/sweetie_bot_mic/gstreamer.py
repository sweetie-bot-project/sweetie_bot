import threading
import weakref
import rospy
import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class WeakCallback():
    def __init__(self, method):
        if method is None:
            self._ref = None
        elif hasattr(method, '__self__'):
            self._ref = weakref.WeakMethod(method)
        elif hasattr(method, '__call__'):
            self._ref = weakref.ref(method)
        else:
            raise TypeError(f'GstreamerCallback: callable or None object is expeced: {method}')

    def __call__(self, *args, **kwargs):
        if self._ref is None:
            return None
        callback = self._ref()
        if callback is None:
            raise ReferenceError('GstreamerCallback: weakly-referenced object no longer exists')
        return callback(*args, **kwargs)

class GstreamerPipeline(object):
    _gstreamer_is_initialized = False

    FINALIZED = Gst.State.NULL
    PAUSED = Gst.State.READY
    RUNNING = Gst.State.PLAYING

    class EventBuffer():
        def __init__(self):
            self._buffer = []
            self._cond = threading.Condition()

        def pop(self):
            # wait until event is avalible in buffer and return it
            with self._cond:
                while  len(self._buffer) == 0:
                    self._cond.wait()
                return self._buffer.pop(0)

        def push(self, event):
            with self._cond:
                self._buffer.append(event)
                self._cond.notify()
            
    def __init__(self, pipeline, on_warning = None, on_error = None):
        self._gstreamer_pipeline = None
        self._event_thread = None
        self._on_warning = WeakCallback(on_warning)
        self._on_error = WeakCallback(on_error)
        # init gstreamer
        if not self._gstreamer_is_initialized:
            Gst.init(None)
            self._gstreamer_is_initialized = True
        # convert pipeline to string if it is represented in form of the list
        if isinstance(pipeline, list):
            pipeline = str.join(' ! ', pipeline_list)
        rospy.loginfo('gstreamer pipeline: \n  %s', pipeline.replace('!', '!\n  '))
        # construct gstreamaer object
        self._gstreamer_pipeline = Gst.parse_launch(pipeline)
        # event processing thread
        self._event_buffer = GstreamerPipeline.EventBuffer()
        self._event_thread = threading.Thread(target = GstreamerPipeline._event_loop, args = (weakref.ref(self), self._event_buffer))
        # add bus calback
        self._gstreamer_bus = self._gstreamer_pipeline.get_bus()
        self._gstreamer_bus.set_sync_handler(GstreamerPipeline._on_event, self._event_buffer)
        # prepare pipeline to start
        self._gstreamer_pipeline.set_state(Gst.State.READY)
        self._event_thread.start()

    @staticmethod
    def _event_loop(self_weakref, event_buffer):
        while True:
            # wait until message is available and get it
            msg = event_buffer.pop()
            # process event
            try:
                # get self 
                self = self_weakref()
                if self is None:
                    rospy.logwarn('GstreamerPipeline: object destroyed prematurely.')
                    return
                # process message
                if msg.type == Gst.MessageType.WARNING:
                    if self._on_warning is not None:
                    # call on warn callback
                        self._on_warning(msg)
                    # log it
                    rospy.logwarn('GstreamerPipeline: %s (%s)' % msg.parse_warning())

                elif msg.type == Gst.MessageType.ERROR:
                    # call on error callback
                    if self._on_error is not None:
                        self._on_error(msg)
                    # stop pipeline
                    self._gstreamer_pipeline.set_state(Gst.State.NULL)
                    rospy.logerr('GstreamerPipeline: stopping pipeline due to an error: %s (%s).' % msg.parse_error())

                elif msg.type == Gst.MessageType.EOS:
                    # exit event loop and thread
                    rospy.loginfo('GstreamerPipeline: stopping pipeline on EOS.')
                    return
            finally:
                del self

    @staticmethod
    def _on_event(bus, message, event_buffer):
        if message.type in (Gst.MessageType.ERROR, Gst.MessageType.WARNING, Gst.MessageType.EOS):
            # add message to buffer and notify event loop
            event_buffer.push(message)
        # drop all messages: GTK event loop is not active
        return Gst.BusSyncReply.DROP

    def __del__(self):
        if self._gstreamer_pipeline.current_state != Gst.State.NULL:
            rospy.logwarn('GstreamerPipeline: destructor is called before close() method is invoked. ')
            self.close()

    def start(self):
        state = self._gstreamer_pipeline.current_state
        if state in [Gst.State.READY, Gst.State.PAUSED, Gst.State.PLAYING]:
            self._gstreamer_pipeline.set_state(Gst.State.PLAYING)
        else:
            raise RuntimeError(f'GstreamerPipeline: pipeline is closed or in error state: {state}')

    def stop(self):
        state = self._gstreamer_pipeline.current_state
        if state in [Gst.State.READY, Gst.State.PAUSED, Gst.State.PLAYING]:
            self._gstreamer_pipeline.set_state(Gst.State.READY)
        else:
            raise RuntimeError(f'GstreamerPipeline: pipeline is closed or in error state: {state}')

    def close(self):
        # stop event loop
        if self._event_thread is not None:
            # send EOS message to event loop thread and wait until it exit
            self._event_buffer.push(Gst.Message.new_eos())
            self._event_thread.join()
            self._event_thread = None
        # stop pipeline
        if self._gstreamer_pipeline is not None:
            self._gstreamer_pipeline.set_state(Gst.State.NULL)

    def get_state(self):
        return self._gstreamer_pipeline.current_state


class GstreamerAudioSource(GstreamerPipeline):
            
    def __init__(self, on_audio, pipeline = None, sample_rate = 16000, n_channels=1, buffer_size = 1024, on_warning = None, on_error = None):
        # process parameters
        self.sample_width = 2
        self.rate = sample_rate
        self.on_audio = WeakCallback(on_audio)
        self.available_channels = n_channels
        # create gstreamer pipeline and configure it
        pipeline_list = []
        pipeline_list.append(pipeline)
        pipeline_list.extend([
            f'audio/x-raw,format=S16LE,channels={n_channels},rate={sample_rate},layout=interleaved',
            f'audiobuffersplit output-buffer-size={buffer_size*n_channels*self.sample_width} strict-buffer-size=true',
            f'appsink name=appsink-microphone-node max-buffers=10 drop=true emit-signals=true'
        ])
        pipeline_string = str.join(' ! ', pipeline_list)
        # init pipeline
        super(GstreamerAudioSource, self).__init__(pipeline_string, on_error = on_error, on_warning = on_warning)
        # add sample callback
        gstreamer_sink = self._gstreamer_pipeline.get_by_name('appsink-microphone-node')
        gstreamer_sink.connect("new-sample", GstreamerAudioSource._on_buffer, weakref.ref(self))
        # start pipeline
        self.start()

    @staticmethod
    def _on_buffer(sink, self_weakref):
        while True:
            # get gstreamer sample
            sample = sink.emit('try-pull-sample', 0.0)
            if sample is None:
                break
            # get buffer
            buffer = sample.get_buffer()
            if buffer is None:
                continue
            try:
                # bet buffer and map memory 
                success, buffer_map = buffer.map(Gst.MapFlags.READ)
                if not success:
                    continue
                # get reference to self
                self = self_weakref()
                if self is None:
                    break
                # construct numpy object over buffer
                data = np.frombuffer(buffer_map.data, dtype=np.int16)
                chunk_per_channel = len(data) // self.available_channels
                data_channels = np.reshape(data, (chunk_per_channel, self.available_channels))
                # TODO: eliminate copy
                # TODO:  as_strided(np.frombuffer(a.data, dtype=a.dtype, offset=a.dtype.type().nbytes*1), shape=(a.shape[0],2), strides=a.strides, writeable=False)
                # invoke callback
                self.on_audio(data_channels)
            finally:
                buffer.unmap(buffer_map)
                self = None

        return Gst.FlowReturn.OK

