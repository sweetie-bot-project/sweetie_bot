#!/usr/bin/env python3
import os
import rospy
import requests
import struct

from sweetie_bot_text_msgs.msg import TextCommand
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse

import six
from google.cloud import translate_v2 as translate


def translate_text(target, text):
    """Translates text into the target language.

    Target must be an ISO 639-1 language code.
    See https://g.co/cloud/translate/v2/translate-reference#supported_languages
    """
    translate_client = translate.Client()

    if isinstance(text, six.binary_type):
        text = text.decode("utf-8")

    # Text can also be a sequence of strings, in which case this method
    # will return a sequence of results for each text.
    result = translate_client.translate(text, target_language=target)

    #print(u"Text: {}".format(result["input"]))
    #print(u"Translation: {}".format(result["translatedText"]))
    #print(u"Detected source language: {}".format(result["detectedSourceLanguage"]))

    return result["translatedText"]


class TranscriberNode(object):

    def __init__(self):
        servers = rospy.get_param("~transcribe_servers", {'0': "http://localhost:8577/"})
        self.enable_gtranslate = rospy.get_param("~enable_gtranslate", True)
        self.transcribe_servers = [ servers[k] for k in sorted(servers) ]
        rospy.loginfo(f"urls: {self.transcribe_servers}")
        self.voice_log = rospy.Publisher('voice_log', TextCommand, queue_size=10)

    def transcribe(self, msg):
        # TODO: Factor out of vaw generation from
        #       this function back into mic
        data = msg.data
        rate = msg.rate
        sample_width = msg.sample_width
        # prepare wav data
        wav_data = bytearray()
        wav_data += struct.pack('<4sL4s4sLHHLLHH4sL', b'RIFF', 
                36 + len(data), b'WAVE', b'fmt ', 16,
                1, # no compression
                1, # n channels
                rate, # framerate,
                1 * rate * sample_width,
                1 * sample_width,
                sample_width * 8, 
                b'data', len(data))
        wav_data += data
        # request transcribe server
        resp = None
        for url in self.transcribe_servers:
            rospy.logdebug(f"Send {len(data)} bytes to {url}")
            try:
                resp = requests.post(url, files={'file': ('audio.wav', wav_data)})
                # check status
                if resp.status_code == 200:
                    break
                else:
                    rospy.logerr(f"{resp.status_code} {resp.reason}")
            except requests.ConnectionError as e:
                rospy.logwarn(f"Connection failed: {e}")
        # deacode server response
        if resp is None:
            rospy.logerr("Transcription failed! Cannot decode response")
            return None
        try:
            resp_decoded = resp.json()
        except:
            rospy.logerr(f"Transcription failed! Cannot decode response ({resp})")
            return None

        self.voice_log.publish('log/voice/in/'+resp_decoded['language'], resp_decoded['text'], '')

        if self.enable_gtranslate and resp_decoded['language'] !='en':
            resp_decoded['text'] = translate_text('en', resp_decoded['text'])

        # Publish translated verssion of the text as well
        self.voice_log.publish('log/voice/in/en', resp_decoded['text'], '')

        rospy.loginfo('Transcription %s (%.2fs) [%s]: "%s"' % (resp_decoded['status'],
                                                               resp_decoded['transcribe_duration'],
                                                               resp_decoded['language'],
                                                               resp_decoded['text']))

        return TranscribeResponse(**resp_decoded)

    def transcribe_file(req):
        if req.filename and os.path.isfile(req.filename):
            rospy.logdebug("Got filename = %s" % req.filename)
            with open(req.filename, 'rb') as file:
                req.data = file.read()

        r = None
        for n in sorted(self.transcribe_servers):
            rospy.loginfo("Send %d bytes to %s" % (len(req.data), self.transcribe_servers[n]))
            try:
                r = None
                r = requests.post(self.transcribe_servers[n], files={'file': ('audio.wav', req.data)})
                if r.status_code == 200:
                    break
                else:
                    rospy.logerr("%d %s" % (r.status_code, r.reason))
                    continue # next url
            except requests.ConnectionError as e:
                rospy.logerr("Connection failed! %s" % e)
                # nexu url

        if r is None:
            return TranscribeResponse(status="All API URLs are down!")
        try:
          response = r.json()
          if r.status_code != 200:
              response['status'] = r.reason

          if self.enable_gtranslate and response['language'] !='en':
              response['text'] = translate_text('en', response['text'])

          rospy.loginfo('Transcription %s (%.2fs): "%s"' % (response['status'],
                                                            response['transcribe_duration'],
                                                            response['text']))
        except:
          rospy.logerr('Cannot decode response (%s)' % (r.content))
          return TranscribeResponse()

        return TranscribeResponse(**response)


def main():
    rospy.init_node('transcribe_server')

    transcriber = TranscriberNode()
    rospy.Service('transcribe', Transcribe, transcriber.transcribe_file)
    rospy.Service('transcribe_api', Transcribe, transcriber.transcribe)

    rospy.loginfo("Ready to transcribe.")
    rospy.spin()

