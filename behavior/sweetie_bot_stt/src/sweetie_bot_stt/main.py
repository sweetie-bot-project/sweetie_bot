#!/usr/bin/env python3
import os
import rospy

from sweetie_bot_load_balancer.balancer import Balancer
from sweetie_bot_text_msgs.msg import TextCommand
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse
from sweetie_bot_text_msgs.srv import Translate, TranslateRequest, TranslateResponse

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
    DEFAULT_CONFIG = dict(server_choices=dict(
        local_host = {'url': 'http://localhost:8577/'},
    ))

    def __init__(self):
        self.enable_translation = rospy.get_param("~enable_translation", True)

        balancer_config = rospy.get_param("~balancer_config", self.DEFAULT_CONFIG)
        self.balancer = Balancer(
            balancer_config,
            loggers={'debug': rospy.logdebug, 'warn': rospy.logwarn, 'info': rospy.loginfo},
            postprocess_func=self.translate_phrase,
        )

        self._translate_client = rospy.ServiceProxy('/translate', Translate)

        rospy.Service('transcribe', Transcribe, self.transcribe_file)
        rospy.Service('transcribe_api', Transcribe, self.transcribe)

        self.voice_log = rospy.Publisher('voice_log', TextCommand, queue_size=10)

    def translate_phrase(self, resp_decoded):
        self.voice_log.publish('log/voice/in/'+resp_decoded['language'], resp_decoded['text'], '')

        if self.enable_translation and resp_decoded['language'] !='en':
            req = TranslateRequest(text=resp_decoded['text'], source='auto', target='en')
            translate_response = self._translate_client(req)

            resp_decoded['text'] = translate_response.text
            resp_decoded['status'] = translate_response.status
            # Switch profile languate to translated one
            resp_decoded['language'] = translate_response.target

        return resp_decoded

    def transcribe(self, req):
        # request transcribe server
        try:
            response, _ = self.balancer.request_available_server(files={'file': ('audio.wav', req.data)}, decode_json=True)
        except Exception as e:
            breakpoint()
            rospy.logerr(f'transcriber: {e}')
            return TranscribeResponse(status = e)

        rospy.loginfo('Transcription %s (%.2fs) [%s]: "%s"' % (response['status'],
                                                               response['transcribe_duration'],
                                                               response['language'],
                                                               response['text']))
        # Publish translated verssion of the text as well
        self.voice_log.publish('log/voice/in/en', response['text'], '')

        return TranscribeResponse(**response)

    def transcribe_file(self, req):
        if req.filename and os.path.isfile(req.filename):
            rospy.logdebug(f'Got filename = {req.filename}')
            with open(req.filename, 'rb') as file:
                req.data = file.read()

        try:
            response, _ = self.balancer.request_available_server(files={'file': ('audio.wav', req.data)}, decode_json=True)
        except Exception as e:
            breakpoint()
            rospy.logerr(f'transcriber: {e}')
            return TranscribeResponse(status = e)

        rospy.loginfo('Transcription %s (%.2fs): "%s"' % (response['status'],
                                                          response['transcribe_duration'],
                                                          response['text']))

        return TranscribeResponse(**response)


def main():
    rospy.init_node('transcribe_server')

    transcriber = TranscriberNode()

    rospy.loginfo("Ready to transcribe.")
    rospy.spin()

