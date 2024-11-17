#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, json
from copy import copy

import rospy
from rospy_message_converter import message_converter

from sweetie_bot_load_balancer.balancer import Balancer
from sweetie_bot_text_msgs.srv import LibreTranslate, LibreTranslateRequest, LibreTranslateResponse
from sweetie_bot_text_msgs.srv import Translate, TranslateRequest, TranslateResponse
from sweetie_bot_text_msgs.msg import TextCommand

import six
import re
from google.cloud import translate_v2 as translate

class TranslateError(RuntimeError):
    def __init__(self, msg, details = ''):
        super(TranslateError, self).__init__(msg)
        self._details = details

    @property
    def details(self):
        return self._details

class TranslateNode:
    DEFAULT_CONFIG = dict(server_choices=dict(
        local_host = {'url': 'http://127.0.0.1:5001/translate'},
    ))

    def __init__(self):
        rospy.init_node('translate_server')
        # create services
        rospy.Service('libre_translate', LibreTranslate, self.libre_translate_callback)
        rospy.Service('translate', Translate, self.translate_callback)

        balancer_config = rospy.get_param("~balancer_config", self.DEFAULT_CONFIG)
        self.balancer = Balancer(
            balancer_config,
            loggers={'debug': rospy.logdebug, 'warn': rospy.logwarn, 'info': rospy.loginfo},
            postprocess_func=self.unpack_translation_result,
            fallback_func=self.translation_is_unavailable_behavior
        )

        # log
        self.log = rospy.Publisher('speech_log', TextCommand, queue_size=10)

    def translation_is_unavailable_behavior(self, original_request):
        # In the case when all translation servers are failed
        # keep the original source language as the profile language
        request = original_request.pop('json')
        response = {'translatedText': request['q']}
        if 'detected_source' in request and request['detected_source'] != '':
            response['detectedLanguage'] = {
                'language'   : request['detected_source'],
                'confidence' : 100.0,
            }

        return response

    def unpack_translation_result(self, response):
        # check response structure
        if ( 'error' in response or 'translatedText' not in response):
            raise TranslateError('wrong response structure', response)

        # Eliminate all dots, so TTS wouldn't have a chance to pronounce them
        response['translatedText'] = response['translatedText'].replace('.', ';')

        # Eliminate all other unwanted characters, that TTS can potentially pronounce
        response['translatedText'] = re.sub("«|»", " ", response['translatedText'])

        # convert to unicode
        try:
            # TODO: what is this? decode and then encode?
            msg = TextCommand()
            msg.type = "translate"
            msg.command = response['translatedText'].encode('utf-8').strip().decode()
            if 'detectedLanguage' in response:
                msg.options = response['detectedLanguage']['language']
            self.log.publish(msg)
        except UnicodeDecodeError:
            raise TranslateError('unicode convertion error', response)

        return response

    def libre_translate_request_server(self, request):
        # display request
        rospy.logdebug(f'request: \n\n %s \n\n' % request)

        try:
            response, _ = self.balancer.request_available_server(json=request, decode_json=True)
        except Exception as e:
            rospy.logerr(f'translator: {e}')
            return TranslateResponse(status = str(e))

        return response

    def libre_translate_callback(self, req_message):
        # prepare request
        request = message_converter.convert_ros_message_to_dictionary(req_message)
        # send it to servers
        try:
            response = self.libre_translate_request_server(request)
        except TranslateError as e:
            rospy.logerr('%s: %s' % (e, e.details))
            return LibreTranslateResponse(status = 'error')
        # return result
        try:
            return message_converter.convert_dictionary_to_ros_message('sweetie_bot_text_msgs/LibreTranslate',
                    response,
                    kind="response",
                    strict_mode=False,
                    check_missing_fields=False)
        except Exception as e:
            rospy.logerr(f'translator: {e}')
            rospy.logerr(f'convertion error: {response}')
            raise

    def translate_callback(self, req_message):
        # prepare request
        translate_request = message_converter.convert_ros_message_to_dictionary(req_message)
        translate_request['q'] = translate_request.pop('text')

        try:
            libre_translate_request = message_converter.convert_dictionary_to_ros_message('sweetie_bot_text_msgs/LibreTranslate',
                    translate_request,
                    kind="request",
                    strict_mode=False,
                    check_missing_fields=False)
        except Exception as e:
            rospy.logerr(f'translator: {e}')
            rospy.logerr(f'convertion error: {translate_request}')
            raise

        libre_translate_request.target = libre_translate_request.target[:2]
        libre_translate_response = self.libre_translate_callback(libre_translate_request)

        response = TranslateResponse()
        response.text   = libre_translate_response.translatedText
        response.source = libre_translate_response.detectedLanguage.language
        response.status = libre_translate_response.status
        response.target = req_message.target
        return response


# TODO: Integrate google translate back with new balancing mechanism
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

def main():
    n = TranslateNode()

    rospy.loginfo("Ready to translate.")
    rospy.spin()

