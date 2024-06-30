#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, json
import requests
from copy import copy

import rospy
from rospy_message_converter import message_converter

from sweetie_bot_text_msgs.srv import LibreTranslate, LibreTranslateRequest, LibreTranslateResponse
from sweetie_bot_text_msgs.srv import Translate, TranslateRequest, TranslateResponse
from sweetie_bot_text_msgs.msg import TextCommand

class TranslateError(RuntimeError):
    def __init__(self, msg, details = ''):
        super(TranslateError, self).__init__(msg)
        self._details = details

    @property
    def details(self):
        return self._details

class TranslateNode:

    def __init__(self):
        rospy.init_node('translate_server')
        # get server URLS
        libre_urls = rospy.get_param("~libre_translate_servers", {'0': "http://127.0.0.1:5001/translate"})
        self._libre_urls = [ libre_urls[k] for k in sorted(libre_urls) ]
        # create services
        rospy.Service('libre_translate', LibreTranslate, self.libre_translate_callback)
        rospy.Service('translate', Translate, self.translate_callback)
        # log
        self.log = rospy.Publisher('speech_log', TextCommand, queue_size=10)
        rospy.loginfo('libre_urls: %s', self._libre_urls )

    def libre_translate_request_server(self, request):
        # display request
        rospy.logdebug(f'request: \n\n %s \n\n' % request)
        # send request to servers
        resp = None
        for url in self._libre_urls:
            try:
                start = time.time()
                resp = requests.post(url, json=request)
                duration = time.time() - start
                if resp.status_code == 200:
                    break
                else:
                    rospy.logwarn("server %s request error %d: %s" % (url, resp.status_code, resp.reason))
                    continue # next url
            except requests.ConnectionError as e:
                rospy.logwarn("connection error: %s" % e)

        # check if response is received
        if resp is None:
            raise TranslateError('unable to get response: tried all servers.')
        # decode
        try:
            response = resp.json()
        except ValueError:
            raise TranslateError('json: can not decode response', resp.content)
        # display result
        rospy.logdebug(f'server %s response: \n\n %s \n\n' % (url, response))
        # check response structure
        if ( 'error' in response or 'translatedText' not in response):
            raise TranslateError('wrong response structure', resp.content)
        # convert to unicode
        try:
            # TODO: what is this? decode and then encode?
            msg = TextCommand()
            msg.type = "translate"
            msg.command = response['translatedText'].encode('utf-8').strip().decode()
            msg.options = response['detectedLanguage']['language']
            self.log.publish(msg)
        except UnicodeDecodeError:
            return TranslateError('json decode error', resp.content)

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
        return message_converter.convert_dictionary_to_ros_message('sweetie_bot_text_msgs/LibreTranslate',
                response,
                kind="response",
                strict_mode=False,
                check_missing_fields=False)

    def translate_callback(self, req_message):
        # prepare request
        translate_request = message_converter.convert_ros_message_to_dictionary(req_message)
        translate_request['q'] = translate_request.pop('text')

        libre_translate_request = message_converter.convert_dictionary_to_ros_message('sweetie_bot_text_msgs/LibreTranslate',
                translate_request,
                kind="request",
                strict_mode=False,
                check_missing_fields=False)

        libre_translate_response = self.libre_translate_callback(libre_translate_request)

        response = TranslateResponse()
        response.text   = libre_translate_response.translatedText
        response.source = libre_translate_response.detectedLanguage.language
        response.status = libre_translate_response.status
        response.target = req_message.target
        return response

def main():
    n = TranslateNode()
    rospy.spin()

