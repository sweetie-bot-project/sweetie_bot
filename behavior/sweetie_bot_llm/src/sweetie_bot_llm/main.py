#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, json
import requests
from copy import copy

import rospy
from rospy_message_converter import message_converter

from sweetie_bot_text_msgs.srv import Complete, CompleteRequest, CompleteResponse
from sweetie_bot_text_msgs.srv import CompleteSimple, CompleteSimpleRequest, CompleteSimpleResponse
from sweetie_bot_text_msgs.srv import CompleteRaw, CompleteRawRequest, CompleteRawResponse
from sweetie_bot_text_msgs.msg import TextCommand


class CompleteError(RuntimeError):
    def __init__(self, msg, details = ''):
        super(CompleteError, self).__init__(msg)
        self._details = details

    @property
    def details(self):
        return self._details

class CompleteNode:

    def __init__(self):
        rospy.init_node('complete_server')
        # get server URLS
        urls = rospy.get_param("~lang_model_servers", {'0': "http://127.0.0.1:5000/api/v1/generate"})
        self._urls = [ urls[k] for k in sorted(urls) ]
        # get profiles
        self._profiles = rospy.get_param("~profile", {})
        # create services
        rospy.Service('complete', Complete, self.complete_callback)
        rospy.Service('input_text', CompleteSimple, self.input_text_callback)
        rospy.Service('llm_request', CompleteRaw, self.complete_simple_callback)
        # log
        self.log_llm = rospy.Publisher('speech_log', TextCommand, queue_size=10)
        rospy.loginfo('urls: %s', self._urls )
        rospy.loginfo('profiles: %s', [self._profiles.keys()] )

    def request_server(self, request):
        # display request
        rospy.logdebug(f'request: \n\n %s \n\n' % request)
        # send request to servers
        resp = None
        for url in self._urls:
            try:
                start = time.time()
                resp = requests.post(url, json=request)
                duration = time.time() - start
                if resp.status_code == 200:
                    break
                else:
                    rospy.logwarn("server $s request error %d: %s" % (url, resp.status_code, resp.reason))
                    continue # next url
            except requests.ConnectionError as e:
                rospy.logwarn("connection error: %s" % e)

        # check if response is received
        if resp is None:
            raise CompleteError('unable to get response: tried all servers.')
        # decode
        try:
            response = resp.json()
        except ValueError:
            raise CompleteError('json: can not decode response', resp.content)
        # display result
        rospy.logdebug(f'server %s response: \n\n %s \n\n' % (url, response))
        # check response structure
        if ( 'results' not in response or len(response['results'])==0 or 'text' not in response['results'][0]):
            raise CompleteError('wrong response structure', resp.content)
        # convert to unicode
        try:
            # TODO: what is this? decode and then encode?
            text = response['results'][0]['text'].encode('utf-8').strip()
            text = text.decode()
        except UnicodeDecodeError:
            return CompleteError('unicode decode error', resp.content)

        return text, duration

    def complete_callback(self, req_message):
        # prepare request
        request = message_converter.convert_ros_message_to_dictionary(req_message)
        # send it to servers
        try:
            text, duration = self.request_server(request)
        except CompleteError as e:
            rospy.logerr('%s: %s' % (e, e.details))
            return CompleteResponse(status = e))
        # return result
        return CompleteResponse(status='ok', text = text, duration=duration)

    def complete_simple_callback(self, msg):
        # get profile
        profile = self._profiles.get(msg.profile)
        if profile is None:
            rospy.logerr('unknown profile: %s' % msg.profile)
            return CompleteRawResponse(status_error = CompleteRawResponse.UNKNOWN_PROFILE)
        # construct request 
        request = copy(profile) # shallow copy: one level is enought
        request['prompt'] = msg.prompt
        if len(msg.stop_list) != 0:
            request['stopping_strings'] = msg.stop_list

        self.log_llm.publish('log/llm/in/'+msg.profile, msg.prompt, '')
        # send it to server
        try:
            text, duration = self.request_server(request)
        except CompleteError as e:
            rospy.logerr('%s: %s' % (e, e.details))
            return CompleteRawResponse(status_error = CompleteRawResponse.SERVER_UNREACHABLE)

        self.log_llm.publish('log/llm/out/'+msg.profile, text, '')

        # return result
        return CompleteRawResponse(result = text, error_code = CompleteRawResponse.SUCCESS)

    def input_text_callback(self, msg):
        # check input command type
        if msg.data.type != 'complete/request':
            return CompleteSimpleResponse(data = TextCommand(type = "complete/error", command = 'wrong command type, expected complete/request'))
        # get profile
        profile = self._profiles.get(msg.data.options)
        if profile is None:
            rospy.logerr('unknown profile: %s' % msg.data.options)
            return CompleteSimpleResponse(data = TextCommand(type = "complete/error", command = 'unknown profile'))
        # produce request
        request = copy(profile)
        request['prompt'] = request['prompt'] % msg.data.command
        # send it to server
        try:
            text, duration = self.request_server(request)
        except CompleteError as e:
            rospy.logerr('%s: %s' % (e, e.details))
            return CompleteSimpleResponse(data = TextCommand(type = "complete/error", command = e))
        # return result
        return CompleteSimpleResponse(data = TextCommand(type = "complete/response", command = text))

def main():
    n = CompleteNode()
    rospy.spin()

