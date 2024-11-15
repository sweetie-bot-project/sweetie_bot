#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, json
import requests
from copy import copy

import rospy
from rospy_message_converter import message_converter

from sweetie_bot_load_balancer.balancer import Balancer
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
    DEFAULT_CONFIG = dict(server_choices=dict(
        local_host = {'url': 'http://127.0.0.1:5000/v1/completions'},
    ))

    def __init__(self):
        rospy.init_node('complete_server')
        # get profiles
        self._profiles = rospy.get_param("~profile", {})
        # create services
        rospy.Service('complete', Complete, self.complete_callback)
        rospy.Service('input_text', CompleteSimple, self.input_text_callback)
        rospy.Service('llm_request', CompleteRaw, self.complete_simple_callback)

        balancer_config = rospy.get_param("~balancer_config", self.DEFAULT_CONFIG)
        self.balancer = Balancer(
            balancer_config,
            loggers={'debug': rospy.logdebug, 'warn': rospy.logwarn, 'info': rospy.loginfo},
            postprocess_func=self.extract_response_text,
            fallback_func=lambda _: balancer_config['fallback_answer']
        )

        # log
        self.log_llm = rospy.Publisher('speech_log', TextCommand, queue_size=10)
        rospy.loginfo(f'profiles: {[self._profiles.keys()]}')

    def extract_response_text(self, response):
        # check response structure
        if ( 'choices' not in response or len(response['choices'])==0 or 'text' not in response['choices'][0]):
            raise CompleteError('wrong response structure', response.get('detail', 'Unknown error'))

        # convert to unicode
        try:
            # TODO: what is this? decode and then encode?
            text = response['choices'][0]['text'].encode('utf-8').strip()
            text = text.decode()
        except UnicodeDecodeError:
            raise CompleteError('unicode decode error', response.get('detail', 'Unknown error'))

        return text

    def complete_callback(self, req_message):
        # prepare request
        request = message_converter.convert_ros_message_to_dictionary(req_message)
        # send it to servers
        try:
            text, duration = self.balancer.request_available_server(json=request, decode_json=True)
        except Exception as e:
            rospy.logerr(e + f': {e.details}' if hasattr(e, 'details') else '')
            return CompleteResponse(status = e)
        # return result
        return CompleteResponse(status='ok', text = text, duration=duration)

    def complete_simple_callback(self, msg):
        # get profile
        profile = self._profiles.get(msg.profile_name)
        if profile is None:
            rospy.logerr(f'unknown profile: {msg.profile_name}')
            return CompleteRawResponse(error_code = CompleteRawResponse.UNKNOWN_PROFILE)
        # construct request 
        request = copy(profile) # shallow copy: one level is enought
        request['prompt'] = msg.prompt
        if msg.temperature >= 0:
            request['temperature'] = msg.temperature
        if len(msg.stop_list) != 0:
            request['stop'] = msg.stop_list

        self.log_llm.publish('log/llm/in/'+msg.profile_name, msg.prompt, '')
        # send it to server
        try:
            text, duration = self.balancer.request_available_server(json=request, decode_json=True)
        except Exception as e:
            rospy.logerr(e + f': {e.details}' if hasattr(e, 'details') else '')
            return CompleteRawResponse(error_code = CompleteRawResponse.SERVER_UNREACHABLE)

        self.log_llm.publish('log/llm/out/'+msg.profile_name, text, '')

        # return result
        return CompleteRawResponse(result = text, error_code = CompleteRawResponse.SUCCESS)

    def input_text_callback(self, msg):
        # check input command type
        if msg.data.type != 'complete/request':
            return CompleteSimpleResponse(data = TextCommand(type = "complete/error", command = 'wrong command type, expected complete/request'))
        # get profile
        profile = self._profiles.get(msg.data.options)
        if profile is None:
            rospy.logerr(f'unknown profile: {msg.data.options}')
            return CompleteSimpleResponse(data = TextCommand(type = "complete/error", command = 'unknown profile'))
        # produce request
        request = copy(profile)
        request['prompt'] = request['prompt'] % msg.data.command
        # send it to server
        try:
            text, duration = self.balancer.request_available_server(json=request, decode_json=True)
        except Exception as e:
            rospy.logerr(e + f': {e.details}' if hasattr(e, 'details') else '')
            return CompleteSimpleResponse(data = TextCommand(type = "complete/error", command = e))
        # return result
        return CompleteSimpleResponse(data = TextCommand(type = "complete/response", command = text))

def main():
    n = CompleteNode()
    rospy.spin()

