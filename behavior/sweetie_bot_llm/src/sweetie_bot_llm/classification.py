#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, json
import requests
from copy import copy

import rospy
from rospy_message_converter import message_converter

from sweetie_bot_load_balancer.balancer import Balancer
from sweetie_bot_text_msgs.srv import Classification, ClassificationRequest, ClassificationResponse
from sweetie_bot_text_msgs.msg import TextCommand


class ClassificationError(RuntimeError):
    def __init__(self, msg, details = ''):
        super(ClassificationError, self).__init__(msg)
        self._details = details

    @property
    def details(self):
        return self._details

class ClassificationNode:
    DEFAULT_CONFIG = dict(server_choices=dict(
        local_host = {'url': 'http://127.0.0.1:5000/classification'},
    ))

    def __init__(self):
        rospy.init_node('classification_server')
        # :ProfileParametrizationSupport
        # TODO: Add support for classification profiles
        # get profiles
        # self._profiles = rospy.get_param("~profile", {})
        # create services
        rospy.Service('classification', Classification, self.classification_callback)

        balancer_config = rospy.get_param("~balancer_config", self.DEFAULT_CONFIG)
        self.balancer = Balancer(
            balancer_config,
            loggers={'debug': rospy.logdebug, 'warn': rospy.logwarn, 'info': rospy.loginfo},
            postprocess_func=self.extract_response_probabilities,
        )

        # log
        self.log_llm = rospy.Publisher('speech_log', TextCommand, queue_size=10)
        # :ProfileParametrizationSupport
        #rospy.loginfo(f'profiles: {[self._profiles.keys()]}')

    def extract_response_probabilities(self, response):
        # check response structure
        if 'class_probs_str' not in response or len(response['class_probs_str']) == 0:
            raise ClassificationError('wrong response structure', response.get('detail', 'Unknown error'))

        # convert to dict
        try:
            probs = json.loads(response['class_probs_str'])
        except json.JSONDecodeError as e:
            raise ClassificationError('json decode error', str(e))

        return probs

    @staticmethod
    def select_most_probable_class(probs_dict):
        return max(probs_dict, key=probs_dict.get)

    @staticmethod
    def sort_probs_dict(probs_dict):
        return dict(sorted(probs_dict.items(), key=lambda it: -it[1]))

    def classification_callback(self, msg):
        # :ProfileParametrizationSupport
        # TODO: Add support for classification profiles
        # get profile
        #profile = self._profiles.get(msg.profile_name)
        #if profile is None:
        #    rospy.logerr(f'unknown profile: {msg.profile_name}')
        #    return ClassificationResponse(error_code = ClassificationResponse.UNKNOWN_PROFILE)
        # construct request
        #request = copy(profile) # shallow copy: one level is enought

        request = dict()
        request['text'] = msg.text

        self.log_llm.publish('log/cls/in/emotion', msg.text, '')
        # send it to server
        try:
            probs_dict, duration = self.balancer.request_available_server(json=request, decode_json=True)
        except Exception as e:
            rospy.logerr(f'Classification: {e.details}' if hasattr(e, 'details') else str(e))
            return ClassificationResponse(error_code = ClassificationResponse.SERVER_UNREACHABLE)

        self.log_llm.publish('log/cls/out/emotion', f'Most probable class: {self.select_most_probable_class(probs_dict)}', '')

        # return result
        probs_dict = self.sort_probs_dict(probs_dict)
        classes, probabilities = zip(*probs_dict.items())
        return ClassificationResponse(classes = classes, probabilities = probabilities, error_code = ClassificationResponse.SUCCESS)


def main():
    n = ClassificationNode()
    rospy.spin()
