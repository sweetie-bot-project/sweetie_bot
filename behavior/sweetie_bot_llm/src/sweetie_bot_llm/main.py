#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, json
import rospy
from sweetie_bot_text_msgs.srv import Complete, CompleteRequest, CompleteResponse
from sweetie_bot_text_msgs.srv import CompleteSimple, CompleteSimpleRequest, CompleteSimpleResponse
import requests
from rospy_message_converter import message_converter

urls = {'0': "http://127.0.0.1:5000/api/v1/generate"}
profiles = {}

def handle_complete(req_message):
    global urls
    request = message_converter.convert_ros_message_to_dictionary(req_message)
    r = None
    for n in sorted(urls):
        rospy.loginfo("Send %d bytes to %s" % (len(json.dumps(request)), urls[n]))
        try:
            start = time.time()
            r = None
            rospy.loginfo(request)
            r = requests.post(urls[n], json=request)
            duration = time.time() - start
            if r.status_code == 200:
                break
            else:
                rospy.logerr("%d %s" % (r.status_code, r.reason))
                continue # next url
        except requests.ConnectionError as e:
            rospy.logerr("Connection failed! %s" % e)
            # nexu url

    if r is None:
        return CompleteResponse(status="All API URLs are down!")

    try:
        response = r.json()
    except ValueError:
        rospy.logerr('Cannot decode response (%s)' % (r.content))
        return CompleteResponse(status='Cannot decode response')

    if ( 'results' not in response or
      len(response['results'])==0 or
      'text' not in response['results'][0]):
        rospy.logerr('Wrong fields in response! (%s)' % (r.content))
        return CompleteResponse(status="Wrong fields in response!")

    try:
        text = response['results'][0]['text'].encode('utf-8').strip()
    except UnicodeDecodeError:
        rospy.logerr('Cannot encode text! (%s)' % (r.content))
        return CompleteResponse(status="Cannot encode text!")

    rospy.loginfo('Complete response (%.2fs): "%s"' % (duration, text.decode()))
    return CompleteResponse(text=text.decode(), status='ok', duration=duration)

def handle_text(msg):
    global profiles
    resp = CompleteSimpleResponse()
    if msg.data.type != 'complete/request':
        resp.data.type = "complete/error"
        resp.data.command = 'Wrong command type, try complete/request'
        return resp

    language = msg.data.options
    if language in profiles:
        comp_req = CompleteRequest(**profiles[language])
        comp_req.prompt = comp_req.prompt % msg.data.command
        comp_res = handle_complete(comp_req)
        resp.data.type = "complete/response"
        resp.data.command = comp_res.text
        resp.data.options = msg.data.options # assume llm respnse the same language as request
    else:
        resp.data.type = "complete/error"
        resp.data.command = 'No such profile'
        resp.data.options = msg.data.options
    return resp

def complete_server():
    global urls
    global profiles
    n = rospy.init_node('complete_server')
    urls = rospy.get_param("~api_url", urls)
    profiles = rospy.get_param("~profile", {})
    s = rospy.Service('complete', Complete, handle_complete)
    s = rospy.Service('input_text', CompleteSimple, handle_text)
    rospy.loginfo(urls)
    rospy.loginfo("Ready to complete.")
    rospy.spin()

def main():
    complete_server()

