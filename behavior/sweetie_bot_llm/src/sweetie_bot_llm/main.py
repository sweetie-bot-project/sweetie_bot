#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, json
import rospy
from sweetie_bot_text_msgs.srv import Complete, CompleteRequest, CompleteResponse
import requests
from rospy_message_converter import message_converter

urls = {'0': "http://127.0.0.1:5000/api/v1/generate"}

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
        except requests.ConnectionError, e:
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
        text = response['results'][0]['text'].strip().encode('utf-8')
    except UnicodeDecodeError:
        rospy.logerr('Cannot encode text! (%s)' % (r.content))
        return CompleteResponse(status="Cannot encode text!")

    rospy.loginfo('Complete response (%.2fs): "%s"' % (duration, text))
    return CompleteResponse(text=text, status='ok', duration=duration)

def complete_server():
    global urls
    n = rospy.init_node('complete_server')
    urls = rospy.get_param("~api_url", urls)
    s = rospy.Service('complete', Complete, handle_complete)
    rospy.loginfo(urls)
    rospy.loginfo("Ready to complete.")
    rospy.spin()

def main():
    complete_server()

