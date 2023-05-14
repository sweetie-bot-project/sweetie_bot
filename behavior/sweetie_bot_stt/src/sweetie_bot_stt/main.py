#!/usr/bin/env python
import os
import rospy
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse
import requests

import six
from google.cloud import translate_v2 as translate

urls = {'0': "http://127.0.0.1:8577/"}

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

def handle_transcribe(req):
    global urls
    global enable_gtranslate
    if req.filename and os.path.isfile(req.filename):
        rospy.logdebug("Got filename = %s" % req.filename)
        with open(req.filename, 'rb') as file:
            req.data = file.read()

    r = None
    for n in sorted(urls):
        rospy.loginfo("Send %d bytes to %s" % (len(req.data), urls[n]))
        try:
            r = None
            r = requests.post(urls[n], files={'file': ('audio.wav', req.data)})
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

      if enable_gtranslate and response['language'] !='en':
          response['text'] = translate_text('en', response['text'])

      rospy.loginfo('Transcription %s (%.2fs): "%s"' % (response['status'],
                                                        response['transcribe_duration'],
                                                        response['text']))
    except:
      rospy.logerr('Cannot decode response (%s)' % (r.content))
      return TranscribeResponse()

    return TranscribeResponse(**response)

def transcribe_server():
    global urls
    rospy.init_node('transcribe_server')
    urls = rospy.get_param("~api_url", urls)
    enable_gtranslate = rospy.get_param("~enable_gtranslate", False)
    s = rospy.Service('transcribe', Transcribe, handle_transcribe)
    rospy.loginfo(urls)
    rospy.loginfo("Ready to transcribe.")
    rospy.spin()

def main():
    transcribe_server()

