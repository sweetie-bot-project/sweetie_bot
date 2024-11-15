from . import output_module
import rospy
import actionlib
from  actionlib import GoalStatus

import numpy as np
from random import choice
from datetime import datetime
import re
from string import Formatter
from ..nlp import SpacyInstance

from sweetie_bot_text_msgs.srv import CompleteRaw, CompleteRawRequest, CompleteRawResponse
from sweetie_bot_text_msgs.srv import Classification, ClassificationRequest, ClassificationResponse

#
# WME helpers
#

class WMEParseError(RuntimeError):
    def __init__(self, *args, **kwargs):
        super(WMEParseError, self).__init__(*args, **kwargs)


def GetChildValueAsType(parent_id, attrib, expected_type):
    # get child WME
    wme_id = parent_id.FindByAttribute(attrib, 0)
    if wme_id is None:
        raise WMEParseError('missing %s attribute' % attrib)
    # convert type
    if expected_type is str:
        wme_id = wme_id.ConvertToStringElement()
    elif expected_type is float:
        wme_id = wme_id.ConvertToFloatElement()
    elif expected_type is int:
        wme_id = wme_id.ConvertToIntElement()
    else:
        raise TypeError('GetChildValueAsType: incompatible type %s' % expected_type)
    if wme_id is None:
        raise WMEParseError('incorrect %s attribute: expected is type %s' % (attrib, expected_type))
    # get value
    return wme_id.GetValue()


#
# Ouput link items parsing
#

class TalkEvent():
    formatter = Formatter()

    def __init__(self, item_id):
        # get event type
        idx = 0
        event_type = None
        while True:
            # get name attr
            name_id = item_id.FindByAttribute('name', idx)
            if name_id is None:
                raise WMEParseError('unknown event type (missing or unknown ^name attribute)')
            # check it
            name_str = name_id.GetValueAsString()
            if name_str in ['talk-heard', 'talk-said', 'talk-ignored', 'talk-no-answer', 'talk-illegible']:
                event_type = name_str
                break
            # next attr
            idx += 1
        self.type = event_type
        # get timestamp
        self.stamp = GetChildValueAsType(item_id, 'initiated-at', float)
        # get text
        if event_type == 'talk-heard':
            self.text = GetChildValueAsType(item_id, 'text', str)
        elif event_type == 'talk-said':
            self.text = GetChildValueAsType(item_id, 'text', str)
            emotion_id = item_id.FindByAttribute('emotion', 0)
            if emotion_id is not None:
                self.emotion = emotion_id.GetValueAsString()
            else:
                self.emotion = 'neutral'
        else:
            self.text = None

    def verbolize(self, templates):
        # get template
        template = templates.get(self.type)
        if template is None:
            raise KeyError('Unknown talk event type: %s (%s)' % (self.type, self.text))
        # get random template
        if isinstance(template, list):
            template = choice(template)
        # use template to generate verbolization
        result = ''
        fmts = TalkEvent.formatter.parse(template)
        for fmt in fmts:
            literal_text, field_name, _, _ = fmt
            result += literal_text
            if field_name is None:
                break
            if field_name == 'text':
                result += self.text
            elif field_name == 'emotion':
                result += self.emotion
        return result

class Predicate():
    def __init__(self, item_id):
        # get timestamp
        self.stamp = GetChildValueAsType(item_id, 'initiated-at', float)
        # get text
        self.text = GetChildValueAsType(item_id, 'text', str)

    def verbolize(self, templates):
        return self.text

#
# Requests to lang model
#

def assert_param(value, error_desc, allowed_types = None, check_func = None):
    # check if parameter type correct
    if allowed_types is not None and not isinstance(value, allowed_types):
        raise TypeError(error_desc)
    # check if parameter value correct
    if check_func is not None and not check_func(value):
        raise ValueError(error_desc)

class AttribRequest:
    subclass_map = {}

    def __new__(cls, type, **kwargs):
        # factory implementatipacy
        cls = AttribRequest.subclass_map[type]
        return super(AttribRequest, cls).__new__(cls)

    def __init__(self, type, prompt = None, stop_list = [], temperature = -1):
        if prompt is not None:
            assert_param(prompt, 'prompt: must be str or None', allowed_types=(str,))
        self.prompt = prompt
        assert_param(stop_list, 'stop_list: must be list of strings', allowed_types=list, check_func=lambda v: all( isinstance(e, str) for e in v ))
        self.stop_list = stop_list
        self.temperature = temperature

class AttribRequestRegex(AttribRequest):

    def __init__(self, regex, **kwargs):
        super(AttribRequestRegex, self).__init__(**kwargs)
        # compile and check regexp
        self._regex = re.compile(regex)
        if self._regex.groups != len(self._regex.groupindex) or self._regex.groups == 0:
            raise ValueError('regexp (%s): must be at least one match group, all groups must be named.' % regex)

    def parse(self, text, **kwargs):
        m = self._regex.match(text)
        # check if match is succesfull
        if m is None:
            return None
        # parse groups and return (attr, value) pairs
        # TODO: why group can be None?
        result = {}
        for attr in self._regex.groupindex:
            val = m.group(attr)
            print('attr: ', attr, ' vaule: ', val)
            if m.group(attr) is not None:
                result[attr] = val
            else:
                return None
        return result
        #return { attr: m.group(attr) for attr in self._regex.groupindex }

AttribRequest.subclass_map.update({'regex': AttribRequestRegex})

class AttribRequestRegexTest(AttribRequest):

    def __init__(self, regex, attrib, value_match = None, value_nomatch = None, **kwargs):
        super(AttribRequestRegexTest, self).__init__(**kwargs)
        # compile and check regexp
        self._regex = re.compile(regex)
        # check other attribs
        assert_param(attrib, 'attrib: must be string', allowed_types=str)
        self._attrib = attrib
        if value_match is not None:
            assert_param(value_match, 'value_match: must be string or none', allowed_types=(str,))
        self._value_match = value_match
        if value_nomatch is not None:
            assert_param(value_match, 'value_nomatch: must be string or none', allowed_types=(str,))
        self._value_nomatch = value_nomatch
        if value_nomatch is None and value_nomatch is None:
            raise TypeError('value_match or value_nomatch cannot be both None')

    def parse(self, text, **kwargs):
        m = self._regex.match(text)
        # check if match is succesfull
        if m is None:
            return { self._attrib: self._value_nomatch }
        else:
            return { self._attrib: self._value_match }

AttribRequest.subclass_map.update({'regex-test': AttribRequestRegexTest})

class AttribRequestMap(AttribRequest):

    def __init__(self, map, attrib, leading_text = '', fallback_value = None, nlp_model='en_core_web_sm', **kwargs):
        super(AttribRequestMap, self).__init__(**kwargs)
        # load spacy
        self._nlp = SpacyInstance(nlp_model)
        # process map: construct key to words map
        try:
            self._map = {}
            for key, words in map.items():
                for word in words:
                    self._map[word.lower()] = key
        except (TypeError, AttributeError):
            raise TypeError('map: must be dict which maps strings to list of strings')
        # save attrib name
        assert_param(attrib, 'attrib: must be string', allowed_types=str)
        self._attrib = attrib
        assert_param(leading_text, 'leading_text: must be string', allowed_types=str)
        self._leading_text = leading_text
        if fallback_value is not None:
            assert_param(fallback_value, 'fallback_value: must be string', allowed_types=str)
        self._fallback_value = fallback_value

    def parse(self, text, **kwargs):
        tokens = self._nlp(self._leading_text + ' ' + text)
        # process parsed tokens and find key
        for idx in range(len(tokens)):
            token = tokens[idx]
            key = self._map.get(token.lemma_)
            if key is not None:
                # check negation
                # TODO: proper token tree processing
                if idx > 0 and tokens[idx].lemma_ == 'not':
                    continue
                # succesfull parsing
                return { self._attrib: key }
        # failure
        if self._fallback_value is None:
            return None
        else:
            return { self._attrib: self._fallback_value }

AttribRequest.subclass_map.update({'map': AttribRequestMap})

class AttribRequestClassification(AttribRequest):

    def __init__(self, merge_map, attrib, leading_text = '', fallback_value = None, classification_model='bert_sentiment', **kwargs):
        super(AttribRequestClassification, self).__init__(**kwargs)
        # connect classification service
        # TODO: Add model selection
        self._classificaion_client = rospy.ServiceProxy("/classification", Classification)
        # process map: construct key to words map
        try:
            self._merge_map = merge_map
        except (TypeError, AttributeError):
            raise TypeError('merge_map: must be dict which maps strings to list of strings')
        # save attrib name
        assert_param(attrib, 'attrib: must be string', allowed_types=str)
        self._attrib = attrib
        assert_param(leading_text, 'leading_text: must be string', allowed_types=str)
        self._leading_text = leading_text
        if fallback_value is not None:
            assert_param(fallback_value, 'fallback_value: must be string', allowed_types=str)
        self._fallback_value = fallback_value
        self._classification_noise_threshold = 0.1

    def merge_emotions(self, llm_emotion, classificator_emotion):
        consistent_emotions = self._merge_map[llm_emotion]
        if classificator_emotion in consistent_emotions:
            selected_emotion = classificator_emotion
        else:
            selected_emotion = llm_emotion

        return selected_emotion

    def parse(self, text, **kwargs):
        llm_emotion = kwargs.get('llm_emotion', self._fallback_value)
        last_heard_phrase = kwargs.get('last_heard_phrase', None)
        llm_response = kwargs.get('llm_response', None)

        # classify phrase with additional sentiment analyzer
        most_probable_emotion = self._fallback_value
        if last_heard_phrase is not None and llm_response is not None:
            phrase_with_response = f'{last_heard_phrase} \nMy response: {llm_response}'
            req = ClassificationRequest(text = phrase_with_response)
            emotion_resp = self._classificaion_client(req)
            if emotion_resp.probabilities[0] > self._classification_noise_threshold:
                most_probable_emotion = emotion_resp.classes[0].lower()

            merged_emotion = self.merge_emotions(llm_emotion, most_probable_emotion)
            print('emotion classification: ', dict(zip(emotion_resp.classes, emotion_resp.probabilities)))

            return { 'emotion': merged_emotion,
                     self._attrib: merged_emotion }
        # failure
        if self._fallback_value is None:
            return None
        else:
            return { self._attrib: self._fallback_value }

AttribRequest.subclass_map.update({'classification': AttribRequestClassification})

#
# Request to Lang model
#

class LangRequest:

    def __init__(self, prompt_header_names, prompt_fact_templates, attrib_requests, max_events, max_predicates, llm_profile_name, selection_cooldown_sec = None, header_change_probability = None, start_event_history_with_heard = False):
        # get parameters: header
        assert_param(prompt_header_names, 'prompt_header_names: must be list', allowed_types=list)
        self._header_names = prompt_header_names

        self._headers = {}
        for header_name in self._header_names:
            prompt_parameter = f'/lang_model/prompts/{header_name}'
            try:
                self._headers[header_name] = rospy.get_param(prompt_parameter)
            except KeyError:
                raise KeyError(f'prompt_header_name: requested prompt {header_name} is missing from parameter server. Load it into {header_name}.txt file from the prompt directory.')

        # Selecting first provided system prompt as default
        self._current_header_name = self._header_names[0]
        self._selected_header = self._headers[self._current_header_name]
        self._selection_cooldown_sec = header_change_probability or 0 #60
        self._header_change_probability = header_change_probability or 0.5
        self._last_header_update_time = datetime.now()

        # get parameters: profile
        assert_param(llm_profile_name, 'profile name: must be string', allowed_types=str)
        self._llm_profile_name = llm_profile_name
        # get parameters: fact templates
        assert_param(prompt_fact_templates, 'prompt_fact_templates: must be dictionary', allowed_types=dict)
        for k, v in prompt_fact_templates.items():
            if not isinstance(k, str) or not (isinstance(v, str) or (isinstance(v, list) and all(isinstance(e, str) for e in v))):
                raise ValueError('prompt_fact_templates: keys must be strings, their values must be strings or lists of strings.')
        self._fact_templates = prompt_fact_templates
        # get number of facts
        assert_param(max_events, 'max_events: must be nonegative integer', allowed_types=int, check_func=lambda v: v >= 0)
        self._max_events = max_events
        assert_param(max_predicates, 'max_events: must be nonegative integer', allowed_types=int, check_func=lambda v: v >= 0)
        self._max_predicates = max_predicates
        assert_param(start_event_history_with_heard, 'start_event_histiry_with_heard: must be bool', allowed_types=bool)
        self._start_event_history_with_heard = start_event_history_with_heard
        # get Attrib requests
        assert_param(attrib_requests, 'attrib_requests: must be list of dictionaries with requests description', allowed_types=list, check_func=lambda v: all(isinstance(e, dict) for e in v))
        self._requests = []
        try:
            for request in attrib_requests:
                self._requests.append( AttribRequest(**request) )
        except TypeError as e:
            # raise KeyError('incorrect attrib_request declaraition: wissing or superfluous parameters (%s): error %s' % ([request.keys()], e))
            raise e
        # check attrib requests
        if len(self._requests) == 0 or self._requests[0].prompt is None:
            raise ValueError('incorrect attrib_request declaraition: at leat one request should present, first request should contain prompt field.')

    def try_change_header(self, events, predicates):
        elapsed_after_update = datetime.now() - self._last_header_update_time
        if elapsed_after_update.total_seconds() > self._selection_cooldown_sec:
            if np.random.uniform() < self._header_change_probability:
                # Chaning prompt on random, but different one
                name_pool = [n for n in self._header_names if n != self._current_header_name]
                if len(name_pool) == 0:
                    return events, predicates

                new_name_choice = np.random.choice(len(name_pool), size=1, replace=False).item()
                self._current_header_name = name_pool[new_name_choice]
                self._selected_header = self._headers[self._current_header_name]

                self._last_header_update_time = datetime.now()

                # return tuple(), tuple()

        return events, predicates

    def perform_request(self, llm_caller, events, predicates, text = None):
        #
        # form prompt
        #
        prompt = self._selected_header
        events, predicates = self.try_change_header(events, predicates)
        # verbolize predicates
        if self._max_predicates > 0:
            # sort by timestamp
            predicates.sort(key = lambda ev: ev.stamp)
            # verbolize last max_predicates
            for pred in predicates[-self._max_predicates:]:
                prompt += pred.verbolize(self._fact_templates)
        # verbolize events
        if self._max_events > 0:
            # sort by timestamp
            events.sort(key = lambda ev: ev.stamp)
            # verbolize last max_events
        last_events = events[-self._max_events:]
        if self._start_event_history_with_heard:
            while len(last_events) >= 1 and last_events[-1].type == 'talk-said':
                last_events.pop(0)

        for ev in last_events:
            prompt += ev.verbolize(self._fact_templates)

        # Save last human phrase for later processing
        last_heard_phrase = None
        if len(last_events) > 0:
            last_heard_phrase = ev.text

        # add text
        if text is not None:
            template = self._fact_templates['text']
            if template is None:
                prompt += text
            else:
                # get random template
                if isinstance(template, list):
                    template = choice(template)
                # use template
                prompt += template % text
        #
        # perform requests
        #
        result = {}
        success = True
        kwargs = {}
        try:
            for request in self._requests:
                # request lang model if prompt is present
                if request.prompt is not None:
                    # update prompt
                    prompt += request.prompt
                    # fix new lines: must be \r\n
                    prompt = prompt.replace('\n', '\r\n')
                    # send request
                    req = CompleteRawRequest(prompt = prompt, profile_name = self._llm_profile_name, stop_list = request.stop_list)
                    resp = llm_caller(req)
                    # check result
                    if resp.error_code != 0:
                        success = False
                        result['error_desc'] = 'LLM request has failed.'
                        return success, result, prompt
                    # add result to prompt
                    prompt += resp.result

                # Provide simple emotion assesment from llm
                if isinstance(request, AttribRequestClassification):
                    kwargs = dict(
                        llm_emotion=result.get('emotion', None),
                        last_heard_phrase=last_heard_phrase,
                        llm_response=result.get('result', None),
                    )

                # parse result (current or prevoius)
                parse_result = request.parse(resp.result, **kwargs)
                if parse_result is None:
                    # TODO: Better handle wrong emotion responses as '1'
                    # TODO: And figure out why is there two emotion requests happening
                    # NOTE: Maybe better instruction following models would give more stable outputs? -Mike
                    success = False
                    result['error_desc'] = 'LLM response parse error.'
                else:
                    print('parse result: ', parse_result)
                    result.update( parse_result )
        except rospy.ServiceException as e:
            success = False
            result['error_desc'] = f'LLM service error: {e}.'
        except Exception as e:
            success = False
            result['error_desc'] = f'Unknown LLM error. Please, investigate it.'

        #
        # return result
        #
        return success, result, prompt


class LangModel(output_module.OutputModule):

    def __init__(self, config):
        super(LangModel, self).__init__("lang-model")
        # module initialization
        service_ns = self.getConfigParameter(config, "service_ns", allowed_types=str)
        # create Service client
        rospy.wait_for_service(service_ns, timeout=5.0)
        self._llm_client = rospy.ServiceProxy(service_ns, CompleteRaw)
        # configuration
        requests = self.getConfigParameter(config, "requests", allowed_types=dict)
        self._requests = {}
        try:
            for req_name, req_conf in requests.items():
                self._requests[req_name] = LangRequest(**req_conf)
        except TypeError as e:
            # raise KeyError('incorrect request declaraition (%s): missing or superfluous parameters: %s' % (req_name, e))
            raise e


    def startHook(self, cmd_id):
        #
        # parse command structure:
        # ( ^preicate <P1> ... ^event <E1> ... ^request "<request>" )
        #
        request_name = None
        events = []
        predicates = []
        text = None
        for idx in range(0, cmd_id.GetNumberChildren()):
            item_id = cmd_id.GetChild(idx)
            item_attr = item_id.GetAttribute()
            rospy.logdebug('lang_model output module: process cmd child %d: ^%s %s' % (idx, item_attr, item_id.GetValueAsString()))

            # get facts
            if item_id.IsIdentifier():
                try:
                    # classufy attribute
                    if item_attr == 'event':
                        events.append( TalkEvent(item_id.ConvertToIdentifier()) )
                    elif item_attr == 'predicate':
                        predicates.append( Predicate(item_id.ConvertToIdentifier()) )
                except WMEParseError as e:
                    rospy.logwarn('^%s %s parse error: %s' & (item_attr, item_id.GetValueAsString(), e))

            # get request
            if item_attr == 'request':
                request_name = item_id.GetValueAsString()
            elif item_attr == 'text':
                text = item_id.GetValueAsString()


        # get LangRequest
        if request_name is None:
            rospy.logerr("lang_model output module: no ^request is supplied.")
            return "error"

        request = self._requests.get(request_name)
        if request is None:
            rospy.logerr("lang_model output module: unknown request %s." % request_name)
            return "error"

        # perform request
        rospy.logdebug('lang_model output module: LLM request: predicates %s, events %s, text %s' % (len(predicates), len(events), text))
        # TODO use future
        success, result, prompt = request.perform_request(self._llm_client, events = events, predicates = predicates, text = text)

        rospy.logdebug('lang_model output module: LLM prompt: \n %s' % repr(prompt))
        rospy.logdebug('lang_model output module: LLM result: \n %s' % result)

        # add WMEs
        for attr, value in result.items():
            cmd_id.CreateStringWME(attr, value)

        # check result
        if success:
            rospy.loginfo('lang_model output module: succeed: %s' % result)
            return 'succeed'
        else:
            rospy.logerr('lang_model output module: failed: %s' % result['error_desc'])
            return 'error'


output_module.register("lang-model", LangModel)
