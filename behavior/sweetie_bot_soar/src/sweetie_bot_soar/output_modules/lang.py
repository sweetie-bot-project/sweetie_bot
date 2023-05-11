from . import output_module 
import rospy
import actionlib
from  actionlib import GoalStatus

import spacy
from random import choice
import re

from sweetie_bot_text_msgs.srv import CompleteRaw, CompleteRawRequest, CompleteRawResponse

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
            if name_str in ['talk-heard', 'talk-said', 'talk-ignored']:
                event_type = name_str
                break
            # next attr
            idx += 1
        self.type = event_type
        # get timestamp
        self.stamp = GetChildValueAsType(item_id, 'initiated-at', float)
        # get text
        if event_type in ['talk-heard', 'talk-said']:
            self.text = GetChildValueAsType(item_id, 'text', str)
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
        if template.find('%s') >= 0:
            return template % self.text
        else:
            return template

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

class SpacyInstance:
    _spacy_models = {}
    def __new__(cls, model, *args, **kwargs):
        # check if correspondin model is loaded
        instance = cls._spacy_models.get(model)
        if instance is None:
            # load model
            instance = spacy.load(model, *args, **kwargs)
            cls._spacy_models[model] = instance
        # return spacy model
        return instance

class AttribRequest:
    subclass_map = {}

    def __new__(cls, type, **kwargs):
        # factory implementatipacy
        cls = AttribRequest.subclass_map[type]
        return super(AttribRequest, cls).__new__(cls)

    def __init__(self, type, prompt, stop_list = []):
        assert_param(prompt, 'prompt: must be str', allowed_types=str)
        self.prompt = prompt
        assert_param(stop_list, 'stop_list: must be list of strings', allowed_types=list, check_func=lambda v: len(v) >=1 and all( isinstance(e, str) for e in v ))
        self.stop_list = stop_list

class AttribRequestRegex(AttribRequest):

    def __init__(self, regex, **kwargs):
        super(AttribRequestRegex, self).__init__(**kwargs)
        # compile and check regexp
        self._regex = re.compile(regex)
        if self._regex.groups != len(self._regex.groupindex) or self._regex.groups == 0:
            raise ValueError('regexp (%s): must be at least one match group, all groups must be named.' % regex)

    def parse(self, text):
        m = self._regex.match(text)
        # check if match is succesfull
        if m is None:
            return None
        # parse groups and return (attr, value) pairs
        return { attr: m.group(attr) for attr in self._regex.groupindex }


AttribRequest.subclass_map.update({'regex': AttribRequestRegex})

class AttribRequestMap(AttribRequest):

    def __init__(self, map, attrib, nlp_model='en_core_web_sm', **kwargs):
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

    def parse(self, text):
        tokens = self._nlp(self.prompt + ' ' + text)
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
        return None

AttribRequest.subclass_map.update({'map': AttribRequestMap})

#
# Request to Lang model
#

class LangRequest:

    def __init__(self, prompt_header, prompt_fact_templates, attrib_requests, max_events, max_predicates, llm_profile):
        # get parameters: header
        assert_param(prompt_header, 'prompt_header: must be string', allowed_types=str)
        self._header = prompt_header
        # get parameters: profile
        assert_param(llm_profile, 'profile: must be string', allowed_types=str)
        self._llm_profile = llm_profile
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
        # get Attrib requests
        assert_param(attrib_requests, 'attrib_requests: must be list of dictionaries with requests description', allowed_types=list, check_func=lambda v: all(isinstance(e, dict) for e in v))
        self._requests = []
        try:
            for request in attrib_requests:
                self._requests.append( AttribRequest(**request) )
        except TypeError as e:
            raise KeyError('incorrect attrib_request declaraition: wissing or superfluous parameters (%s): error %s' % ([request.keys()], e))

    def perform_request(self, llm_caller, text = None, events=[], predicates=[]):
        #
        # form prompt
        #
        prompt = self._header
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
            for ev in events[-self._max_events:]:
                prompt += ev.verbolize(self._fact_templates)
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
        for request in self._requests:
            # update prompt
            prompt += request.prompt
            # send request
            req = CompleteRawRequest(prompt = prompt, profile = self._llm_profile, stop_list = request.stop_list)
            resp = llm_caller(req)
            # check result
            if resp.error_code != 0:
                success = False
                result['error_desc'] = 'LLM request has failed.'
                return success, result, prompt
            # parse result
            parse_result = request.parse(resp.result)
            if parse_result is None:
                success = False
                result['error_desc'] = 'LLM response parse error.'
            else:
                result.update( parse_result )
            # add result to prompt
            prompt += resp.result

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
        self._llm_client = rospy.ServiceProxy(service_ns, CompleteRaw, persistent=True) 
        # configuration
        requests = self.getConfigParameter(config, "requests", allowed_types=dict)
        self._requests = {}
        try:
            for req_name, req_conf in requests.items():
                self._requests[req_name] = LangRequest(**req_conf)
        except TypeError as e:
            raise KeyError('incorrect request declaraition (%s): missing or superfluous parameters: %s' % (req_name, e))
        

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

        rospy.logdebug('lang_model output module: LLM prompt: \n %s' % prompt)
        rospy.logdebug('lang_model output module: LLM result: \n %s' % result)

        # add WMEs
        for attr, value in result.items():
            cmd_id.CreateStringWME(attr, value)

        # check result
        if success:
            rospy.loginfo('lang_model output module: succeed: %s' % result)
            return 'succeed'
        else:
            rospy.loginfo('lang_model output module: failed: %s' % result['error_desc'])
            return 'error'


output_module.register("lang-model", LangModel)



