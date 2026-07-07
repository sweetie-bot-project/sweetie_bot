# SOAR output-link WME parsing shared by BOTH lang-model backends (R6).
#
# Moved VERBATIM from lang_legacy.py so the agent backend (lang.py) no longer imports its data
# classes from the legacy module; lang_legacy re-imports them (back-compat). rospy is optional:
# only TalkEvent's missing-emotion warning uses it, and unit tests run without a ROS env.

from random import choice
from string import Formatter

try:
    import rospy
    _warn = rospy.logwarn
except ImportError:  # unit tests / headless: plain logging instead of rosout
    import logging
    _warn = logging.getLogger(__name__).warning

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
                _warn('lang output module: emotion WME is not provided.')
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
