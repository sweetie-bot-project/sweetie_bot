#!/usr/bin/env python3
import yaml, sys

class QuestParseError(RuntimeError):
    def __init__(self, scene, segment_id, msg):
        super(QuestParseError, self).__init__("Processing scene '%s' segment '%d': %s" % (scene, segment_id, msg))

def get_attribute(desc, attribute):
    value = desc.get(attribute)
    if value is not None:
        return '^%s |%s|' % (attribute, value)
    else:
        return ''

if len(sys.argv) != 2:
    print("Usage: %s QUEST_FILE" % sys.argv[0])
    print("")
    print("Translate QUEST_FILE to soar rules and print them to standart output. QUEST_FILE should contain quest description in YAML format.")
    sys.exit(1)

with open(sys.argv[1]) as f:
    quest_desc = yaml.safe_load(f)

for scene, scene_desc in quest_desc.items():
    # check scene
    last_segment_desc = scene_desc[-1]
    if 'replica' in last_segment_desc and 'ending' not in last_segment_desc:
        raise QuestParseError(scene, len(scene_desc), "Last replica in scene must contain ending attribute.")
    # header
    print("sp {quest*elaborate*scene*%s" % scene)
    # rule lhs
    print("\t(state <s> ^top-state 1 ^beliefs.predicate <pred>)")
    print("\t(<pred> ^name quest ^scene %s ^scene-frame <f>)" % scene)
    # separator
    print("-->")
    # rule lhs: collect all elements
    segment_id = 0
    segments_rhs = ''
    segments_ids = ''
    choice_id = 0
    choices_rhs = ''
    choices_ids = ''
    question_detected = False
    for segment_desc in scene_desc:
        next_segemnt_id = min(segment_id+1, len(scene_desc)-1)
        wme_id = "<s%d>" % segment_id

        if 'replica' in segment_desc:
            # process replica
            optional_attributes = str.join(' ', [ get_attribute(segment_desc, attr) for attr in ['animation-tag', 'character', 'ending'] ])
            segments_rhs += "\t(%s ^id %d ^type reaction %s ^next-id %d)\n" % (wme_id, segment_id, optional_attributes, next_segemnt_id)
            segments_rhs += "\t(%s ^text |%s|)\n" % (wme_id, segment_desc['replica'])
            if 'sound' in segment_desc:
                segments_rhs += "\t(%s ^sound |%s|)\n" % (wme_id, segment_desc['sound'])

        elif 'question' in segment_desc:
            question_detected = True
            # process question
            optional_attributes = str.join(' ', [ get_attribute(segment_desc, attr) for attr in ['animation-tag', 'character'] ])
            segments_rhs += "\t(%s ^id %d ^type question %s ^ignore-id %d)\n" % (wme_id, segment_id, optional_attributes, next_segemnt_id)
            segments_rhs += "\t(%s ^text |%s|)\n" % (wme_id, segment_desc['question'])
            if 'sound' in segment_desc:
                segments_rhs += "\t(%s ^sound |%s|)\n" % (wme_id, segment_desc['sound'])
            # process choices
            choices_desc = segment_desc.get('choices')
            if choices_desc is not None:
                if choice_id != 0:
                    raise QuestParseError(scene, segment_id, "Only one choices is supported. It is shared for all questions.")
                # add choices
                for answer, next_scene in choices_desc.items():
                    if next_scene not in quest_desc:
                        raise QuestParseError(scene, segment_id, "Unknown scene name '%s' in choice '%s'." % (next_scene, answer))
                    choices_rhs += "\t(<c%d> ^answer-topic %s ^next-scene %s)\n" % (choice_id, answer, next_scene)
                    # append segement wme id to list
                    choices_ids += ' <c%d>' % choice_id
                    # increase segment id
                    choice_id += 1

        else:
            raise QuestParseError(scene, segment_id, "Scene segment must be replica or question")

        # append segement wme id to list
        segments_ids += ' ' + wme_id
        # increase segment id
        segment_id += 1

    if question_detected and choice_id == 0:
        raise QuestParseError(scene, 0, "The question is provided but no choices are presented.")

    # construct rhs
    print("\t(<f> ^segment %s)" % segments_ids)
    print(segments_rhs[:-1])
    if choice_id != 0:
        print("\t(<f> ^choice %s)" % choices_ids)
        print(choices_rhs[:-1])
    # finish rule
    print("}\n")


