#!/usr/bin/env python3
import yaml, sys

def collect_segement_attributes(quest_desc, attribute):
    values = set()
    # walk over oll segments and collect values to set
    for state_desc in quest_desc.values():
        for segment_desc in state_desc:
            value = segment_desc.get(attribute)
            if value is not None:
                values.add( value )
    # return resulting set
    return values

if len(sys.argv) != 2:
    print("Usage: %s QUEST_FILE" % sys.argv[0])
    print("")
    print("Print all 'animation-tag' used in quest.")
    sys.exit(1)

with open(sys.argv[1]) as f:
    quest_desc = yaml.safe_load(f)

# collect all tags
animation_tags = collect_segement_attributes(quest_desc, 'animation-tag')

# print tags
for animation_tag in animation_tags:
    print(animation_tag)
