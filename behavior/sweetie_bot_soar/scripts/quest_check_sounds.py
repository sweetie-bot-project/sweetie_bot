#!/usr/bin/env python3
import sys, os
import yaml
import rospkg

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

if len(sys.argv) != 3:
    print("Usage: %s QUEST_FILE SOUND_PACKAGE" % sys.argv[0])
    print("")
    print("Check that all sound files presents in ros package SOUND_PACKAGE.")
    sys.exit(1)

with open(sys.argv[1]) as f:
    quest_desc = yaml.safe_load(f)

# collect all sounds
sounds_required = collect_segement_attributes(quest_desc, 'sound')

# get sound package path
rospack = rospkg.RosPack()
sounds_path = os.path.join(rospack.get_path(sys.argv[2]), 'sounds')
# walk and collect all .wav and *.ogg files
sounds_avaliable = set()
for currentpath, folders, files in os.walk(sounds_path):
    for file_name in files:
        basename, extension = os.path.splitext(file_name)
        if extension in ('.wav', '.ogg'):
            sounds_avaliable.add( basename )

# print missing sounds
sounds_missing = sounds_required.difference(sounds_avaliable)
if len(sounds_missing) == 0:
    print("NO MISSING SOUNDS")
else:
    print("MISSING SOUNDS:")
    for sound in sounds_missing:
        print(sound)

