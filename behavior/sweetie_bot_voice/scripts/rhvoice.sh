#!/bin/sh
# issue speech sinnthesis
echo $1 | RHVoice-client -r -0.2 -v 1.0 -s Anna+CLB | gst-launch-1.0 fdsrc fd=0 ! decodebin ! autoaudiosink
