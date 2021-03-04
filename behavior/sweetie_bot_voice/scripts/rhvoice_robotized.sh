#!/bin/sh
export LADSPA_PATH=~/.ladspa/
# For debian
export JAVA_HOME=/lib/jvm/adoptopenjdk-8-openj9-amd64/
# For ubuntu
#export JAVA_HOME=/lib/jvm/java-8-openjdk-amd64/

# fix broken ladspa support
#rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin; 
#gst-inspect-1.0 ladspa
# issue speech sinnthesis
echo $1 | RHVoice-client -r -0.2 -v 1.0 -s Anna+CLB | gst-launch-1.0 fdsrc fd=0 ! decodebin ! audioconvert ! audioresample ! ladspa-autotalent-so-autotalent \
        concert-a="440.48001"           \
        fixed-pitch="0.0"               \
        pull-to-fixed-pitch="0.384"     \
        a="0" ab="1" b="0" bb="0" c="1" \
        d="0" db="0" e="0" eb="1" f="1" \
        g="0" gb="0"                    \
        correction-strength="1"         \
        correction-smoothness="0"       \
        pitch-shift="-3.0"              \
        output-scale-rotate="5"         \
        lfo-depth="0.0"                 \
        lfo-rate="10.0"                 \
        lfo-shape="1"                   \
        lfo-symmetry="0.032"            \
        lfo-quantization="0"            \
        formant-correction="0"          \
        formant-warp="-0.286"           \
        mix="1.0"                       \
! audioconvert ! ladspa-pt-robotize-so-pt-robotize \
        mode="2"                        \
! autoaudiosink
