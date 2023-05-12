Sweetie Bot voice package 
=========================

`voice` node
------------------

This node listen to `TextCommand` mesages on `control` topic and performs voice-related commands.
It may play sound using ROS `sound_play` node or by invoking system command.

Prerecorded files should be located in so-called sound packages in `sounds/<lang_prefix>` directories.
The list of sound packages, lang prefixes and their precedence is controlled by `~sound_packages` and `lang`
parameters.


### Supported `TextCommand` commands types

* `voice/play_wav` --- play corresponding `.wav` or `.ogg` file. You should specify only basename in `command` filed.
* `voice/say` --- use `sound_play` speech synthesis service to pronounce phrase in `command` field.

### ROS interface

#### Publish topics

* `robotsound` (`sound_play::SoundRequest`) --- receive text commands.

#### Subscribe topics

* `control` (`TextCommand`) --- receive text commands.

#### Actionlib

* Client: `sound_play` (`sound_play::SoundRequestAction`)

### Parameters

* `~playback_command` (`string`, default None) --- if set use provided system command to play sound files.
* `~sound_packages` (`strings`, default []) --- ROS packages with sounds. First elements have precedence over lasts. 
    Sound files should be located in `<package_path>/sound/<lang_prefix>` directories. `sweetie_bot_voice` package is always appended to this list.
* `lang`, (`string`, default "ru,en") --- use provided language prefixes.  First prefix has precedence over the last.
* `~tts_backend`, (`string`, default "sound_play") --- Text-to-Speech service selection. Currently "sound_play", "rhvoice" and "rhvoice_robotized" are supported.

# Requirements

```
sudo apt-get install libcairo2-dev libgirepository1.0-dev
pip install TTS PyGObject
```
