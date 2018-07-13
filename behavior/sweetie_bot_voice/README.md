Sweetie Bot voice package 
=========================

`voice` node
------------------

This node listen to `TextCommand` mesages on `control` topic and performs voice-related commands.
It may play sound using ROS `sound_play` node or by invoking system command.

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

$### Parameters

* `~playback_command` --- if set use provided system command to play sound files.
* `lang`, (`string`, default "ru,en") --- use provided language prefixes. `voice` search sounds files in corresponding subdirs under `sound_path`. 
    First prefix has precedence over the last.
* `sounds_path` (`string`) --- sounds location (full path).

