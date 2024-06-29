# STT ROS Package

An out of the box speach to text recognizer using [Vosk speech recognition toolkit](https://alphacephei.com/vosk/).
It works offline, does not rely on external services and supports multiple languages.


## Installation Prerequisites

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install the below dependencies.

See also recommended version information on the [Vosk website](https://alphacephei.com/vosk/).

```bash
pip3 install vosk
pip3 install sounddevice
# maybe these are also needed
sudo apt-get install libportaudio2
sudo apt-get install libasound-dev
```

## Setup Package ##

```bash
# run in your ros2_ws/src folder
git clone https://gitlab.com/bob-ros2/voskros.git
cd ..
colcon build
. install/setup.bash
```

## Usage

```bash
# get device list
ros2 run voskros vosk -l

# run node with default parameter
ros2 run voskros vosk

# run via launch file
ros2 launch voskros voskros.launch.yaml model:=fr

# run via launch and overide result output topic
ros2 launch voskros voskros.launch.yaml result:=/tts_topic
```

## Models

During first startup Vosk downloads a small sized default model around 50MB training data. It works quite good on small sized devices. Other custom models can be placed in the cache folder.

```bash
# list existing models
ls ~/.cache/vosk
```

Find further models and languages here: https://alphacephei.com/vosk/models

## Node VOSK

### Node Parameter

> ~device (string, default: "")\
Device to use as input microphone. Leave it empty to use the default input.

> ~model (string, default: "en")\
Model to be used. 

> ~samplerate (int, default: 0)\
Sample rate to use.

### Published Topics

> ~result (std_msgs/String)\
Detected result text.

> ~partial (std_msgs/String)\
Detected partial text.

### Services

> ~set_grammar (srv/SetGrammar)\
Set Vosk grammar list to only allow certain words. See also [srv/SetGrammar.srv](srv/SetGrammar.srv) for details.

## Node PROMPTER

This very simple ROS Node can be used to receive an input command (e.g. from Vosk). If this command matches with one of the rules in the config file the according command will be executed.

### Usage

```bash
# start Vosk if not yet running
# in order to work a microphone must be connected or another input device must be configured
ros2 launch voskros voskros.launch.yaml

# if needed reduce the grammar according to the config in prompter.yaml
# setting the grammar greatly improves the STT detection
ros2 run voskros set_grammar.sh <path>/prompter.yaml /stt

# start prompter node and wait for commands to be executed
ros2 run voskros prompter --ros-args \
    -r input:=/stt/result \
    -r __ns:=/stt \
    -p yaml:=<path>/prompter.yaml

```

### Node Parameter

> ~yaml (string, default: "")\
Path to a YAML file containing the prompter configuration.\
See config directory for an example.

> ~cwd (string, default: "")\
Working directory where to start programms from the configuration. 

### Subscribed Topics

> ~input (std_msgs/String)\
Input topic for incommning commands to identify.

### Published Topics

> ~output (std_msgs/String)\
Output topic containing stdout produced by the called program.

## Helper set_grammar.sh
```bash
$ ros2 run voskros set_grammar.sh
Usage: set_grammar.sh <prompter config> [<namespace>]
Produces a JSON Grammar array for vosk from a prompter config yaml.
It calls afterwards the vosk node service set_grammar if a namespace is provided.
The Grammar can also be configured using a vosk node parameter.
```

## License

[Apache2.0](https://www.apache.org/licenses/LICENSE-2.0)