# STT ROS Node

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

## Setup Node ##

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
ros2 run voskros voskros -l

# run node with default parameter
ros2 run voskros voskros

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

## Node Parameter

> ~device (string, default: "")\
Device to use as input microphone. Leave it empty to use the default input.

> ~model (string, default: "en")\
Model to be used. 

> ~samplerate (int, default: 0)\
Sample rate to use.

## Published Topics

> ~result (std_msgs/String)\
Detected result text.

> ~partial (std_msgs/String)\
Detected partial text.


## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[Apache2.0](https://www.apache.org/licenses/LICENSE-2.0)