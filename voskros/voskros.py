#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# Copyright 2023 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import rclpy, json, argparse, queue
from rclpy.node import Node
from std_msgs.msg import Bool, String
from vosk import Model, KaldiRecognizer
import sounddevice as sd

class VoskNode(Node):

    def __init__(self):
        super().__init__('voskros')

        self.declare_parameter('device', '')
        self.declare_parameter('model', 'en-us')
        self.declare_parameter('samplerate', 0)
        self.declare_parameter('supress', False)

        self.device     = self.get_parameter('device').get_parameter_value().string_value
        self.model      = self.get_parameter('model').get_parameter_value().string_value
        self.samplerate = self.get_parameter('samplerate').get_parameter_value().integer_value
        self.supress    = self.get_parameter('supress').get_parameter_value().bool_value
        
        self.sub_supress = self.create_subscription(Bool, 'supress', self.supress_callback, 10)
        self.pub_result = self.create_publisher(String, 'result', 10)
        self.pub_parcial = self.create_publisher(String, 'partial', 10)

        # Startup Vosk

        self.queue = queue.Queue()

        try:
            if not self.device: self.device = None

            if not self.samplerate:
                device_info = sd.query_devices(self.device, "input")
                # soundfile expects an int, sounddevice provides a float:
                self.samplerate = int(device_info["default_samplerate"])
                
            if not len(self.model):
                self.model = Model(lang="en-us")
            else:
                self.model = Model(lang=self.model)

            with sd.RawInputStream(samplerate=self.samplerate, blocksize = 8000, device=self.device,
                    dtype="int16", channels=1, callback=self.callback):

                rec = KaldiRecognizer(self.model, self.samplerate)

                while True:

                    rclpy.spin_once(self, timeout_sec=0.01)

                    data = self.queue.get()
                    if self.supress:
                        continue

                    if rec.AcceptWaveform(data):
                        text = json.loads(rec.Result())['text']
                        if text:
                            self.publish(self.pub_result, text)
                            self.get_logger().info(text)
                    else:
                        self.publish(self.pub_parcial, rec.PartialResult())

        except KeyboardInterrupt:
            self.get_logger().info("Ended stt")
        except Exception as e:
            self.get_logger().error(type(e).__name__ + ": " + str(e))

    def supress_callback(self, msg):
        """Set flag if wave input has to be discared."""
        self.supress = msg.data
    
    def callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            self.get_logger().info(status)
        self.queue.put(bytes(indata))

    def publish(self, pub, text):
        """Publish a single text message"""
        msg = String()
        msg.data = text
        pub.publish(msg)
        self.get_logger().debug("Pub %s: %s" % (pub.topic_name, text))


def main(args=None):

    parser = argparse.ArgumentParser(
        description='STT ROS Node. A speach to text recognizer using Vosk speech recognition toolkit.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-l', '--list', action="store_true", help='list available devices')
    parsed, remaining = parser.parse_known_args()

    if parsed.list:
        print(sd.query_devices())
        parser.exit(0)

    rclpy.init(args=args)
    n = VoskNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
