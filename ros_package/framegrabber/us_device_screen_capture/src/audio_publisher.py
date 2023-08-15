#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import pyaudio
import wave
import os

from dataclasses import dataclass, asdict

@dataclass
class StreamParams:
    """
    Class for setting the parameters of the sound interface.
    """
    format: int = pyaudio.paInt16   # sample size: pyaudio.paInt16 with [-32768, 32767] / pyaudio.paFloat32
    channels: int = 1               # channel number: 1 / 2
    rate: int = 44100               # frame rate: 48000 / 44100
    frames_per_buffer: int = 1024   # chunk: 256 / 1024

    # Set I/O modes
    input: bool = True                # input mode
    output: bool = False              # output mode

    # Default device indices are used.
    # input_device_index: int = 0
    # output_device_index: int = 0

    """You can use this code snippet to find the indices of available sound devices.
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    
    for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))
            print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i))
    """
    def to_dict(self) -> dict:
        """
        Take the stream parameters
        :return: dictionary of stream parameters
        """
        return asdict(self)


class AudioStream:
    def __init__(self):
        self.stream_params = StreamParams()
        self.data_format = 'int16'
        self.pyaudio = None
        self.stream = None
        self.wav_file = None

        # Data folder path
        self.main_dir = os.path.dirname(os.getcwd())
        self.data_dir = os.path.join(self.main_dir, r"Audio Data")

        # Audio file name
        self.filename = None

    def create_resources(self):
        """Create streaming resources with PyAudio.
        """
        self.pyaudio = pyaudio.PyAudio()
        self.stream_params = StreamParams()
        self.stream = self.pyaudio.open(**self.stream_params.to_dict())

    def create_wav_file(self, save_path: str) -> None:
        """Create a WAV file.
        :param save_path: Absolute path to save the WAV file.
        """
        self.wav_file = wave.open(save_path, "wb")
        self.wav_file.setnchannels(self.stream_params.channels)
        self.wav_file.setsampwidth(self.pyaudio.get_sample_size(self.stream_params.format))
        self.wav_file.setframerate(self.stream_params.rate)

    def read_from_stream(self, duration):
        """Read the audio from the audio stream of PyAudio.
        :param duration: audio duration
        :return: audio filename in the WAV format
        """
        # Take the starting time of the audio file
        ros_time = rospy.get_rostime()
        str_secs = str(ros_time.secs)
        str_nsecs_unfilled = str(ros_time.nsecs)
        str_nsecs = str_nsecs_unfilled.rjust(9, '0')
        str_msecs = str_nsecs[0:3]

        # String message to publish later
        filename = String()
        filename.data = ''.join([str_secs, '_', str_msecs]) + ".wav"

        # Create the WAV file with the absolute path
        filepath = os.path.join(self.data_dir, filename.data)
        self.create_wav_file(filepath)

        # Record to the WAV file
        for i in range(int(self.stream_params.rate * duration / self.stream_params.frames_per_buffer)):
            audio_data_str = self.stream.read(self.stream_params.frames_per_buffer, exception_on_overflow=False)
            self.wav_file.writeframes(audio_data_str)
        self.wav_file.close()

        return filename

    def close_resources(self) -> None:
        """Stop Stream object and close PyAudio interface.
        """
        self.stream.stop_stream()
        self.stream.close()
        self.pyaudio.terminate()


def audio_publisher():
    """ Initialize ROS inside the publisher method.
    Publish the audio filenames as long as the node is not killed.
    """

    # ROS initialization
    rospy.init_node('audio_publisher_node', anonymous=True)

    audio_pub = rospy.Publisher('audio', String, queue_size=1)

    # Peak time is observed as maximum 1 second, therefore
    # the audio files are saved with the time step of 1 second
    # to capture all the peak sound.
    duration = 1  # seconds

    # AudioStream object to create audio files
    audio_streamer = AudioStream()
    audio_streamer.create_resources()

    while not rospy.is_shutdown():
        filename = audio_streamer.read_from_stream(duration=duration)
        audio_pub.publish(filename)
        rospy.loginfo(filename)

    audio_streamer.close_resources()


if __name__ == '__main__':
    try:
        audio_publisher()
    except rospy.ROSInterruptException:
        pass
