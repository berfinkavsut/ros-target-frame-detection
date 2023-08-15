#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import shutil
import numpy as np
import pyaudio
import wave
import os

from dataclasses import dataclass, asdict
from audio_peak_detectors import gradient_method

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
    input: bool = True  # input mode
    output: bool = False  # output mode

    # Default device indices are used.
    # input_device_index: int = 0
    # output_device_index: int = 0

    def to_dict(self) -> dict:
        """
        Take the stream parameters
        :return: dictionary of stream parameters
        """
        return asdict(self)


class TargetFrameDetector:
    def __init__(self):
        self.stream_params = StreamParams()
        self.data_format = 'int16'
        self.pyaudio = None
        self.stream = None
        self.wav_files = []

        # Data folder paths
        self.main_dir = os.path.dirname(os.getcwd())
        self.audio_data_dir = os.path.join(self.main_dir, r"Audio Data")
        self.image_data_dir = os.path.join(self.main_dir, r"Image Data")
        self.biopsy_data_dir = os.path.join(self.main_dir, r"Biopsy Data")

        # Biopsy folder names/paths
        self.biopsy_folder_name = None
        self.biopsy_folder_path = None

        # Audio file names/paths
        self.cur_filename = None
        self.cur_filepath = None
        self.filenames = {0: "", 1: "", 2: ""}
        self.filepaths = {0: "", 1: "", 2: ""}

        # Average peak sound duration
        self.duration_per_file = 1  # seconds

        # Counter to set the name of biopsy folders
        self.peak_counter = 0

        # Captured frame numbers in one second, i.e. frame rate
        self.frame_num = 30

        # By observation, one biopsy shot sound lasts for maximum 1 second
        self.peak_time = 1

        # Subscriber is a member variable of TargetFrameDetector object
        self.audio_subscriber = rospy.Subscriber("audio", String, self.audio_callback)

    def audio_callback(self, data):
        """Callback method of the Subscriber.
        :param data: String message of the audio filename
        """

        self.cur_filename = str(data.data)

        # File #0 and #1 are used for target frame detection.
        # File #2 is saved for the next iterations.
        self.filenames[0] = self.filenames[1]
        self.filenames[1] = self.filenames[2]
        self.filenames[2] = self.cur_filename

        # Two conditions for an empty first audio file:
        # 1.We can be at the start of the biopsy, so the previous file will not exist.
        # 2.In the last callback iteration, peak was detected and
        # then the current file name was set to be an empty string.
        self.filepaths[0] = os.path.join(self.audio_data_dir, self.filenames[0])
        self.filepaths[1] = os.path.join(self.audio_data_dir, self.filenames[1])
        self.filepaths[2] = os.path.join(self.audio_data_dir, self.filenames[2])

        print("Start processing...")

        if self.filenames[0] != "":

            self.create_resources()

            # Always delete the previous frames which were taken before the first audio file
            #
            # When we come to this iteration, we already have the frame images for 3*duration_per_file,
            # which now corresponds to 90 frames. (Frame rate is 30 fps.)
            #
            # If this iteration is skipped, then deletion occurs in the next iteration.
            # This means we will have more frames with the duration of duration_per_file,
            # accounting for 30 more frames

            self.delete_frames(audio_filename=self.filenames[0])

            ############################################################################################
            # Choose your biopsy shot detection method here!
            peak_time = self.process(detection_method="gradient")
            if peak_time is not None:
                self.save_target_frames(peak_time=peak_time)
            ############################################################################################

            self.close_resources(remove_audio_file=True)

        else:
            # Skip the iteration when peak was detected at the previous iteration.
            # The aim is to avoid double counting of the peaks.
            print("Skipping this iteration!!!")

        print("Stop processing.")

    def create_resources(self):
        """Create streaming resources with PyAudio.
        """
        self.pyaudio = pyaudio.PyAudio()
        self.stream_params = StreamParams()
        self.wav_files = []
        for i in [0, 1]:
            self.open_wav_file(read_path=self.filepaths[i])
        self.stream = self.pyaudio.open(**self.stream_params.to_dict())

    def open_wav_file(self, read_path: str):
        """Open a WAV file and set the stream parameters with its specifications.
        :param read_path: Absolute path to read the WAV file.
        """
        # Get specifications from the WAV file
        wav_file = wave.open(read_path, "rb")
        self.wav_files.append(wav_file)

        # Set stream parameters with respect to the WAV file
        self.stream_params.rate = wav_file.getframerate()
        self.stream_params.channels = wav_file.getnchannels()
        self.stream_params.format = self.pyaudio.get_format_from_width(wav_file.getsampwidth())

    def close_resources(self, remove_audio_file=True) -> None:
        """Close WAV file, stop Stream object and close PyAudio interface.
        :param remove_audio_file: flag for deletion of previous audio files
        """

        for i in [0, 1]:
            self.wav_files[i].close()

        if remove_audio_file is True:
            os.remove(self.filepaths[0])

            # If peak is detected, also remove the second audio file
            # Next iteration will be skipped, and it will not be used in the following iteration
            if self.filenames[1] == "":
                os.remove(self.filepaths[1])

        self.stream.stop_stream()
        self.stream.close()
        self.pyaudio.terminate()

    def process(self, detection_method: str):
        """ Process the audio sample and find the target frames of the audio.
        :param detection_method: the choice of the detection method
        :return the detected target frames and the corresponding magnitudes
        """

        window_frames = self.wav_files[0].getnframes()
        window_chunks = int(window_frames / self.stream_params.frames_per_buffer)
        half_window_frames = int(window_frames * 0.5)  # it should be an even number anyway
        total_frames = window_frames * 2

        audio_data = np.empty(total_frames, dtype=self.data_format)
        for i in [0, 1]:
            start_chunk = window_chunks * i
            end_chunk = window_chunks * (i+1)
            for j in range(start_chunk, end_chunk):
                data_str = self.wav_files[i].readframes(self.stream_params.frames_per_buffer)
                data_int = np.frombuffer(data_str, self.data_format)
                audio_data[j * self.stream_params.frames_per_buffer: (j + 1) * self.stream_params.frames_per_buffer] = data_int
                # self.stream.write(data_str)  # working fine, but not necessary!

        if detection_method == "gradient":
            peak_frames = self.stream_params.rate * self.peak_time
            peak_frame_pos = gradient_method(audio_data[half_window_frames:-1], peak_frames)
        # elif detection_method is "method_name":
            # output = method(audio_data)
        # ...

        if peak_frame_pos is not None:
            peak_time = peak_frame_pos / self.stream_params.rate

            # This file will not be used again,
            # since we will skip the next iteration!
            self.filenames[1] = ""

            return peak_time

        return None

    def delete_frames(self, audio_filename: str):
        """Delete the frames that were captured before the starting time of the first audio file.
        :param audio_filename: first audio filename that having the timestamp
        """

        # Take the timestamp from the audio filename
        start_time_str = audio_filename[:-4]
        start_time = self.from_rostime_to_float(time_str=start_time_str)

        # Images are listed with ascending time
        image_list = os.listdir(self.image_data_dir)
        for image_filename in sorted(image_list):
            old_filepath = os.path.join(self.image_data_dir, image_filename)
            os.remove(old_filepath)

            # Take the timestamp from the frame image filename
            image_time_str = image_filename[:-4]
            image_time = self.from_rostime_to_float(image_time_str)

            if image_time > start_time:
                break

    def save_target_frames(self, peak_time: int):
        """Save the target frames around the biopsy shot time.

        Save maximum 30 frames, since frame rate is 30 fps.
        The frame images from the starting/end times might be lost during time comparison,
        since we cannot have exact time step of 1 second, and
        frame capture and audio capture processes cannot start exactly at the same time.

        :param peak_time: global peak time wrt the ROS clock
        """

        # We check for peak detection from the half of the first audio file
        # The aim is to be able to detect the peak sound even if it occurs at the start of the sampled audio file
        data_time_str = self.filenames[0][:-4]
        data_time = self.from_rostime_to_float(time_str=data_time_str)  # float(data_time_str.replace("_", "."))
        half_audio_time = float(self.duration_per_file) * 0.5

        # Compute the peak time wrt to the global ROS clock
        global_peak_time = data_time + half_audio_time + peak_time
        [secs, msecs] = self.from_float_to_rostime(time_float=global_peak_time)

        # Make the folder of biopsy shot with peak time
        self.peak_counter += 1
        self.biopsy_folder_name = ''.join(["Shot_", str(self.peak_counter), '_', str(secs), '_', str(msecs).rjust(3, '0')])
        self.biopsy_folder_path = os.path.join(self.biopsy_data_dir, self.biopsy_folder_name)
        os.mkdir(self.biopsy_folder_path)

        start_time = global_peak_time - self.peak_time / 2
        end_time = global_peak_time + self.peak_time / 2

        iter_num = 0
        image_list = os.listdir(self.image_data_dir)
        for image_filename in sorted(image_list):
            image_time_str = image_filename[:-4]
            image_time = self.from_rostime_to_float(image_time_str)

            if image_time <= start_time:
                continue

            if (image_time > start_time) and (image_time <= end_time):
                source = os.path.join(self.image_data_dir, image_filename)
                destination = os.path.join(self.biopsy_folder_path, image_filename)
                shutil.move(source, destination)
                iter_num += 1

            elif image_time > end_time:
                break

            if iter_num == self.frame_num:
                break

    def from_rostime_to_float(self, time_str: str):
        """Convert timestamp string from filename to float numbers.
        :param time_str: string of timestamp
        :return: float time
        """
        time_float = float(time_str.replace("_", "."))
        return time_float

    def from_float_to_rostime(self, time_float: float):
        """Split float time to seconds and miliseconds.
        :param time_float: float time
        :return: list of second and milisecond
        """
        secs = int(time_float)
        msecs = int((time_float - secs) * 1e+3)
        return [secs, msecs]


if __name__ == '__main__':

    # ROS initialization
    rospy.init_node('target_frame_detection_node', anonymous=True)

    # Subscriber is a member variable of the TargetFrameDetector object.
    target_frame_detector = TargetFrameDetector()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


