#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sounddevice as sd
from scipy.io import wavfile
from scipy import signal
import os
import numpy as np
import threading

class SoundClip:
    def __init__(self, file_name, duration, amplitude=1.0, preload=False, samplerate=None):
        self.file_path = os.path.join(os.path.dirname(__file__), 'sound_clips', file_name)
        self.duration = duration
        self.amplitude = amplitude
        self.samplerate = 0
        self.clip = []
        self.channels = 1
        self.loaded = False
        self.target_samplerate = samplerate

        if preload:
            self.load_clip()

    def load_clip(self):
        rospy.loginfo(f'Loading sound clip: {self.file_path}')
        self.samplerate, self.clip = wavfile.read(self.file_path)

        self.clip = np.array(self.clip, dtype=np.float32)

        # Always keep clip in (frames, channels) format to simplify stream writes.
        rospy.loginfo(f'Clip dimension: {self.clip.ndim}');
        if self.clip.ndim == 1:
            self.clip = self.clip.reshape(-1, 1)
        elif self.clip.ndim != 2:
            rospy.logerr(f'Unsupported audio shape {self.clip.shape} for {self.file_path}')

        target_frames = int(np.floor(self.samplerate * self.duration))
        current_frames = int(self.clip.shape[0])
        if target_frames <= 0 or current_frames <= 0:
            rospy.logerr(f'Invalid clip length for {self.file_path}: target={target_frames}, current={current_frames}')
        rospy.loginfo(f'Current frames: {current_frames}');
        rospy.loginfo(f'Target frames: {target_frames}');

        # Trim the clip to the specified duration
        if current_frames >= target_frames:
            self.clip = self.clip[:target_frames]
            rospy.loginfo(f'Trimming clip to {target_frames} frames');
        else:
            repeat_count = int(np.ceil(float(target_frames) / float(current_frames)))
            self.clip = np.tile(self.clip, (repeat_count, 1))
            self.clip = self.clip[:target_frames]
            rospy.loginfo(f'Repeating clip {repeat_count} times and trimming');

        if self.target_samplerate is not None and self.target_samplerate != self.samplerate and self.clip.shape[0] > 0:
            self.clip = signal.resample_poly(self.clip, up=int(self.target_samplerate), down=int(self.samplerate), axis=0)
            self.samplerate = int(self.target_samplerate)
            rospy.loginfo(f'Resampled clip to {self.samplerate} Hz for device output');

        # Normalize the clip
        peak = np.max(np.abs(self.clip))
        if peak > 0:
            self.clip = self.amplitude * self.clip / peak
            rospy.loginfo(f'Peak was at {peak}. Normalizing peak to {self.amplitude}');
        else:
            rospy.logwarn(f'Sound clip {self.file_path} is silent; skipping normalization.')

        # self.clip = np.ascontiguousarray(self.clip, dtype=np.float32)
        self.channels = int(self.clip.shape[1])
        self.loaded = True
        rospy.loginfo('Loaded sound clip: ' + self.file_path)

class SoundGenerator:
    def __init__(self):
        rospy.loginfo('SOUND GENERATOR NODE STARTED')

        # List sound devices
        self.sound_devices = sd.query_devices()
        self.sound_devices = [device for device in self.sound_devices if 'HDMI' in device['name'] and device['hostapi']==0]
        self.device_samplerate = None

        if not self.sound_devices:
            default_output = sd.default.device[1]
            if default_output is not None and default_output >= 0:
                self.sound_devices = [sd.query_devices(default_output)]
                rospy.logwarn('No HDMI output device matched. Falling back to default output device.')
        else:
            self.device_samplerate = int(self.sound_devices[0]['default_samplerate'])

        # Clips dictionary
        self.clips = {'White_Noise': SoundClip('whitenoise.wav', duration=5, amplitude=0.5, samplerate=self.device_samplerate),
                '5KHz': SoundClip('5khz.wav', duration=5.0, amplitude=1.0, samplerate=self.device_samplerate),
                'Error': SoundClip('wrong-answer-buzz.wav', duration=1.0, amplitude=1.0, samplerate=self.device_samplerate),
                '1KHz': SoundClip('1kHz.wav', duration=0.5, amplitude=0.5, samplerate=self.device_samplerate),
                '8KHz': SoundClip('8kHz.wav', duration=0.5, amplitude=0.5, samplerate=self.device_samplerate),
                'White_Noise_Training': SoundClip('white-noise-2min.wav', duration=120.0, amplitude=1.0, samplerate=self.device_samplerate),
                '5KHz_Training': SoundClip('5khz-2min.wav', duration=120.0, amplitude=1.0, samplerate=self.device_samplerate),
                'Starting_Sound': SoundClip('1kHz.wav', duration=0.5, amplitude=0.5, samplerate=self.device_samplerate),
                '1KHz_120s': SoundClip('tone_1000Hz_120.0s.wav', duration=120.0, amplitude=0.5, samplerate=self.device_samplerate),
                '8KHz_120s': SoundClip('tone_8000Hz_120.0s.wav', duration=120.0, amplitude=0.5, samplerate=self.device_samplerate)
                        }

        self.active_streams = {}
        self.active_streams_lock = threading.Lock()
        self.device_stop_events = {}
        rospy.loginfo('Sound devices found:')
        for device in self.sound_devices:
            rospy.loginfo(device['name'])
            device_index = int(device['index'])
            self.device_stop_events[device_index] = threading.Event()
        
        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)
        
        rospy.loginfo('SoundGenerator initialized successfully.')

    def _play_clip_on_device(self, clip, device_index, stop_event):
        rospy.loginfo('Playing sound clip: ' + clip.file_path + f' on device index: {device_index}')
        frames_per_chunk = 4096

        with sd.OutputStream(device=device_index,
                             samplerate=clip.samplerate,
                             channels=clip.channels,
                             dtype='float32',
                             blocksize=1024,
                             latency='low') as stream:
            with self.active_streams_lock:
                self.active_streams[device_index] = stream
            
            for start in range(0, clip.clip.shape[0], frames_per_chunk):
                if stop_event.is_set() or rospy.is_shutdown():
                    stream.abort()
                    break

                end = min(start + frames_per_chunk, clip.clip.shape[0])
                stream.write(clip.clip[start:end])

            with self.active_streams_lock:
                if self.active_streams.get(device_index) is stream:
                    self.active_streams.pop(device_index, None)

    def sound_callback(self, msg):
        rospy.loginfo('[sound_generator] Sound received: ' + msg.data)

        clip = self.clips.get(msg.data)
        if clip:
            if not clip.loaded:
                clip.load_clip()

            for device in self.sound_devices:
                device_index = int(device['index'])
                stop_event = self.device_stop_events.get(device_index)
                if stop_event is None:
                    stop_event = threading.Event()
                    self.device_stop_events[device_index] = stop_event
                else:
                    stop_event.set()

                new_stop_event = threading.Event()
                self.device_stop_events[device_index] = new_stop_event
                playback_thread = threading.Thread(target=self._play_clip_on_device,
                                                   args=(clip, device_index, new_stop_event),
                                                   daemon=False)
                playback_thread.start()

            rospy.loginfo('[sound_generator] Started playback: ' + msg.data)
        elif msg.data == 'Stop':
            for device_index, stop_event in list(self.device_stop_events.items()):
                stop_event.set()

            with self.active_streams_lock:
                for device_index, stream in list(self.active_streams.items()):
                    try:
                        stream.abort()
                    except Exception as exc:
                        rospy.logwarn(f'[sound_generator] Failed to abort stream on {device_index}: {exc}')
        else:
            rospy.logwarn('[sound_generator] Sound command not recognized: ' + msg.data)

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'sound_generator'
    rospy.init_node('sound_generator')
    SoundGenerator()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown
