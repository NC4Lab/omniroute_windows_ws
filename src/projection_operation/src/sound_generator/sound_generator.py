#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sounddevice as sd
from scipy.io import wavfile
import os
import numpy as np
import threading
from queue import Queue, Empty

class SoundClip:
    def __init__(self, file_name, duration, amplitude=1.0, preload = False):
        self.file_path = os.path.join(os.path.dirname(__file__), 'sound_clips', file_name)
        self.duration = duration
        self.amplitude = amplitude
        self.samplerate = 0
        self.clip = []
        self.channels = 1
        self.loaded = False

        if preload:
            self.load_clip()

    def load_clip(self):
        rospy.loginfo(f'Loading sound clip: {self.file_path}')
        self.samplerate, self.clip = wavfile.read(self.file_path)
        rospy.loginfo('Loaded sound clip: ' + self.file_path)
        rospy.loginfo('Sample rate: ' + str(self.samplerate))

        self.clip = np.array(self.clip, dtype=np.float32)

        # Always keep clip in (frames, channels) format to simplify stream writes.
        if self.clip.ndim == 1:
            self.clip = self.clip.reshape(-1, 1)
        elif self.clip.ndim != 2:
            raise ValueError(f'Unsupported audio shape {self.clip.shape} for {self.file_path}')

        target_frames = int(np.floor(self.samplerate * self.duration))
        current_frames = int(self.clip.shape[0])
        if target_frames <= 0 or current_frames <= 0:
            raise ValueError(f'Invalid clip length for {self.file_path}: target={target_frames}, current={current_frames}')

        # Trim the clip to the specified duration
        if current_frames >= target_frames:
            self.clip = self.clip[:target_frames]
        else:
            repeat_count = int(np.ceil(float(target_frames) / float(current_frames)))
            self.clip = np.tile(self.clip, (repeat_count, 1))
            self.clip = self.clip[:target_frames]

        # Normalize the clip
        peak = np.max(np.abs(self.clip))
        if peak > 0:
            self.clip = self.amplitude * self.clip / peak
        else:
            rospy.logwarn(f'Sound clip {self.file_path} is silent; skipping normalization.')

        self.clip = np.ascontiguousarray(self.clip, dtype=np.float32)
        self.channels = int(self.clip.shape[1])
        self.loaded = True

class SoundGenerator:
    def __init__(self):
        rospy.loginfo('SOUND GENERATOR NODE STARTED')

        # Clips dictionary
        self.clips = {'White_Noise': SoundClip('whitenoise.wav', duration=5, amplitude=0.5, preload=True),
                '5KHz': SoundClip('5khz.wav', duration=5.0, amplitude=1.0, preload=True),
                'Error': SoundClip('wrong-answer-buzz.wav', duration=1.0, amplitude=1.0, preload=True),
                '1KHz': SoundClip('1kHz.wav', duration=0.5, amplitude=0.5, preload=True),
                '8KHz': SoundClip('8kHz.wav', duration=0.5, amplitude=0.5, preload=True),
                'White_Noise_Training': SoundClip('white-noise-2min.wav', duration=120.0, amplitude=1.0, preload=True),
                '5KHz_Training': SoundClip('5khz-2min.wav', duration=120.0, amplitude=1.0, preload=True),
                'Starting_Sound': SoundClip('1kHz.wav', duration=0.5, amplitude=0.5, preload=True),
                '1KHz_120s': SoundClip('tone_1000Hz_120.0s.wav', duration=120.0, amplitude=0.5, preload=True),
                '8KHz_120s': SoundClip('tone_8000Hz_120.0s.wav', duration=120.0, amplitude=0.5, preload=True)
                        }

        # List sound devices
        self.sound_devices = sd.query_devices()
        self.sound_devices = [device for device in self.sound_devices if 'HDMI' in device['name'] and device['hostapi']==3]

        if not self.sound_devices:
            default_output = sd.default.device[1]
            if default_output is not None and default_output >= 0:
                self.sound_devices = [sd.query_devices(default_output)]
                rospy.logwarn('No HDMI output device matched. Falling back to default output device.')

        self.device_workers = {}
        self.active_streams = {}
        self.active_streams_lock = threading.Lock()
        rospy.loginfo('Sound devices found:')
        for device in self.sound_devices:
            rospy.loginfo(device['name'])
            device_index = int(device['index'])
            work_queue = Queue()
            stop_event = threading.Event()
            worker = threading.Thread(target=self._device_worker, args=(device_index, work_queue, stop_event), daemon=True)
            worker.start()
            self.device_workers[device_index] = {'queue': work_queue, 'thread': worker, 'stop_event': stop_event}
        
        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)
        
        rospy.loginfo('SoundGenerator initialized successfully.')

    def _clear_queue(self, work_queue):
        while True:
            try:
                work_queue.get_nowait()
                work_queue.task_done()
            except Empty:
                break

    def _play_clip_on_device(self, clip, device_index, stop_event):
        if not clip.loaded:
            rospy.logwarn(f'Sound {clip.file_path} not preloaded, loading now...')
            clip.load_clip()

        sd.check_output_settings(device=device_index, samplerate=clip.samplerate, channels=clip.channels, dtype='float32')

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

            try:
                for start in range(0, clip.clip.shape[0], frames_per_chunk):
                    if stop_event.is_set() or rospy.is_shutdown():
                        stream.abort()
                        break

                    end = min(start + frames_per_chunk, clip.clip.shape[0])
                    stream.write(clip.clip[start:end])
            finally:
                with self.active_streams_lock:
                    self.active_streams.pop(device_index, None)

    def _device_worker(self, device_index, work_queue, stop_event):
        while True:
            clip, device = work_queue.get()
            try:
                stop_event.clear()
                self._play_clip_on_device(clip, device, stop_event)
            except Exception as exc:
                rospy.logerr(f'[sound_generator] Playback failed on device {device_index}: {exc}')
            finally:
                work_queue.task_done()

    def sound_callback(self, msg):
        rospy.loginfo('[sound_generator] Sound received: ' + msg.data)

        clip = self.clips.get(msg.data)
        if clip:
            for device in self.sound_devices:
                device_index = int(device['index'])
                worker_state = self.device_workers[device_index]
                work_queue = worker_state['queue']
                work_queue.put((clip, device_index))

            rospy.loginfo('[sound_generator] Queued sound: ' + msg.data)
        elif msg.data == 'Stop':
            for worker_state in self.device_workers.values():
                worker_state['stop_event'].set()
                self._clear_queue(worker_state['queue'])

            with self.active_streams_lock:
                for device_index, stream in self.active_streams.items():
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
