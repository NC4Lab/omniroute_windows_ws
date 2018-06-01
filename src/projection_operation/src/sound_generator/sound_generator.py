#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sounddevice as sd
from scipy.io import wavfile
import os
import numpy as np

class SoundClip:
    def __init__(self, file_name, duration, amplitude=1.0, preload = False):
        self.file_path = os.path.join(os.path.dirname(__file__), 'sound_clips', file_name)
        self.duration = duration
        self.amplitude = amplitude
        self.samplerate = 0
        self.clip = []
        self.loaded = False

        if preload:
            self.load_clip()

    def load_clip(self):
        rospy.loginfo(f'Loading sound clip: {self.file_path}')
        self.samplerate, self.clip = wavfile.read(self.file_path)
        rospy.loginfo('Loaded sound clip: ' + self.file_path)
        rospy.loginfo('Sample rate: ' + str(self.samplerate))

        self.clip = np.array(self.clip, dtype=np.float32)

        # Trim the clip to the specified duration
        if len(self.clip) > np.floor(self.samplerate*self.duration):
            self.clip = self.clip[:int(np.floor(self.samplerate*self.duration))]
        else:
            self.clip = np.tile(self.clip, int(np.ceil(self.duration*1.0/len(self.clip))))
            self.clip = self.clip[:int(np.floor(self.samplerate*self.duration))]

        # Normalize the clip
        self.clip = self.amplitude*self.clip/np.max(np.abs(self.clip))
        self.loaded = True

    def play(self, device):
        rospy.loginfo('Playing sound clip: ' + self.file_path + ' on device: ' + device)
        if not self.loaded:
            rospy.logwarn(f'Sound {self.file_path} not preloaded, loading now...')
            self.load_clip()

        rospy.loginfo(f'Playing {self.file_path}')
        if device:
            sd.play(self.clip, self.samplerate, device=device)
        else:
            sd.play(self.clip, self.samplerate)

class SoundGenerator:
    def __init__(self):
        rospy.loginfo('SOUND GENERATOR NODE STARTED')

        # List sound devices
        self.sound_devices = sd.query_devices()
        self.sound_devices = [device for device in self.sound_devices if 'Laser Proj' in device['name'] and device['hostapi'] == 0]
        rospy.loginfo('Sound devices found:')
        for device in self.sound_devices:
            rospy.loginfo(device['name'])

        # Clips dictionary
        self.clips = {'White_Noise': SoundClip('whitenoise.wav', 5, 0.5, preload=True),
                        '5KHz': SoundClip('5khz.wav', 5.0, preload=True),
                        'Error': SoundClip('wrong-answer-buzz.wav', 8.0, preload=True),
                        '1KHz': SoundClip('1kHz.wav', 0.5, 0.5, preload=True),
                        '8KHz': SoundClip('8kHz.wav', 0.5, 0.5, preload=True),
                        'White_Noise_Training': SoundClip('white-noise-2min.wav', 120.0, preload=True),
                        '5KHz_Training': SoundClip('5khz-2min.wav', 120.0, preload=True),
                        'Starting_Sound': SoundClip('1kHz.wav', 0.5, 0.5, preload=True),
                        '1KHz_120s': SoundClip('tone_1000Hz_120.0s.wav', 120.0, 0.5, preload=True),
                        '8KHz_120s': SoundClip('tone_8000Hz_120.0s.wav', 120.0, 0.5, preload=True)
                        }
        
        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)
        
        #r = rospy.Rate(10)
        #while not rospy.is_shutdown():
        #    r.sleep()

        rospy.loginfo('SoundGenerator initialized successfully.')

    def sound_callback(self, msg):
        rospy.loginfo('Sound received: ' + msg.data)

        clip = self.clips.get(msg.data)
        if clip:
            [clip.play(device['name']) for device in self.sound_devices]
        elif msg.data == 'Stop':
            sd.stop()
        else:
            rospy.logwarn('Sound command not recognized: ' + msg.data)

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'sound_generator'
    rospy.init_node('sound_generator')
    SoundGenerator()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown
