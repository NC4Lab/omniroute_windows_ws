# import rospy
# from std_msgs.msg import String
# import sounddevice as sd
# from scipy.io import wavfile
# import os
# import numpy as np



# class SoundGenerator:
#     def __init__(self):
#         rospy.loginfo('Sound Generator node started')

#         curDir = os.path.dirname(__file__)

#         # List sound devices
#         self.sound_devices = sd.query_devices()
#         self.sound_devices = [device for device in self.sound_devices if 'Laser Proj' in device['name'] and device['hostapi'] == 0]
#         rospy.loginfo('Sound devices found:')
#         for device in self.sound_devices:
#             rospy.loginfo(device['name'])

#         white_noise_file = 'audiocheck.net_whitenoise.wav'
#         self.white_noise_samplerate, self.white_noise = wavfile.read(os.path.join(curDir, white_noise_file))

#         five_KHz_file = 'audiocheck.net_sin_5000Hz_-3dBFS_3s.wav'
#         self.five_KHz_samplerate, self.five_KHz = wavfile.read(os.path.join(curDir, five_KHz_file))

#         #error_sound_file = 'mixkit-game-show-wrong-answer-buzz-950.wav'
#         error_sound_file = 'mixkit-game-show-wrong-answer-buzz-950 (Joined by Happy Scribe.wav'
#         self.error_sound_samplerate, self.error_sound = wavfile.read(os.path.join(curDir, error_sound_file))

#         self.sound_duration = 5  # seconds
#         self.error_sound_duration = 8  # seconds
#         self.white_noise = self.white_noise[:self.white_noise_samplerate*self.sound_duration]
#         self.five_KHz = self.five_KHz[:self.five_KHz_samplerate*self.sound_duration]
#         self.error_sound = self.error_sound[:int(self.error_sound_samplerate*self.error_sound_duration)]

#         # Create a subscriber for the sound topic
#         self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)


#         r = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             r.sleep()
    

#     # @brief Callback function for the sound topic
#     def sound_callback(self, msg):
#         rospy.loginfo('Sound received: ' + msg.data)
#         if msg.data == 'White_Noise':
#             [sd.play(self.white_noise, self.white_noise_samplerate, device=device['name']) for device in self.sound_devices] 
#         elif msg.data == '5KHz':
#             [sd.play(self.five_KHz, self.five_KHz_samplerate, device=device['name']) for device in self.sound_devices]
#         elif msg.data == 'Error':
#             [sd.play(self.error_sound, self.error_sound_samplerate, device=device['name']) for device in self.sound_devices]
#         else:
#             rospy.logwarn('Sound not recognized: ' + msg.data)
        

# # @brief Main code
# if __name__ == '__main__':
#     # Initialize the ROS node with name 'sound_generator'
#     rospy.init_node('sound_generator')
#     SoundGenerator()  # Create an instance of the class
#     rospy.spin()  # Keep the program running until it is explicitly shutdown

import os
import rospy
import sounddevice as sd
import threading
from scipy.io import wavfile
from std_msgs.msg import String

class SoundGenerator:
    def __init__(self):
        rospy.loginfo('Sound Generator node started')

        curDir = os.path.dirname(__file__)

        # List sound devices
        self.sound_devices = sd.query_devices()
        self.sound_devices = [device for device in self.sound_devices if 'Laser Proj' in device['name'] and device['hostapi'] == 0]
        rospy.loginfo('Sound devices found:')
        for device in self.sound_devices:
            rospy.loginfo(device['name'])

        white_noise_file = 'audiocheck.net_whitenoise.wav'
        self.white_noise_samplerate, self.white_noise = wavfile.read(os.path.join(curDir, white_noise_file))

        five_KHz_file = 'audiocheck.net_sin_5000Hz_-3dBFS_3s.wav'
        self.five_KHz_samplerate, self.five_KHz = wavfile.read(os.path.join(curDir, five_KHz_file))

        error_sound_file = 'mixkit-game-show-wrong-answer-buzz-950.wav'
        self.error_sound_samplerate, self.error_sound = wavfile.read(os.path.join(curDir, error_sound_file))

        self.sound_duration = 5  # seconds
        self.error_sound_duration = 8  # seconds
        self.white_noise = self.white_noise[:self.white_noise_samplerate*self.sound_duration]
        self.five_KHz = self.five_KHz[:self.five_KHz_samplerate*self.sound_duration]
        self.error_sound = self.error_sound[:int(self.error_sound_samplerate*self.error_sound_duration)]

        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)

        # Initialize loop control variables
        self.white_noise_looping = False
        self.five_KHz_looping = False

        self.white_noise_thread = None
        self.five_KHz_thread = None

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    def play_sound_loop(self, sound, samplerate, sound_name):
        while getattr(self, f"{sound_name}_looping"):
            for device in self.sound_devices:
                sd.play(sound, samplerate, device=device['name'])
                sd.wait()  # Wait until the sound finishes playing before looping again

    def stop_sound_loop(self, sound_name):
        setattr(self, f"{sound_name}_looping", False)
        if getattr(self, f"{sound_name}_thread") and getattr(self, f"{sound_name}_thread").is_alive():
            getattr(self, f"{sound_name}_thread").join()  # Ensure the thread stops properly

    def sound_callback(self, msg):
        rospy.loginfo('Sound received: ' + msg.data)
        
        if msg.data == 'White_Noise':
            [sd.play(self.white_noise, self.white_noise_samplerate, device=device['name']) for device in self.sound_devices]
        elif msg.data == '5KHz':
            [sd.play(self.five_KHz, self.five_KHz_samplerate, device=device['name']) for device in self.sound_devices]
        elif msg.data == 'Error':
            [sd.play(self.error_sound, self.error_sound_samplerate, device=device['name']) for device in self.sound_devices]
        elif msg.data == 'White_Noise_Training_Start':
            if not self.white_noise_looping:
                self.white_noise_looping = True
                self.white_noise_thread = threading.Thread(target=self.play_sound_loop, args=(self.white_noise, self.white_noise_samplerate, 'White_Noise'))
                self.white_noise_thread.start()
        elif msg.data == 'White_Noise_Training_Stop':
            self.stop_sound_loop('White_Noise')
        elif msg.data == '5KHz_Training_Start':
            if not self.five_KHz_looping:
                self.five_KHz_looping = True
                self.five_KHz_thread = threading.Thread(target=self.play_sound_loop, args=(self.five_KHz, self.five_KHz_samplerate, '5KHz'))
                self.five_KHz_thread.start()
        elif msg.data == '5khz_Training_Stop':
            self.stop_sound_loop('5KHz')
        else:
            rospy.logwarn('Sound not recognized: ' + msg.data)

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'sound_generator'
    rospy.init_node('sound_generator')
    SoundGenerator()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown