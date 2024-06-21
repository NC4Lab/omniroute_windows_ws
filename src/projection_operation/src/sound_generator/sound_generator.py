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

import rospy
from std_msgs.msg import String
import sounddevice as sd
from scipy.io import wavfile
import os
import numpy as np
import threading

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
        
        self.looping_sound = False
        self.sound_thread = None
        
        white_noise_file_five = 'audiocheck.net_whitenoise.wav'
        self.white_noise_samplerate_five, self.white_noise_five = wavfile.read(os.path.join(curDir, white_noise_file_five))
        
        five_KHz_file_six = 'audiocheck.net_sin_5000Hz_-3dBFS_3s.wav'
        self.five_KHz_samplerate_six, self.five_KHz_six = wavfile.read(os.path.join(curDir, five_KHz_file_six))
        
        white_noise_file_twenty = 'white-noise-20(Joined by Happy Scribe).wav'
        self.white_noise_samplerate_twenty, self.white_noise_twenty = wavfile.read(os.path.join(curDir, white_noise_file_twenty))
        
        five_KHz_file_twenty = '5khz-20 (Joined by Happy Scribe).wav'
        self.five_KHz_samplerate_twenty, self.five_KHz_twenty = wavfile.read(os.path.join(curDir, five_KHz_file_twenty))

        error_sound_file = 'mixkit-game-show-wrong-answer-buzz-950.wav'
        self.error_sound_samplerate, self.error_sound = wavfile.read(os.path.join(curDir, error_sound_file))

        self.sound_duration_one = 5  # seconds
        self.sound_duration_two = 20  # seconds
        self.error_sound_duration = 8  # seconds
        self.white_noise_five = self.white_noise_five[:self.white_noise_samplerate_five*self.sound_duration_one]
        self.five_KHz_six = self.five_KHz_six[:self.five_KHz_samplerate_six*self.sound_duration_one]
        self.white_noise_twenty = self.white_noise_twenty[:self.white_noise_samplerate_twenty*(-1)]
        self.five_KHz_twenty = self.five_KHz_twenty[:self.five_KHz_samplerate_twenty*(-1)]
        self.error_sound = self.error_sound[:int(self.error_sound_samplerate*self.error_sound_duration)]

        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)
        
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    def sound_callback(self, msg):
        rospy.loginfo('Sound received: ' + msg.data)
        
        if msg.data == 'White_Noise':
            [sd.play(self.white_noise_five, self.white_noise_samplerate_five, device=device['name']) for device in self.sound_devices]
        elif msg.data == '5KHz':
            [sd.play(self.five_KHz_six, self.five_KHz_samplerate_six, device=device['name']) for device in self.sound_devices]
        elif msg.data == 'Error':
            [sd.play(self.error_sound, self.error_sound_samplerate, device=device['name']) for device in self.sound_devices]
        elif msg.data == 'White_Noise_Training_Start':
            [sd.play(self.white_noise_twenty, self.white_noise_samplerate_twenty, device=device['name']) for device in self.sound_devices]
            # if not self.looping_sound:
            #     self.looping_sound = True
            #     self.sound_thread = threading.Thread(target=self.play_sound)
            #     self.sound_thread.start()
        # elif msg.data == 'White_Noise_Training_Stop':
            # self.looping_sound = False
            # for device in self.sound_devices:
            #     sd.stop(device=device['name'])
            # if self.sound_thread:
            #     self.sound_thread.join()
        elif msg.data == '5KHz_Training_Start':
            # while self.looping_sound:
            [sd.play(self.five_KHz_twenty, self.five_KHz_samplerate_twenty, device=device['name']) for device in self.sound_devices]
        # elif msg.data == '5KHz_Training_Stop':
        #     self.looping_sound = False
        else:
            rospy.logwarn('Sound not recognized: ' + msg.data)

    def play_sound_on_device(self, device):
        while self.looping_sound:
            sd.play(self.white_noise_twenty, self.white_noise_samplerate_twenty, device=device['name'])
            sd.wait()  # Ensure the sound finishes playing
            # Check if looping_sound is still True before starting next loop
            if not self.looping_sound:
                break  # Exit the loop if stopping signal received

    def play_sound(self):
        threads = []
        for device in self.sound_devices:
            thread = threading.Thread(target=self.play_sound_on_device, args=(device,))
            threads.append(thread)
            thread.start()

        # Wait for all threads to complete (only when looping_sound becomes False)
        for thread in threads:
            thread.join()
       

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'sound_generator'
    rospy.init_node('sound_generator')
    SoundGenerator()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown