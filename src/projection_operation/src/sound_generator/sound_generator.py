import rospy
from std_msgs.msg import String
import sounddevice as sd
from scipy.io import wavfile
import os
import numpy as np
import threading

class SoundGenerator:
    def __init__(self):
        rospy.loginfo('SOUND GENERATOR NODE STARTED')

        curDir = os.path.dirname(__file__)

        # List sound devices
        self.sound_devices = sd.query_devices()
        self.sound_devices = [device for device in self.sound_devices if 'Laser Proj' in device['name'] and device['hostapi'] == 0]
        rospy.loginfo('Sound devices found:')
        for device in self.sound_devices:
            rospy.loginfo(device['name'])
        
        self.looping_sound = False
        self.sound_thread = None
        
        white_noise_file_short = 'whitenoise.wav'
        self.white_noise_samplerate_short, self.white_noise_short = wavfile.read(os.path.join(curDir, white_noise_file_short))
        
        five_KHz_file_short = '5khz.wav'
        self.five_KHz_samplerate_short, self.five_KHz_short = wavfile.read(os.path.join(curDir, five_KHz_file_short))
        
        white_noise_file_long = 'white-noise-2min.wav'
        self.white_noise_samplerate_long, self.white_noise_long = wavfile.read(os.path.join(curDir, white_noise_file_long))
        
        five_KHz_file_long = '5khz-2min.wav'
        self.five_KHz_samplerate_long, self.five_KHz_long = wavfile.read(os.path.join(curDir, five_KHz_file_long))

        error_sound_file = 'wrong-answer-buzz.wav'
        self.error_sound_samplerate, self.error_sound = wavfile.read(os.path.join(curDir, error_sound_file))

        self.sound_duration_one = 5  # seconds
        self.sound_duration_two = 20  # seconds
        self.error_sound_duration = 8  # seconds
        self.white_noise_short = self.white_noise_short[:self.white_noise_samplerate_short*self.sound_duration_one]
        self.five_KHz_short = self.five_KHz_short[:self.five_KHz_samplerate_short*self.sound_duration_one]
        self.white_noise_long = self.white_noise_long[:self.white_noise_samplerate_long*(-1)]
        self.five_KHz_long = self.five_KHz_long[:self.five_KHz_samplerate_long*(-1)]
        self.error_sound = self.error_sound[:int(self.error_sound_samplerate*self.error_sound_duration)]

        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)
        
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    def sound_callback(self, msg):
        rospy.loginfo('Sound received: ' + msg.data)
        
        if msg.data == 'White_Noise':
            [sd.play(self.white_noise_short, self.white_noise_samplerate_short, device=device['name']) for device in self.sound_devices]
        elif msg.data == '5KHz':
            [sd.play(self.five_KHz_short, self.five_KHz_samplerate_short, device=device['name']) for device in self.sound_devices]
        elif msg.data == 'Error':
            [sd.play(self.error_sound, self.error_sound_samplerate, device=device['name']) for device in self.sound_devices]
        elif msg.data == 'White_Noise_Training_Start':
            [sd.play(self.white_noise_long, self.white_noise_samplerate_long, device=device['name']) for device in self.sound_devices]
            # if not self.looping_sound:
            #     self.looping_sound = True
            #     self.sound_thread = threading.Thread(target=self.play_sound, args=('White_Noise_Training_Start',) )
            #     self.sound_thread.start()
        elif msg.data == 'White_Noise_Training_Stop':
            # self.looping_sound = False
            sd.stop()
            # if self.sound_thread:
            #     self.sound_thread.join()
        elif msg.data == '5KHz_Training_Start':
            # if not self.looping_sound:
            #     self.looping_sound = True
            #     self.sound_thread = threading.Thread(target=self.play_sound, args=('5KHz_Training_Start',) )
            #     self.sound_thread.start()
            [sd.play(self.five_KHz_long, self.five_KHz_samplerate_long, device=device['name']) for device in self.sound_devices]
        elif msg.data == '5KHz_Training_Stop':
            # self.looping_sound = False
            sd.stop()
            # if self.sound_thread:
            #     self.sound_thread.join()
        else:
            rospy.logwarn('Sound not recognized: ' + msg.data)

    def play_sound_on_device(self, device, noise_type):
        while self.looping_sound:
            if noise_type == 'White_Noise_Training_Start':
                sd.play(self.white_noise_five, self.white_noise_samplerate_five, device=device['name'])
                #sd.wait()  # Ensure the sound finishes playing
            elif noise_type == '5KHz_Training_Start':
                sd.play(self.five_KHz_six, self.five_KHz_samplerate_six, device=device['name'])
                #sd.wait()  # Ensure the sound finishes playing
            # Check if looping_sound is still True before starting next loop
            if not self.looping_sound:
                break # Exit the loop if stopping signal received

    def play_sound(self, noise_type):
        threads = []
        for device in self.sound_devices:
            thread = threading.Thread(target=self.play_sound_on_device, args=(device, noise_type,))
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




# import rospy
# from std_msgs.msg import String
# import sounddevice as sd
# from scipy.io import wavfile
# import os
# import threading

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
        
#         # Load audio files
#         white_noise_file_short = 'whitenoise.wav'
#         self.white_noise_samplerate_short, self.white_noise_short = wavfile.read(os.path.join(curDir, white_noise_file_short))
        
#         five_KHz_file_short = '5khz.wav'
#         self.five_KHz_samplerate_short, self.five_KHz_short = wavfile.read(os.path.join(curDir, five_KHz_file_short))
        
#         white_noise_file_long = 'white-noise-2min.wav'
#         self.white_noise_samplerate_long, self.white_noise_long = wavfile.read(os.path.join(curDir, white_noise_file_long))
        
#         five_KHz_file_long = '5khz-2min.wav'
#         self.five_KHz_samplerate_long, self.five_KHz_long = wavfile.read(os.path.join(curDir, five_KHz_file_long))

#         error_sound_file = 'wrong-answer-buzz.wav'
#         self.error_sound_samplerate, self.error_sound = wavfile.read(os.path.join(curDir, error_sound_file))

#         self.sound_duration_one = 5  # seconds
#         self.sound_duration_two = 20  # seconds
#         self.error_sound_duration = 8  # seconds
#         self.white_noise_short = self.white_noise_short[:self.white_noise_samplerate_short * self.sound_duration_one]
#         self.five_KHz_short = self.five_KHz_short[:self.five_KHz_samplerate_short * self.sound_duration_one]

#         # Create a subscriber for the sound topic
#         self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)
        
#         r = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             r.sleep()

#     def sound_callback(self, msg):
#         rospy.loginfo('Sound received: ' + msg.data)
        
#         if msg.data == 'White_Noise':
#             self.play_single(self.white_noise_short, self.white_noise_samplerate_short)
#         elif msg.data == '5KHz':
#             self.play_single(self.five_KHz_short, self.five_KHz_samplerate_short)
#         elif msg.data == 'Error':
#             self.play_single(self.error_sound, self.error_sound_samplerate)
#         elif msg.data == 'White_Noise_Training_Start':
#             self.start_streaming(self.white_noise_short, self.white_noise_samplerate_short)
#         elif msg.data == '5KHz_Training_Start':
#             self.start_streaming(self.five_KHz_short, self.five_KHz_samplerate_short)
#         elif msg.data == 'White_Noise_Training_Stop' or msg.data == '5KHz_Training_Stop':
#             self.stop_streaming()
#         else:
#             rospy.logwarn('Sound not recognized: ' + msg.data)

#     def play_single(self, sound_data, sample_rate):
#         for device in self.sound_devices:
#             sd.play(sound_data, sample_rate, device=device['name']) 

#     def start_streaming(self, sound_data, sample_rate):
#         self.looping_sound = True
#         self.streams = []

#         def make_callback(sound_data, sample_rate):
#             position = 0
#             data_len = len(sound_data)

#             def callback(outdata, frames, time, status):
#                 nonlocal position
#                 if status:
#                     print(status)

#                 current_position = position % data_len
#                 remaining_data = data_len - current_position

#                 if frames <= remaining_data:
#                     outdata[:frames] = sound_data[current_position:current_position + frames]
#                     position += frames
#                 else:
#                     outdata[:remaining_data] = sound_data[current_position:]
#                     outdata[remaining_data:frames] = sound_data[:frames - remaining_data]
#                     position = frames - remaining_data

#             return callback

#         for device in self.sound_devices:
#             callback = make_callback(sound_data, sample_rate)
#             stream = sd.OutputStream(samplerate=sample_rate, channels=sound_data.shape[1], callback=callback, device=device['name'])
#             stream.start()
#             self.streams.append(stream)

#         # Keep the streams active until stopped
#         self.stream_thread = threading.Thread(target=self.keep_streaming)
#         self.stream_thread.start()

#     def keep_streaming(self):
#         while self.looping_sound:
#             sd.sleep(1000)

#     def stop_streaming(self):
#         self.looping_sound = False
#         if hasattr(self, 'streams'):
#             for stream in self.streams:
#                 stream.stop()
#                 stream.close()
#             self.streams = []
#         if hasattr(self, 'stream_thread') and self.stream_thread:
#             self.stream_thread.join()

