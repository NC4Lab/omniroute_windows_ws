import rospy
from std_msgs.msg import String
import sounddevice as sd
from scipy.io import wavfile
import os


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
        #self.error_sound_duration = 0.1  # seconds
        self.white_noise = self.white_noise[:self.white_noise_samplerate*self.sound_duration]
        self.five_KHz = self.five_KHz[:self.five_KHz_samplerate*self.sound_duration]
        #self.error_sound = self.error_sound[:self.error_sound_samplerate*self.error_sound_duration]

        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)


        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
    

    # @brief Callback function for the sound topic
    def sound_callback(self, msg):
        rospy.loginfo('Sound received: ' + msg.data)
        if msg.data == 'White_Noise':
            [sd.play(self.white_noise, self.white_noise_samplerate, device=device['name']) for device in self.sound_devices] 
        elif msg.data == '5KHz':
            [sd.play(self.five_KHz, self.five_KHz_samplerate, device=device['name']) for device in self.sound_devices]
        elif msg.data == 'Error':
            [sd.play(self.error_sound, self.error_sound_samplerate, device=device['name']) for device in self.sound_devices]
        else:
            rospy.logwarn('Sound not recognized: ' + msg.data)
        

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'sound_generator'
    rospy.init_node('sound_generator')
    SoundGenerator()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown