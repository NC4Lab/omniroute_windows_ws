import rospy
from std_msgs.msg import String
import sounddevice as sd
from scipy.io import wavfile


class SoundGenerator:
    def __init__(self):
        rospy.loginfo('Sound Generator node started')

        # List sound devices
        self.sound_devices = sd.query_devices()
        self.sound_devices = [device for device in self.sound_devices if 'Laser Proj' in device['name'] and device['hostapi'] == 0]
        rospy.loginfo('Sound devices found:')
        for device in self.sound_devices:
            rospy.loginfo(device['name'])

        white_noise_file = 'audiocheck.net_whitenoise.wav'
        self.white_noise_samplerate, self.white_noise = wavfile.read(white_noise_file)

        five_KHz_file = 'audiocheck.net_sin_5000Hz_-3dBFS_3s.wav'
        self.five_KHz_samplerate, self.five_KHz = wavfile.read(five_KHz_file)

        self.sound_duration = 1  # seconds
        self.white_noise = self.white_noise[:self.white_noise_samplerate*self.sound_duration]
        self.five_KHz = self.five_KHz[:self.five_KHz_samplerate*self.sound_duration]

        # Create a subscriber for the sound topic
        self.sound_sub = rospy.Subscriber('/sound_cmd', String, self.sound_callback)

        # r = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.loop()
        #     r.sleep()

    # @brief Callback function for the sound topic
    def sound_callback(self, msg):
        rospy.loginfo('Sound received: ' + msg.data)

        if msg.data == 'White_Noise':
            for device in self.sound_devices:
                rospy.loginfo('Playing white noise on ' + device['name'])
                sd.play(self.white_noise, self.white_noise_samplerate, device=device['name'])
                sd.wait()
        elif msg.data == '5KHz':
            for device in self.sound_devices:
                rospy.loginfo('Playing 5 KHz on ' + device['name'])
                sd.play(self.five_KHz, self.five_KHz_samplerate, device=device['name'])
                sd.wait()

    # @brief Loop function to keep the node alive
    def loop(self):
        pass
        

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'gantry_operation'
    rospy.init_node('sound_generator')
    SoundGenerator()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown