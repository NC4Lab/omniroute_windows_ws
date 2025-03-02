# Make clips at the specified frequencies and durations
#
# Manu Madhav
# 2025

import numpy as np
from scipy.io import wavfile
import os
import argparse


def make_tone(frequency, duration, amplitude=1.0, sample_rate=48000):
    """
    Make a tone at the specified frequency and duration
    """
    # Generate the time vector
    t = np.linspace(0, duration, int(sample_rate*duration), False)
    
    # Generate the tone
    tone = amplitude*np.sin(2*np.pi*frequency*t)
    tone = np.array(tone, dtype = np.float32)

    try:
        wavfile.write(os.path.join(os.path.dirname(__file__), 'sound_clips', 'tone_' + str(frequency) + 'Hz_' + str(duration) + 's.wav'), sample_rate, tone.astype(np.float32))
    except:
        print('Error writing the tone to a file')


if __name__ == '__main__':
    """
    Make a tone at the specified frequency and duration

    Usage: python make_tone_clips.py frequency duration [--amplitude amplitude] [--sample_rate sample_rate]
    """
    parser = argparse.ArgumentParser(description='Make a tone at the specified frequency and duration')
    parser.add_argument('frequency', type=int, help='Frequency of the tone in Hz')
    parser.add_argument('duration', type=float, help='Duration of the tone in seconds')
    parser.add_argument('--amplitude', type=float, default=1.0, help='Amplitude of the tone')
    parser.add_argument('--sample_rate', type=int, default=48000, help='Sample rate of the tone')
    args = parser.parse_args()

    make_tone(args.frequency, args.duration, args.amplitude, args.sample_rate)
    

    