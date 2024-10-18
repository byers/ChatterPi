# -*- coding: utf-8 -*-
"""
Created on Sun May 17 22:19:49 2020
@author: Mike McGurrin
Updated to improve speed and run on Pi Zero 7/13/2020
"""
import wave
import time
import pyaudio
import atexit
import numpy as np
from bandpassFilter import BPFilter
import config as c

c.update()
try:
    #from gpiozero.pins.pigpio import PiGPIOFactory
    from gpiozero import Device, AngularServo
    #Device.pin_factory = PiGPIOFactory()
except:
    print("Unable to setup Pi GPIO pin factory: Audio!")

class AUDIO:
    def __init__(self, control = None, jaw_servo = None):
        print("Initializing PyAudio...")
        self.p = pyaudio.PyAudio()
        print("if you see ALSA error messages above, ignore them")
        print("End of PyAudio initialization")
        if jaw_servo == None:
            print("Using default GPIO based jaw servo!")
            self.jaw = AngularServo(c.JAW_PIN, min_angle=c.MIN_ANGLE,
                    max_angle=c.MAX_ANGLE, initial_angle=None,
                    min_pulse_width=c.SERVO_MIN/(1*10**6),
                    max_pulse_width=c.SERVO_MAX/(1*10**6))
        else:
            print("Using user assigned jaw servo!")
            self.jaw = jaw_servo
        self.bp = BPFilter()

        # Negate the output jaw angle for some cases
        self._negate_angle = False
        # flipping MIN_ANGLE and MAX_ANGLE in settings changes direction of servo movement BUT
        # must use unflipped values in calculating the amount of jaw movement
        if c.MIN_ANGLE > c.MAX_ANGLE:
            self.j_min = c.MIN_ANGLE
            self.j_max = c.MAX_ANGLE
        else:
            self.j_min = c.MAX_ANGLE
            self.j_max = c.MIN_ANGLE          

        self._control = control

    def set_control(self, contoller):
        self._control = controller

    def negate_angle(self, flag):
        self._negate_angle = bool(flag)
        
    def update_jaw(self, jaw_servo = None):
        if jaw_servo == None:
            self.jaw = AngularServo(c.JAW_PIN, min_angle=c.MIN_ANGLE,
                max_angle=c.MAX_ANGLE, initial_angle=None,
                min_pulse_width=c.SERVO_MIN/(1*10**6),
                max_pulse_width=c.SERVO_MAX/(1*10**6))
        else:
            print("Using user assigned jaw servo!")
            self.jaw = jaw_servo
            if c.MIN_ANGLE > c.MAX_ANGLE:
                self.j_min = c.MIN_ANGLE
                self.j_max = c.MAX_ANGLE
            else:
                self.j_min = c.MAX_ANGLE
                self.j_max = c.MIN_ANGLE
           
    def play_vocal_track(self, filename=None):
        # Used for both threshold (Scary Terry style) and multi-level (jawduino style)
        def get_avg(levels, channels):
            """Gets and returns the average volume for the frame (chunk).
            for stereo channels, only looks at the right channel (channel 1)"""
            # Apply bandpass filter if STYLE=2
            if c.STYLE == 2:
                levels = self.bp.filter_data(levels)
            levels = np.absolute(levels)
            if channels == 1:
                avg_volume = np.sum(levels)//len(levels)
            elif channels == 2:
                rightLevels = levels[1::2]
                avg_volume = np.sum(rightLevels)//len(rightLevels)
            return(avg_volume)
         
        def get_target(data, channels):
            levels = abs(np.frombuffer(data, dtype='<i2'))
            volume = get_avg(levels, channels)
            jawStep = (self.j_max - self.j_min) / 3
            if c.STYLE == 0:      # Scary Terry style single threshold
                if volume > c.THRESHOLD: 
                    jawTarget = self.j_max
                else: 
                    jawTarget = self.j_min
            elif c.STYLE == 1:     # Jawduino style multi-level or Wee Talker bandpss multi-level   
                if volume > c.LEVEL3:
                    jawTarget = self.j_max
                elif volume > c.LEVEL2:
                    jawTarget = self.j_min + 2 * jawStep
                elif volume > c.LEVEL1:
                    jawTarget = self.j_min + jawStep
                else:
                    jawTarget = self.j_min
            else:     # Jawduino style multi-level or Wee Talker bandpss multi-level   
                if volume > c.FIlTERED_LEVEL3:
                    jawTarget = self.j_max
                elif volume > c.FIlTERED_LEVEL2:
                    jawTarget = self.j_min + 2 * jawStep
                elif volume > c.FIlTERED_LEVEL1:
                    jawTarget = self.j_min + jawStep
                else:
                    jawTarget = self.j_min   
            if self._negate_angle:
                return -jawTarget
            return jawTarget
        
        def overwrite(data, channels):
            """ overwrites left channel onto right channel for playback"""
            if channels != 2:
                raise ValueError("channels must equal 2")
            levels = np.frombuffer(data, dtype='<i2')
            new_levels = np.copy(levels)
            new_levels[1::2] = levels[::2]
            # levels[1::2] = levels[::2]
            # data = new_levels.tolist()
            # return data
            return new_levels
        
        def filesCallback(in_data, frame_count, time_info, status):
            nonlocal latest_time
            data = wf.readframes(frame_count)
            channels = wf.getnchannels()
            # Only proces jaw movements 50x per second, to avoid buffer overruns
            now = time.monotonic()
            if now - latest_time > 0.02:
                latest_time = now   
                jawTarget = get_target(data, channels)
                print("Jaw Angle: %f" % (jawTarget))
                self.jaw.angle = jawTarget
            # If only want left channel of input, duplicate left channel on right
            if (channels == 2) and (c.OUTPUT_CHANNELS == 'LEFT'):
                data = overwrite(data, channels)
            return (data, pyaudio.paContinue)  
           
        def micCallback(in_data, frame_count, time_info, status):
            nonlocal latest_time
            channels = 1 # Microphone input is always monaural
            # Only proces jaw movements 50x per second, to avoid buffer overruns
            now = time.monotonic()
            if now - latest_time > 0.02:
                latest_time = now   
                jawTarget = get_target(in_data, channels)
                self.jaw.angle = jawTarget            
            return (in_data, pyaudio.paContinue)     
               
        def normalEnd():
            self.stream.stop_stream()
            self.stream.close()
            if (c.SOURCE == 'FILES'):
                wf.close()
            self.jaw.angle = None  
            
        def cleanup():
            normalEnd()
            self.p.terminate()
            self.jaw.close()
            
        try:
            atexit.register(cleanup)                      
            #Playing from wave file
            if c.SOURCE == 'FILES':
                wf = wave.open(filename, 'rb')
                file_sw = wf.getsampwidth()  
                # New code to support only process jaw movements 50x per second
                start_time = time.monotonic() 
                latest_time = start_time                                 
                self.stream = self.p.open(format=self.p.get_format_from_width(file_sw),
                            channels=wf.getnchannels(),
                            rate=wf.getframerate(),
                            frames_per_buffer = c.BUFFER_SIZE,
                            output=True,
                            stream_callback=filesCallback)  
                while self.stream.is_active():                
                    time.sleep(0.1)

            # Playing from microphone
            elif c.SOURCE == 'MICROPHONE':
                # New code to support only process jaw movements 50x per second
                start_time = time.monotonic() 
                latest_time = start_time                  
                self.stream = self.p.open(format=pyaudio.paInt16, channels=1,
                            rate=48000, frames_per_buffer=c.BUFFER_SIZE,
                            input=True, output=True,
                            stream_callback=micCallback)  
                if c.PROP_TRIGGER != 'START':
                    time.sleep(c.MIC_TIME)
                    self.stream.close()   
                else:
                    while self.stream.is_active():
                        time.sleep(1.)                                           
            normalEnd() 
        except (KeyboardInterrupt, SystemExit):
            cleanup()               
        
    def play_ambient_track(self, filename=None):    
        def ambientCallback(in_data, frame_count, time_info, status):
            data = wf.readframes(frame_count)
            return (data, pyaudio.paContinue)  
               
        def normalEnd():
            self.stream.stop_stream()
            self.stream.close()
            wf.close()
            
        def cleanup():
            normalEnd()
            self.p.terminate()
            self.jaw.close()
            
        try:
            atexit.register(cleanup)                      
            #Playing from ambient file
            wf = wave.open(filename, 'rb')
            file_sw = wf.getsampwidth()                                    
            self.stream = self.p.open(format=self.p.get_format_from_width(file_sw),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        frames_per_buffer = c.BUFFER_SIZE,
                        output=True,
                        stream_callback=ambientCallback)  

            while self.stream.is_active():           
                time.sleep(0.1)
                # interrupt and play vocal track, moving jaw
                if self._control:
                    if c.PROP_TRIGGER == 'PIR':
                        if self._control.pir.is_pressed:
                            self._control.ambient_interrupt = True
                            break
                    if c.PROP_TRIGGER == 'TIMER':
                        if time.time() > self._control.trigger_time:
                            self._control.ambient_interrupt = True
                            break
            normalEnd()
                    
        except (KeyboardInterrupt, SystemExit):
            cleanup()

   
        

            
