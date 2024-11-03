#
# Servo and Angular Servo wrappers for Adafruit ServoKit: based on a library for controlling the Raspberry Pi's GPIO pins
#
from adafruit_servokit import ServoKit
import config as c

# Function for callbacks before and after ambient sound
def pre_ambient(controller):
   print("Pre Ambient Function")
   # Run script to do motion (use loop) during ambient sound

def post_ambient(controller):
   print("Post Ambient Function")
   # Stop the currently running script
   # Run script to set prop into neutral position for talking

class ServoAdapter:
    def __init__(self, servo, pin):
        self._servo = servo
        self._pin = pin

    def set_target(self, fraction, pulse_width):
        if pulse_width is None:
            print("Target None: %d" % (self._pin))
            # NOTE: TBD
            #self._controller.setTarget(self._servo.pin, 0)
        else:
            self._servo.fraction = fraction

    def set_pulse_width_range(self, min_pulse_width, max_pulse_width):
        print("Set Pulse Width Range: [%d, %d]" % (min_pulse_width, max_pulse_width))
        self._servo.set_pulse_width_range(min_pulse_width, max_pulse_width)

class Factory:
    def __init__(self, nchannels=16):
        self._controller = ServoKit(channels=nchannels)

    def create(self, pin):
        return ServoAdapter(self._controller.servo[pin], pin)
