#
# Servo and Angular Servo wrappers for Pololu servo controller: based on a library for controlling the Raspberry Pi's GPIO pins
#

from maestro import Controller

# Danger Will Robinson - These callbacks are specific to your servo controller and servos

# Function for callbacks before and after ambient sound
def pre_ambient(controller):
   # Run script to do motion (use loop) during ambient sound
   controller.runScriptSub(0)

def post_ambient(controller):
   # Stop the currently running script
   controller.stopScript()
   # Run script to set prop into neutral position for talking
   #controller.runScriptSub(1)

class ServoAdapter:
    def __init__(self, controller, pin):
        self._controller = controller
        self._pin = pin

    def set_target(self, fraction, pulse_width):
        if pulse_width is None:
            print("Target None: %d" % (self._pin))
            self._controller.setTarget(self._pin, 0)
        else:
            # Covert to quarter microseconds for Pololu
            pololu_pulse_width = pulse_width * 4 * 1000000
            print("Pulse Width (quarter Microseconds): %d->%d" % (pololu_pulse_width, self._pin))
            self._controller.setTarget(self._pin, int(pololu_pulse_width))

    def set_pulse_width_range(self, min_pulse_width, max_pulse_width):
        print("Set Pulse Width Range: [%d, %d]" % (min_pulse_width, max_pulse_width))

    def close(self):
        print("Pololu servo close.")
        self.set_target(0, None)

class Factory:
    def __init__(self, port = "/dev/ttyACM0"):
        self._controller = Controller(port)

    def create(self, index):
        return ServoAdapter(self._controller, index)

class EyesPinAdapter:
    def __init__(self, link):
        self._link = link

    def off(self):
        self._link._controller.setTarget(self._link._pin, 4000)

    def on(self):
        self._link._controller.setTarget(self._link._pin, 8000)

    def close(self):
        print("Close eyes!")
