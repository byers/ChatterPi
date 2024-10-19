#
# Servo and Angular Servo wrappers for Pololu servo controller: based on a library for controlling the Raspberry Pi's GPIO pins
#
# Copyright (c) 2015-2023 Dave Jones <dave@waveform.org.uk>
# Copyright (c) 2022 gnicki2000 <89583687+gnicki2000@users.noreply.github.com>
# Copyright (c) 2020 Fangchen Li <fangchen.li@outlook.com>
# Copyright (c) 2015-2020 Ben Nuttall <ben@bennuttall.com>
# Copyright (c) 2019 tuftii <3215045+tuftii@users.noreply.github.com>
# Copyright (c) 2019 tuftii <pi@raspberrypi>
# Copyright (c) 2019 Yisrael Dov Lebow 🐻 <lebow@lebowtech.com>
# Copyright (c) 2019 Kosovan Sofiia <sofiia.kosovan@gmail.com>
# Copyright (c) 2016-2019 Andrew Scheller <github@loowis.durge.org>
# Copyright (c) 2016 Ian Harcombe <ian.harcombe@gmail.com>
#
# SPDX-License-Identifier: BSD-3-Clause

from maestro import Controller
import config as c
from dataclasses import dataclass

# Danger Will Robinson - These callbacks are specific to your servo controller and servos

# Function for callbacks before and after ambient sound
def pre_ambient(controller):
   controller.runScriptSub(0)

def post_ambient(controller):
   controller.stopScript()
   # Set the head swivel servo to front
   controller.setTarget(2, 1500*4)

@dataclass
class ServoLink:
    controller: Controller
    pin: int

class Factory:
    def __init__(self, port = "/dev/ttyACM0"):
        self._controller = Controller(port)

    def create(self, index):
        return ServoLink(self._controller, index)

class EyesPinAdapter:
    def __init__(self, link):
        self._link = link

    def off(self):
        self._link.controller.setTarget(self._link.pin, 4000) 

    def on(self):
        self._link.controller.setTarget(self._link.pin, 8000) 

    def close(self):
        print("Close eyes!")

class Servo:
    """
    Use Pololu servo controller board.

    The following code will make the servo move between its minimum, maximum,
    and mid-point positions with a pause between each::

        from angularServoKit import Servo
        from time import sleep

        servo = Servo(0)

        while True:
            servo.min()
            sleep(1)
            servo.mid()
            sleep(1)
            servo.max()
            sleep(1)

    You can also use the :attr:`value` property to move the servo to a
    particular position, on a scale from -1 (min) to 1 (max) where 0 is the
    mid-point::

        from angularServoKit import Servo

        servo = Servo(0)

        servo.value = 0.5

    .. note::

        To reduce servo jitter, use the pigpio pin driver rather than the default
        RPi.GPIO driver (pigpio uses DMA sampling for much more precise edge
        timing). See :ref:`changing-pin-factory` for further information.

    :type pin: int or str
    :param pin:
        Index of the servo: 0-7 or 0-15, depending on servo driver board

    :param float initial_value:
        If ``0`` (the default), the device's mid-point will be set initially.
        Other values between -1 and +1 can be specified as an initial position.
        :data:`None` means to start the servo un-controlled (see
        :attr:`value`).

    :param float min_pulse_width:
        The pulse width corresponding to the servo's minimum position. This
        defaults to 1ms.

    :param float max_pulse_width:
        The pulse width corresponding to the servo's maximum position. This
        defaults to 2ms.

    :param float frame_width:
        The length of time between servo control pulses measured in seconds.
        This defaults to 20ms which is a common value for servos.

    :type pin_factory: Factory or None
    :param pin_factory:
        See :doc:`api_pins` for more information (this is an advanced feature
        which most users can ignore).
    """
    def __init__(self, pin=None, *, initial_value=0.0, min_pulse_width=1/1000,
                 max_pulse_width=2/1000, frame_width=20/1000,
                 pin_factory=None):
        if min_pulse_width >= max_pulse_width:
            raise ValueError('min_pulse_width must be less than max_pulse_width')
        if max_pulse_width >= frame_width:
            raise ValueError('max_pulse_width must be less than frame_width')
        self._frame_width = frame_width
        self._min_dc = min_pulse_width / frame_width
        self._dc_range = (max_pulse_width - min_pulse_width) / frame_width

        # Default range of AngularServo
        self._min_value = -1
        self._value_range = 2

        self._pin = int(pin)
        self._servo = pin_factory.create(self._pin) 
        # This is done with Pololu control panel
        #self._servo.set_pulse_width_range(min_pulse_width*1000000, max_pulse_width*1000000)
        self._mqtt = None

        # Store -1 to 1 value
        self._value = initial_value

    @property
    def frame_width(self):
        """
        The time between control pulses, measured in seconds.
        """
        return self._frame_width

    @property
    def min_pulse_width(self):
        """
        The control pulse width corresponding to the servo's minimum position,
        measured in seconds.
        """
        return self._min_dc * self.frame_width

    @property
    def max_pulse_width(self):
        """
        The control pulse width corresponding to the servo's maximum position,
        measured in seconds.
        """
        return (self._dc_range * self.frame_width) + self.min_pulse_width

    @property
    def pulse_width(self):
        """
        Returns the current pulse width controlling the servo.
        """
        if self.pwm_device.pin.frequency is None:
            return None
        else:
            # TODO: how to get duty cycle from ServoKit
            return self.pwm_device.pin.state * self.frame_width

    @pulse_width.setter
    def pulse_width(self, value):
        print("Set pulse_width")
        # TODO: how to set duty cycle from ServoKit
        self.pwm_device.pin.state = value / self.frame_width

    def min(self):
        """
        Set the servo to its minimum position.
        """
        self.value = -1

    def mid(self):
        """
        Set the servo to its mid-point position.
        """
        self.value = 0

    def max(self):
        """
        Set the servo to its maximum position.
        """
        self.value = 1

    def detach(self):
        """
        Temporarily disable control of the servo. This is equivalent to
        setting :attr:`value` to :data:`None`.
        """
        self.value = None

    def _get_value(self):
        if self._value is None:
            return None
        else:
            return self._value

    @property
    def value(self):
        """
        Represents the position of the servo as a value between -1 (the minimum
        position) and +1 (the maximum position). This can also be the special
        value :data:`None` indicating that the servo is currently
        "uncontrolled", i.e. that no control signal is being sent. Typically
        this means the servo's position remains unchanged, but that it can be
        moved by hand.
        """
        result = self._get_value()
        if result is None:
            return result
        else:
            # NOTE: This round() only exists to ensure we don't confuse people
            # by returning 2.220446049250313e-16 as the default initial value
            # instead of 0. The reason _get_value and _set_value are split
            # out is for descendents that require the un-rounded values for
            # accuracy
            return round(result, 14)

    @value.setter
    def value(self, value):
        #print("Servo Value: %s" % value)
        if value is None:
            self._value = None
            self._servo.controller.setTarget(self._servo.pin, 0)
        elif -1 <= value <= 1:
            self._value = value

            # Map from -1 to 1 to 0 to 1
            fraction = round((value - self._min_value) / self._value_range, 5)
            print("Value: %f" % (fraction))

            # Send command to servo controller in quarter microseconds
            pulse_width = (fraction * (self.max_pulse_width - self.min_pulse_width) +  self.min_pulse_width) * 4 * 1000000
            print("Pulse Width (quarter Microseconds): %d" % (pulse_width))
            #self._servo.setTarget(self._pin, int(pulse_width))
            self._servo.controller.setTarget(self._servo.pin, int(pulse_width))

            # Calculate the duty cycle
            #print("Duty Cycle: %f" % (
            #    self._min_dc + self._dc_range *
            #    ((value - self._min_value) / self._value_range)
            #    )
            #)
        else:
            raise ValueError(
                "Servo value must be between -1 and 1, or None")

    @property
    def is_active(self):
        return self._value is not None
        
    @value.setter
    def min_value(self, new_min_value):
        print("Min Value: %f" % (new_min_value))
        self._min_value = new_min_value

    @value.setter
    def value_range(self, new_value_range):
        print("Value Range: %f" % (new_value_range))
        self._value_range = new_value_range

    @value.setter
    def mqtt(self, mqtt):
        print("MQTT: %s" % (mqtt))
        self._mqtt = mqtt

class AngularServo(Servo):
    """
    extends :class:`servo` and represents a rotational pwm-controlled servo
    motor which can be set to particular angles (assuming valid minimum and
    maximum angles are provided to the constructor).

    connect a power source (e.g. a battery pack or the 5v pin) to the power
    cable of the servo (this is typically colored red); connect the ground
    cable of the servo (typically colored black or brown) to the negative of
    your battery pack, or a gnd pin; connect the final cable (typically colored
    white or orange) to the gpio pin you wish to use for controlling the servo.

    next, calibrate the angles that the servo can rotate to. in an interactive
    python session, construct a :class:`servo` instance. the servo should move
    to its mid-point by default. set the servo to its minimum value, and
    measure the angle from the mid-point. set the servo to its maximum value,
    and again measure the angle::

        >>> from angularservokit import servo
        >>> s = servo(0)
        >>> s.min() # measure the angle
        >>> s.max() # measure the angle

    You should now be able to construct an :class:`AngularServo` instance
    with the correct bounds::

        >>> from angularServoKit import AngularServo
        >>> s = AngularServo(0, min_angle=-42, max_angle=44)
        >>> s.angle = 0.0
        >>> s.angle
        0.0
        >>> s.angle = 15
        >>> s.angle
        15.0

    .. note::

        You can set *min_angle* greater than *max_angle* if you wish to reverse
        the sense of the angles (e.g. ``min_angle=45, max_angle=-45``). This
        can be useful with servos that rotate in the opposite direction to your
        expectations of minimum and maximum.

    :type pin: int or str
    :param pin:
        Index of the servo: 0-7 or 0-15, depending on servo driver board

    :param float initial_angle:
        Sets the servo's initial angle to the specified value. The default is
        0. The value specified must be between *min_angle* and *max_angle*
        inclusive. :data:`None` means to start the servo un-controlled (see
        :attr:`value`).

    :param float min_angle:
        Sets the minimum angle that the servo can rotate to. This defaults to
        -90, but should be set to whatever you measure from your servo during
        calibration.

    :param float max_angle:
        Sets the maximum angle that the servo can rotate to. This defaults to
        90, but should be set to whatever you measure from your servo during
        calibration.

    :param float min_pulse_width:
        The pulse width corresponding to the servo's minimum position. This
        defaults to 1ms.

    :param float max_pulse_width:
        The pulse width corresponding to the servo's maximum position. This
        defaults to 2ms.

    :param float frame_width:
        The length of time between servo control pulses measured in seconds.
        This defaults to 20ms which is a common value for servos.

    :type pin_factory: Factory or None
    :param pin_factory:
        See :doc:`api_pins` for more information (this is an advanced feature
        which most users can ignore).
    """
    def __init__(self, pin=None, *, initial_angle=0.0, min_angle=-90,
                 max_angle=90, min_pulse_width=1/1000, max_pulse_width=2/1000,
                 frame_width=20/1000, pin_factory=Factory()):
        self._min_angle = min_angle
        self._angular_range = max_angle - min_angle
        self._last_angle = None
        if initial_angle is None:
            initial_value = None
        elif ((min_angle <= initial_angle <= max_angle) or
            (max_angle <= initial_angle <= min_angle)):
            initial_value = 2 * ((initial_angle - min_angle) / self._angular_range) - 1
        else:
            raise ValueError(
                f"AngularServo angle must be between {min_angle} and "
                f"{max_angle}, or None")
        super().__init__(pin, initial_value=initial_value,
                         min_pulse_width=min_pulse_width,
                         max_pulse_width=max_pulse_width,
                         frame_width=frame_width, pin_factory=pin_factory)

    @property
    def min_angle(self):
        """
        The minimum angle that the servo will rotate to when :meth:`min` is
        called.
        """
        return self._min_angle

    @property
    def max_angle(self):
        """
        The maximum angle that the servo will rotate to when :meth:`max` is
        called.
        """
        return self._min_angle + self._angular_range

    @property
    def angle(self):
        """
        The position of the servo as an angle measured in degrees. This will
        only be accurate if :attr:`min_angle` and :attr:`max_angle` have been
        set appropriately in the constructor.

        This can also be the special value :data:`None` indicating that the
        servo is currently "uncontrolled", i.e. that no control signal is being
        sent.  Typically this means the servo's position remains unchanged, but
        that it can be moved by hand.
        """
        result = self._get_value()
        if result is None:
            return None
        else:
            # NOTE: Why round(n, 12) here instead of 14? Angle ranges can be
            # much larger than -1..1 so we need a little more rounding to
            # smooth off the rough corners!
            return round(
                self._angular_range *
                ((result - self._min_value) / self._value_range) +
                self._min_angle, 12)

    @angle.setter
    def angle(self, angle):
        if angle is None:
            self.value = None
        elif ((self.min_angle <= angle <= self.max_angle) or
              (self.max_angle <= angle <= self.min_angle)):
            self.value = (
                self._value_range *
                ((angle - self._min_angle) / self._angular_range) +
                self._min_value)

            # Publish to topic for remote micro-controller
            # Only send message if last angle and new angle aren't the same!
            if self._mqtt and (self._last_angle != angle):
                rc = self._mqtt.publish(c.MQTT_TOPIC, "%d,%.3f" % (self._pin, angle), 0) 
                #print("MQTT Publish: %d,%f,%s" % (self._pin, angle, rc))
                print("MQTT Publish: <%d,%.3f> => %s" % (self._pin, angle, c.MQTT_TOPIC))
                self._last_angle = angle
        else:
            raise ValueError(
                f"AngularServo angle must be between {self.min_angle} and "
                f"{self.max_angle}, or None")

    def close(self):
        self.angle = None
        print("Close servo!")
