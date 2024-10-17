import time
import audio
import config as c

import paho.mqtt.client as mqtt

# Get configuration
c.update()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    client.subscribe("$SYS/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
print("MQTT: %s:%d, %s:%s" % (c.MQTT_HOST, c.MQTT_PORT, c.MQTT_USER, c.MQTT_PWD))
client.username_pw_set(c.MQTT_USER, c.MQTT_PWD)
client.connect(c.MQTT_HOST, c.MQTT_PORT, 60)

jaw_servo = None
if c.SERVO_STYLE.lower() == 'servokit':
    import servoKit
    print("Using ServoKit!")
    # Setup servo to use with audio processor
    jaw_servo = servoKit.AngularServo(0,
        min_angle=c.MIN_ANGLE,
        max_angle=-c.MAX_ANGLE, initial_angle=None,
        min_pulse_width=c.SERVO_MIN/(1*10**6),
        max_pulse_width=c.SERVO_MAX/(1*10**6))
elif c.SERVO_STYLE.lower() == 'pololu':
    import pololu
    print("Using Pololu!")
    jaw_servo = pololu.AngularServo(0,
        min_angle=c.MIN_ANGLE,
        max_angle=-c.MAX_ANGLE, initial_angle=None,
        min_pulse_width=c.SERVO_MIN/(1*10**6),
        max_pulse_width=c.SERVO_MAX/(1*10**6))
    # Set Nod to level
    jaw_servo._controller.setTarget(6,1250*4)
    # Turn on PWM LED
    jaw_servo._controller.setTarget(11,2000*4)
    #jaw_servo._controller.runScriptSub(0)

# Set MQTT client for publishing
jaw_servo.mqtt = client

client.loop_start()
time.sleep(1)
a = audio.AUDIO(None, jaw_servo)
a.negate_angle(True)
a.play_vocal_track("./vocals/v01.wav")
client.loop_stop()

if c.SERVO_STYLE.lower() == 'servokit':
    print("Cleanup ServoKit")
elif c.SERVO_STYLE.lower() == 'pololu':
    print("Cleanup Pololu")
    jaw_servo._controller.setTarget(11,4000)
    jaw_servo._controller.setTarget(0,0)
