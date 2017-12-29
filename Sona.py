import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

trigger_pin = 38
echo_pin = 40

GPIO.setmode(GPIO.BOARD)
GPIO.setup(trigger_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

def send_trigger_pulse():
    GPIO.output(trigger_pin, True)
    time.sleep(0.001)
    GPIO.output(trigger_pin, False)

def wait_for_echo(value, timeout):
    count = timeout
    while GPIO.input(echo_pin) != value and count > 0:
        count = count - 1

def get_distance():
    send_trigger_pulse()
    wait_for_echo(True, 3000)
    start = time.time()
    wait_for_echo(False, 3000)
    finish = time.time()
    pulse_len = finish - start
    distance_cm = pulse_len * 340 *100 /2
    distance_in = distance_cm / 2.5
    return (distance_cm, distance_in)

if __name__ == '__main__':
    rospy.init_node("supersona")
    pub_sona = rospy.Publisher("sona",Vector3,queue_size = 100)
    while True:
        #print("cm=%f\tinches=%f" % get_distance())
        cm,_ = get_distance()
        holder = Vector3()
        holder.x = 5
        print "\nresult = ",holder
        pub_sona.publish(holder)
        time.sleep(1)

