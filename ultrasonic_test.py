import RPi.GPIO as GPIO
import ultrasonic_opponent_detection
from time import sleep

GPIO.setmode(GPIO.BCM)

detector = ultrasonic_opponent_detection.OpponentDetector(70)
try:
    while True:
        print(detector.get_sensor_states())
        sleep(1)
except KeyboardInterrupt as e:
    print(e)
finally:
    GPIO.cleanup()
