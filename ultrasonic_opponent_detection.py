import RPi.GPIO as GPIO
import time
import numpy


class OpponentDetector:
    def __init__(self, DISTANCE_THRESHOLD_ULTRASONIC):
        self.MIN_MEANINGFUL_DISTANCE = 2  # in cm
        self.MEASUREMENT_TIMEOUT = 0.025  # in seconds
        self.ECHO_LISTENING_TIMEOUT = 0.1  # in seconds
        self.left_echo = 7
        self.left_trigger = 8
        self.right_echo = 16
        self.right_trigger = 12
        self.back_echo = 21
        self.back_trigger = 20
        self.DISTANCE_THRESHOLD_ULTRASONIC = DISTANCE_THRESHOLD_ULTRASONIC
        self.DISTANCE_THRESHOLD_BACK = 50

        self.sensors_state = {"left": 0, "right": 0, "back": 0}

        GPIO.setup(self.left_echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.left_trigger, GPIO.OUT)
        GPIO.setup(self.right_echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.right_trigger, GPIO.OUT)
        GPIO.setup(self.back_echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.back_trigger, GPIO.OUT)

    # get distance in cm
    def distance(self, echo, trigger):
        GPIO.output(trigger, True)

        time.sleep(0.00001)
        GPIO.output(trigger, False)

        StartTime = time.time()
        StopTime = time.time()

        trigStartTime = time.time()

        while (
            GPIO.input(echo) == 0
            and StopTime - trigStartTime < self.MEASUREMENT_TIMEOUT
        ):
            StartTime = time.time()
            StopTime = time.time()

        RealStartTime = StartTime

        while (
            GPIO.input(echo) == 1
            and trigStartTime - StopTime < self.ECHO_LISTENING_TIMEOUT
        ):
            StopTime = time.time()

        TimeElapsed = StopTime - RealStartTime
        distance = (TimeElapsed * 34300) / 2

        return distance

    def update_sensor_states(self):
        left_distances = []
        right_distances = []
        back_distances = []

        for _ in range(4):
            left_distance = self.distance(self.left_echo, self.left_trigger)
            right_distance = self.distance(self.right_echo, self.right_trigger)
            back_distance = self.distance(self.back_echo, self.back_trigger)

            left_distances.append(left_distance)
            right_distances.append(right_distance)
            back_distances.append(back_distance)

        median_left_distance = numpy.median(left_distances)
        median_right_distance = numpy.median(right_distances)
        median_back_distance = numpy.median(back_distances)

        if (
            self.MIN_MEANINGFUL_DISTANCE
            <= median_back_distance
            <= self.DISTANCE_THRESHOLD_ULTRASONIC
        ):
            self.sensors_state["back"] = median_back_distance
        else:
            self.sensors_state["back"] = 0

        if (
            self.MIN_MEANINGFUL_DISTANCE
            <= median_left_distance
            <= self.DISTANCE_THRESHOLD_ULTRASONIC
        ):
            self.sensors_state["left"] = median_left_distance
        else:
            self.sensors_state["left"] = 0

        if (
            self.MIN_MEANINGFUL_DISTANCE
            <= median_right_distance
            <= self.DISTANCE_THRESHOLD_ULTRASONIC
        ):
            self.sensors_state["right"] = median_right_distance
        else:
            self.sensors_state["right"] = 0

    def get_sensor_states(self):
        self.update_sensor_states()
        return self.sensors_state
