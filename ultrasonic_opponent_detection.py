import RPi.GPIO as GPIO
import time
import numpy


class OpponentDetector:
    def __init__(self, DISTANCE_THRESHOLD_ULTRASONIC):
        self.MIN_MEANINGFUL_DISTANCE = 3  # in cm
        self.MEASUREMENT_TIMEOUT = 0.025  # in seconds
        self.left_echo = 7
        self.left_trigger = 8
        self.right_echo = 16
        self.right_trigger = 12
        self.DISTANCE_THRESHOLD_ULTRASONIC = DISTANCE_THRESHOLD_ULTRASONIC

        self.sensors_state = {"left": 0, "right": 0}

    # get distance in cm
    def distance(self, echo, trigger):
        GPIO.output(trigger, True)

        time.sleep(0.00001)
        GPIO.output(trigger, False)

        StartTime = time.time()
        StopTime = time.time()

        while GPIO.input(echo) == 0 and StopTime - StartTime < self.MEASUREMENT_TIMEOUT:
            StartTime = time.time()
            StopTime = time.time()

        while GPIO.input(echo) == 1:
            StopTime = time.time()

        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300) / 2

        return distance

    def update_sensor_states(self):
        left_distances = []
        right_distances = []

        for _ in range(6):
            left_distance = self.distance(self.left_echo, self.left_trigger)
            right_distance = self.distance(self.right_echo, self.right_trigger)

            left_distances.append(left_distance)
            right_distances.append(right_distance)

        median_left_distance = numpy.median(left_distances)
        median_right_distance = numpy.median(right_distances)

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
