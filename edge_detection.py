import RPi.GPIO as GPIO


class EdgeDetector:
    def __init__(self):
        self.GPIO_PIN_left_top = 18
        self.GPIO_PIN_right_top = 23
        self.GPIO_PIN_left_back = 24
        self.GPIO_PIN_right_back = 25
        self.current_surface_is_line = False

        GPIO.setup(self.GPIO_PIN_left_top, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.GPIO_PIN_right_top, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.GPIO_PIN_left_back, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.GPIO_PIN_right_back, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def get_sensor_states(self):
        if self.current_surface_is_line:
            return {
                "top_left": not GPIO.input(self.GPIO_PIN_left_top),
                "top_right": not GPIO.input(self.GPIO_PIN_right_top),
                "bottom_left": not GPIO.input(self.GPIO_PIN_left_back),
                "bottom_right": not GPIO.input(self.GPIO_PIN_right_back),
            }
        else:
            return {
                "top_left": GPIO.input(self.GPIO_PIN_left_top),
                "top_right": GPIO.input(self.GPIO_PIN_right_top),
                "bottom_left": GPIO.input(self.GPIO_PIN_left_back),
                "bottom_right": GPIO.input(self.GPIO_PIN_right_back),
            }

    def set_current_surface_as_not_edge(self):
        if GPIO.input(self.GPIO_PIN_left_top):
            self.current_surface_is_line = True
        else:
            self.current_surface_is_line = False


"""
from gpiozero import LineSensor


class EdgeDetector:
    def __init__(self):
        # Initialize sensors with their respective GPIO pins
        self.edge_detector_top_left = LineSensor(18)
        self.edge_detector_top_right = LineSensor(23)
        self.edge_detector_bottom_left = LineSensor(24)
        self.edge_detector_bottom_right = LineSensor(25)
        self.current_surface_is_line = False

        # Initialize the state of each sensor
        self.sensors_state = {
            "top_left": False,
            "top_right": False,
            "bottom_left": False,
            "bottom_right": False,
        }

        # Setup event handlers for all sensors
        self.edge_detector_top_left.when_line = self.create_event_handler(
            "top_left", True
        )
        self.edge_detector_top_left.when_no_line = self.create_event_handler(
            "top_left", False
        )
        self.edge_detector_top_right.when_line = self.create_event_handler(
            "top_right", True
        )
        self.edge_detector_top_right.when_no_line = self.create_event_handler(
            "top_right", False
        )
        self.edge_detector_bottom_left.when_line = self.create_event_handler(
            "bottom_left", True
        )
        self.edge_detector_bottom_left.when_no_line = self.create_event_handler(
            "bottom_left", False
        )
        self.edge_detector_bottom_right.when_line = self.create_event_handler(
            "bottom_right", True
        )
        self.edge_detector_bottom_right.when_no_line = self.create_event_handler(
            "bottom_right", False
        )

    def create_event_handler(self, sensor_name, state):
        # Return a function that updates the state of the sensor
        def event_handler():
            self.sensors_state[sensor_name] = state
            print(
                f"{sensor_name.replace('_', ' ').title()}: {'Line detected' if state else 'No line'}"
            )

        return event_handler

    def get_sensor_states(self):
        if self.current_surface_is_line:
            return {
                "top_left": not self.sensors_state["top_left"],
                "top_right": not self.sensors_state["top_right"],
                "bottom_left": not self.sensors_state["bottom_left"],
                "bottom_right": not self.sensors_state["bottom_right"],
            }
        else:
            return self.sensors_state

    def set_current_surface_as_not_edge(self):
        if self.edge_detector_top_left.line_detected:
            self.current_surface_is_line = True
        else:
            self.current_surface_is_line = False
"""
