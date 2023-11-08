from gpiozero import LineSensor


class EdgeDetector:
    def __init__(self):
        # Initialize sensors with their respective GPIO pins
        self.edge_detector_top_left = LineSensor(18)
        self.edge_detector_top_right = LineSensor(23)
        self.edge_detector_bottom_left = LineSensor(24)
        self.edge_detector_bottom_right = LineSensor(25)

        # Initialize the state of each sensor
        self.sensors_state = {
            'top_left': False,
            'top_right': False,
            'bottom_left': False,
            'bottom_right': False
        }

        # Setup event handlers for all sensors
        self.edge_detector_top_left.when_line = self.create_event_handler('top_left', True)
        self.edge_detector_top_left.when_no_line = self.create_event_handler('top_left', False)
        self.edge_detector_top_right.when_line = self.create_event_handler('top_right', True)
        self.edge_detector_top_right.when_no_line = self.create_event_handler('top_right', False)
        self.edge_detector_bottom_left.when_line = self.create_event_handler('bottom_left', True)
        self.edge_detector_bottom_left.when_no_line = self.create_event_handler('bottom_left', False)
        self.edge_detector_bottom_right.when_line = self.create_event_handler('bottom_right', True)
        self.edge_detector_bottom_right.when_no_line = self.create_event_handler('bottom_right', False)

    def create_event_handler(self, sensor_name, state):
        # Return a function that updates the state of the sensor
        def event_handler():
            self.sensors_state[sensor_name] = state
            print(f"{sensor_name.replace('_', ' ').title()}: {'Line detected' if state else 'No line'}")

        return event_handler

    def get_sensor_states(self):
        # Return the current states of all sensors
        return self.sensors_state