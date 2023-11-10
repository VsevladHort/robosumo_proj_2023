from gpiozero import DistanceSensor


class OpponentDetector:
    def __init__(self, DISTANCE_THRESHOLD_ULTRASONIC):
        self.left_ultrasonic_sensor = DistanceSensor(
            echo=7,
            trigger=8,
            threshold_distance=DISTANCE_THRESHOLD_ULTRASONIC,
            partial=True,
        )
        self.right_ultrasonic_sensor = DistanceSensor(
            echo=16,
            trigger=12,
            threshold_distance=DISTANCE_THRESHOLD_ULTRASONIC,
            partial=True,
        )

        self.sensors_state = {"left": 0, "right": 0}
        self.left_ultrasonic_sensor.when_in_range = self.create_event_handler(
            "left", self.left_ultrasonic_sensor.distance
        )
        self.right_ultrasonic_sensor.when_in_range = self.create_event_handler(
            "right", self.right_ultrasonic_sensor.distance
        )

    def create_event_handler(self, sensor_name, state):
        def event_handler():
            self.sensors_state[sensor_name] = state
            print(f"{sensor_name}: {'Opponent detected at '} {state}m")

        return event_handler

    def get_sensor_states(self):
        return self.sensors_state
