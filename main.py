from time import sleep, time

from gpiozero import Button, RGBLED, Motor, Device
from enum import Enum, auto
import program
import edge_detection
import RPi.GPIO as GPIO
import ultrasonic_opponent_detection

# import numpy
# import vl53l5cx_ctypes as vl53l5cx
# from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

from gpiozero.pins.rpigpio import RPiGPIOFactory

factory = RPiGPIOFactory()

Device.pin_factory = factory

# Colors of LED meaning:
# RED - stopped or handling detected edge
# GREEN - searching for target
# BLUE - attacking target

DISTANCE_THRESHOLD = 700  # 70cm limit in mm
DISTANCE_THRESHOLD_ULTRASONIC = 70  # 70cm limit
TURNING_SPEED_FOR_SEARCHING_TARGET = (
    0.6  # speed at which to run the motors while turning during search
)
TURNING_SPEED_FOR_CENTERING_TARGET = 1.0  # Scales the speed at which robot turns when trying to keep it's target in the center of it's FOV

MAX_SPEED = 0.75

# print("Uploading firmware, please wait...")
# vl53 = vl53l5cx.VL53L5CX()
# print("Done!")
# vl53.set_resolution(8 * 8)
# vl53.set_ranging_frequency_hz(60)
# vl53.set_integration_time_ms(
#     5
# )  # the amount of time for a reading, cannot be greater than what is possible due to set frequency
# # maybe it will help us with precision?..
# vl53.start_ranging()

this_program = program.ProgramStatus()
button_start = Button(17)  # these are BCM numbers
button_stop = Button(27)

motor_left = Motor(6, 5)
motor_right = Motor(19, 13)

edge_detector = edge_detection.EdgeDetector()

ultrasonic_opponent_detector = ultrasonic_opponent_detection.OpponentDetector(
    DISTANCE_THRESHOLD_ULTRASONIC
)


class Direction(Enum):
    NOT_FOUND = auto()
    FRONT = auto()
    LEFT = auto()
    RIGHT = auto()
    BACK = auto()


last_seen = Direction.NOT_FOUND  # the last direction opponent was found

led = RGBLED(10, 9, 11)


def start_program():
    print("starting program")
    this_program.start()


def stop_program():
    print("stopping program")
    motor_left.stop()
    motor_right.stop()
    led.value = (1, 0, 0)  # red - emergency stop underway
    this_program.stop()


def set_up_buttons():
    button_start.when_activated = (
        start_program  # callbacks are handled in the implicitly created callback thread
    )
    button_stop.when_activated = stop_program


def turn_right():
    motor_left.backward(TURNING_SPEED_FOR_SEARCHING_TARGET)
    motor_right.forward(TURNING_SPEED_FOR_SEARCHING_TARGET)


def turn_left():
    motor_left.forward(TURNING_SPEED_FOR_SEARCHING_TARGET)
    motor_right.backward(TURNING_SPEED_FOR_SEARCHING_TARGET)


def any_edge_detected():
    edge_detector_states = edge_detector.get_sensor_states()
    return (
        edge_detector_states["top_left"]
        or edge_detector_states["top_right"]
        or edge_detector_states["bottom_left"]
        or edge_detector_states["bottom_right"]
    )


def launch_smart_sleep(timeout):
    StartTime = time()
    StopTime = time()
    continueFlag = False
    while StopTime - StartTime < timeout:
        if any_edge_detected():
            continueFlag = True
            break
        StopTime = time()
    return continueFlag


def handle_edge_detection():
    while True:
        led.color = (1, 0, 0)
        edge_detector_states = edge_detector.get_sensor_states()
        if edge_detector_states["top_left"] and edge_detector_states["top_right"]:
            motor_left.backward(0.7)
            motor_right.backward(0.7)
            if launch_smart_sleep(0.4):
                continue
            motor_left.forward(0.7)
            motor_right.backward(0.7)
            if launch_smart_sleep(0.6):
                continue
            return True
        elif edge_detector_states["top_left"] and edge_detector_states["bottom_left"]:
            motor_left.forward(0.7)
            motor_right.backward(0.7)
            if launch_smart_sleep(0.3):
                continue
            motor_left.forward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.1):
                continue
            return True
        elif edge_detector_states["top_right"] and edge_detector_states["bottom_right"]:
            motor_left.backward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.3):
                continue
            motor_left.forward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.1):
                continue
            return True
        elif (
            edge_detector_states["bottom_left"] and edge_detector_states["bottom_right"]
        ):
            motor_left.forward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.1):
                continue
            return True
        elif edge_detector_states["top_left"]:
            motor_left.backward(0.5)
            motor_right.backward(0.5)
            if launch_smart_sleep(0.7):
                continue
            motor_left.forward(0.7)
            motor_right.backward(0.7)
            if launch_smart_sleep(1):
                continue
            motor_left.forward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.3):
                continue
            return True
        elif edge_detector_states["bottom_left"]:
            motor_left.forward(0.5)
            motor_right.forward(0.5)
            if launch_smart_sleep(0.7):
                continue
            motor_left.forward(0.7)
            motor_right.backward(0.7)
            if launch_smart_sleep(1):
                continue
            motor_left.forward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.3):
                continue
            return True
        elif edge_detector_states["top_right"]:
            motor_left.backward(0.5)
            motor_right.backward(0.5)
            if launch_smart_sleep(0.7):
                continue
            motor_left.backward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(1):
                continue
            motor_left.forward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.3):
                continue
            return True
        elif edge_detector_states["bottom_right"]:
            motor_left.forward(0.5)
            motor_right.forward(0.5)
            if launch_smart_sleep(0.7):
                continue
            motor_left.backward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.7):
                continue
            motor_left.forward(0.7)
            motor_right.forward(0.7)
            if launch_smart_sleep(0.3):
                continue
            return True
        else:
            return False


def consider_ultrasonic_sensors():
    distances = ultrasonic_opponent_detector.get_sensor_states()
    if distances["back"] < 50 and distances["back"] > 0:
        return Direction.FRONT
    if distances["left"] > 0 and distances["right"] > 0:
        if distances["left"] < distances["right"]:
            return Direction.LEFT
        else:
            return Direction.RIGHT
    elif distances["left"] > 0:
        return Direction.LEFT
    elif distances["right"] > 0:
        return Direction.RIGHT
    else:
        return Direction.NOT_FOUND


def handle_opponent_search():
    # if vl53.data_ready():
    #     data = vl53.get_data()

    #     distance = numpy.array(data.distance_mm).reshape((8, 8))
    #     status = numpy.array(data.target_status).reshape((8, 8))

    #     scalar = 0  # sum of all valid distances
    #     target_distance = 0  # average distance to target
    #     n_distances = 0  # number of valid distance readings
    #     for ox in range(8):
    #         for oy in range(8):
    #             d = distance[ox][oy]
    #             if d > DISTANCE_THRESHOLD or not (status[ox][oy] == STATUS_RANGE_VALID):
    #                 distance[ox][oy] = 0
    #             else:
    #                 distance[ox][oy] = (
    #                     DISTANCE_THRESHOLD - d
    #                 )  # We are insterested in closer targets having a higher weight, thus this operation

    #     # Get a total from all the distances within our accepted target
    #     for ox in range(4):
    #         for oy in range(8):
    #             d = distance[ox][oy]
    #             target_distance += d
    #             if d > 0:
    #                 n_distances += 1

    #     scalar = target_distance

    #     # Average the target distance
    #     if n_distances > 0:
    #         target_distance /= n_distances
    #         target_distance = DISTANCE_THRESHOLD - target_distance
    #     else:
    #         target_distance = 0

    #     # Calculate the center of mass along horizontal and vertical axes
    #     # horizontal coordinates should be the only thing we care about when it comes to aligning the robot
    #     # 3.5 - average value of our coordinates
    #     print(distance)
    #     vertical = 0
    #     horizontal = 0
    #     if scalar > 0 and n_distances > 4:
    #         for ox in range(4, 8):
    #             for oy in range(8):
    #                 horizontal += distance[ox][oy] * ox
    #         horizontal /= scalar
    #         horizontal /= 3.5
    #         horizontal -= 1.0  # at this point horizontal is in range from 0 to 2, we bring it to -1 to 1 for speed control

    #         for oy in range(4, 8):
    #             for ox in range(8):
    #                 vertical += distance[oy][ox] * oy
    #         vertical /= scalar
    #         vertical /= 3.5
    #         vertical -= 1.0

    #         print(
    #             "Object detected at x: {:.2f}, y: {:.2f}".format(horizontal, vertical)
    #         )

    #         print("Distance is {:.1f} mm.".format(target_distance))

    #         print("Regulating motors")
    #         left_speed = min(
    #             ((MAX_SPEED - (horizontal * TURNING_SPEED_FOR_CENTERING_TARGET))),
    #             MAX_SPEED,
    #         )
    #         right_speed = min(
    #             ((MAX_SPEED + (horizontal * TURNING_SPEED_FOR_CENTERING_TARGET))),
    #             MAX_SPEED,
    #         )
    #         if right_speed < -MAX_SPEED:
    #             right_speed = -MAX_SPEED
    #         if left_speed < -MAX_SPEED:
    #             left_speed = -MAX_SPEED

    #         motor_left.value = MAX_SPEED
    #         motor_right.value = MAX_SPEED

    #         # if distance is sufficiently short, just attack at full speed.
    #         if target_distance < 200:
    #             motor_left.value = 1
    #             motor_right.value = 1

    #         print("Right motor speed: {:.2f}".format(motor_left.value))
    #         print("Left motor speed: {:.2f}".format(motor_right.value))

    #         return Direction.FRONT
    #     else:
    #         print("I should be printed")
    #         return consider_ultrasonic_sensors()
    # else:
    if last_seen != Direction.FRONT:
        return consider_ultrasonic_sensors()
    else:
        return last_seen


def launch_search_routine():
    opponent_search_result = handle_opponent_search()
    last_seen = opponent_search_result
    print(last_seen)
    if opponent_search_result == Direction.FRONT:
        motor_left.value = -MAX_SPEED
        motor_right.value = -MAX_SPEED
        led.color = (0, 0, 1)
    if opponent_search_result == Direction.LEFT:
        turn_left()
        led.color = (0, 1, 0)  # light up the LED green
    elif opponent_search_result == Direction.RIGHT:
        turn_right()
        led.color = (0, 1, 0)  # light up the LED green
    elif opponent_search_result == Direction.BACK:
        led.color = (0, 0, 1)  # light up the LED blue
        motor_left.value = -1
        motor_right.value = -1
    elif opponent_search_result != Direction.FRONT:
        if last_seen == Direction.RIGHT:
            led.color = (0, 1, 0)  # light up the LED green
            turn_right()
        elif last_seen == Direction.LEFT:
            led.color = (0, 1, 0)  # light up the LED green
            turn_left()
        elif last_seen == Direction.FRONT:
            led.color = (0, 0, 1)  # light up the LED blue
            motor_left.value = -MAX_SPEED
            motor_right.value = -MAX_SPEED
    if opponent_search_result != Direction.NOT_FOUND:
        last_seen = opponent_search_result


if __name__ == "__main__":
    try:
        set_up_buttons()
        motor_left.stop()
        motor_right.stop()
        first_launch = True
        print("Entered main")
        while True:
            if this_program.is_program_running():
                if first_launch:
                    print("Sleeping before start")
                    sleep(5)  # sleep in the main thread
                    edge_detector.set_current_surface_as_not_edge()
                    last_seen = Direction.NOT_FOUND
                    first_launch = False
                is_edge = handle_edge_detection()
                if is_edge:
                    led.color = (1, 0, 0)
                if not is_edge:
                    launch_search_routine()
            else:
                first_launch = True
                motor_left.stop()
                motor_right.stop()
                led.color = (1, 0, 0)  # light up the LED RED
                sleep(0.01)
    except Exception as e:
        print("Exiting because of interrupt or error")
        print(e)
    finally:
        motor_left.stop()
        motor_right.stop()
        led.off()
        # vl53.stop_ranging()
        GPIO.cleanup()  # this ensures a clean exit
