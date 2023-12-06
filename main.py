from time import sleep

from gpiozero import Button, RGBLED, Motor, Device
import program
import edge_detection
import RPi.GPIO as GPIO
import ultrasonic_opponent_detection
import numpy
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero.pins.pigpio import PiGPIOFactory

factory = RPiGPIOFactory()

Device.pin_factory = factory

DISTANCE_THRESHOLD = 700  # 70cm limit in mm
DISTANCE_THRESHOLD_ULTRASONIC = 70  # 70cm limit
TURNING_SPEED_FOR_SEARCHING_TARGET = (
    0.5  # speed at which to run the motors while turning during search
)
TURNING_SPEED_FOR_CENTERING_TARGET = 1.5  # Scales the speed at which robot turns when trying to keep it's target in the center of it's FOV

print("Uploading firmware, please wait...")
vl53 = vl53l5cx.VL53L5CX()
print("Done!")
vl53.set_resolution(8 * 8)
vl53.start_ranging()

this_program = program.ProgramStatus()
button_start = Button(17)  # these are BCM numbers
button_stop = Button(27)

motor_left = Motor(5, 6)
motor_right = Motor(19, 13)

edge_detector = edge_detection.EdgeDetector()

ultrasonic_opponent_detector = ultrasonic_opponent_detection.OpponentDetector(
    DISTANCE_THRESHOLD_ULTRASONIC
)

last_seen = -1  # the last direction opponent was found
# -1 - not found
# 0 - front
# 1 - left
# 2 - right

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
    motor_left.forward(TURNING_SPEED_FOR_SEARCHING_TARGET)
    motor_right.backward(TURNING_SPEED_FOR_SEARCHING_TARGET)


def turn_left():
    motor_left.backward(TURNING_SPEED_FOR_SEARCHING_TARGET)
    motor_right.forward(TURNING_SPEED_FOR_SEARCHING_TARGET)


def handle_edge_detection():
    # print("Handling edge detection")
    edge_detector_states = edge_detector.get_sensor_states()
    if edge_detector_states["top_left"] and edge_detector_states["top_right"]:
        motor_left.backward(0.5)
        motor_right.backward(1)  # moving straight backwards
        return True
    elif edge_detector_states["top_left"] and edge_detector_states["bottom_left"]:
        motor_left.forward(1)
        motor_right.backward(0.6)  # moving forwards while turning right
        return True
    elif edge_detector_states["top_right"] and edge_detector_states["bottom_right"]:
        motor_left.backward(0.5)
        motor_right.forward(1)  # moving forwards while turning left
        return True
    elif edge_detector_states["bottom_left"] and edge_detector_states["bottom_right"]:
        motor_left.forward(1)
        motor_right.forward(1)  # moving straights forwards
        return True
    elif edge_detector_states["top_left"]:
        motor_left.forward(1)
        motor_right.backward(0.6)  # moving backwards while turning left
        return True
    elif edge_detector_states["bottom_left"]:
        motor_left.backward(0.6)
        motor_right.forward(1)  # moving forwards while turning right
        return True
    elif edge_detector_states["top_right"]:
        motor_left.backward(1)
        motor_right.forward(0.6)  # moving backwards while turning left
        return True
    elif edge_detector_states["bottom_right"]:
        motor_left.backward(0.6)
        motor_right.forward(1)  # moving forwards while turning left
        return True
    else:
        return False


def consider_ultrasonic_sensors():
    # print("Considering ultrasonics")
    distances = ultrasonic_opponent_detector.get_sensor_states()
    print(distances)
    if distances["left"] > 0 and distances["right"] > 0:
        if distances["left"] < distances["right"]:
            turn_left()
            return 1
        else:
            turn_right()
            return 2
    elif distances["left"] > 0:
        turn_left()
        return 1
    elif distances["right"] > 0:
        turn_right()
        return 2
    else:
        return -1


def handle_opponent_search():
    # print("Handling opponent search)
    if vl53.data_ready():
        data = vl53.get_data()

        # print("Getting distance from data")
        distance = numpy.array(data.distance_mm).reshape((8, 8))
        status = numpy.array(data.target_status).reshape((8, 8))

        scalar = 0
        target_distance = 0
        n_distances = 0
        # Filter out unwanted distance values
        # Unlike in the pimoroni example, I think we should only care about distance to the object and not it's reflectance
        for ox in range(8):
            for oy in range(3, 8):
                d = distance[ox][oy]
                if d > DISTANCE_THRESHOLD or not (status[ox][oy] == STATUS_RANGE_VALID):
                    distance[ox][oy] = 0
                else:
                    distance[ox][oy] = (
                        DISTANCE_THRESHOLD - d
                    )  # We are insterested in closer targets having a higner weight, thus this operation
                    scalar += distance[ox][oy]

        # Get a total from all the distances within our accepted target
        for ox in range(8):
            for oy in range(3, 8):
                d = distance[ox][oy]
                target_distance += d
                if d > 0:
                    n_distances += 1

        # Average the target distance
        if n_distances > 0:
            target_distance /= n_distances
            target_distance = DISTANCE_THRESHOLD - target_distance
        else:
            target_distance = 0

        print("Flipping")
        distance = numpy.flip(distance, axis=0)

        # Calculate the center of mass along X and Y (Should probably just remove Y in the future)
        print(distance)
        x = 0
        y = 0
        if scalar > 0:
            for ox in range(8):
                for oy in range(3, 8):
                    y += distance[ox][oy] * ox
            y /= scalar
            y /= 3.5
            y -= 1.0

            for oy in range(3, 8):
                for ox in range(8):
                    x += distance[ox][oy] * oy
            x /= scalar  # x should be the only thing we care about when it comes to aligning the robot
            x /= 3.5  # 3.5 - average value of our coordinates
            x -= 1.0  # at this point x is in range from 0 to 2, we bring it to -1 to 1 for speed control

            print("Object detected at x: {:.2f}, y: {:.2f}".format(x, y))

            # # Our robot will try to attack the target at full speed.
            print("Distance is {:.1f} mm.".format(target_distance))

            # print("Regulating motors")
            left_speed = min(((1 - (x * TURNING_SPEED_FOR_CENTERING_TARGET))), 1)
            right_speed = min(((1 + (x * TURNING_SPEED_FOR_CENTERING_TARGET))), 1)
            if right_speed < -1:
                right_speed = -1
            if left_speed < -1:
                left_speed = -1
            motor_left.value = left_speed
            motor_right.value = (
                right_speed  # x < 0 will make the robot start turning left
            )

            print("Right motor speed: {:.2f}".format(right_speed))
            print("Left motor speed: {:.2f}".format(left_speed))
            return 0  # Let's consider 0 to be an indicator that opponent is in front of the robot
        else:
            return consider_ultrasonic_sensors()
    else:
        if last_seen != 0:
            return consider_ultrasonic_sensors()
        else:
            return 0


if __name__ == "__main__":
    try:
        set_up_buttons()
        motor_left.stop()
        motor_right.stop()
        first_launch = True
        print("Entered main")
        while True:
            # print(this_program.is_program_running())
            if this_program.is_program_running():
                if first_launch:
                    print("Sleeping before start")
                    sleep(5)  # sleep in the main thread
                    edge_detector.set_current_surface_as_not_edge()
                    last_seen = -1
                    first_launch = False
                led.color = (0, 1, 0)  # light up the LED green
                if handle_edge_detection():
                    sleep(1)  # give the robot some time to get away from the edge
                opponent_search_result = handle_opponent_search()
                if opponent_search_result != -1:
                    last_seen = opponent_search_result
                elif last_seen == 1:
                    turn_left()
                elif last_seen == 2:
                    turn_right()
                elif last_seen != 0:
                    turn_right()
            else:
                print("I am stopped")
                first_launch = True
                motor_left.stop()
                motor_right.stop()
                led.color = (1, 1, 0)  # light up the LED yellow
                sleep(0.01)
    except Exception as e:
        print("Exiting because of interrupt or error")
        print(e)
    finally:
        motor_left.stop()
        motor_right.stop()
        led.off()
        vl53.stop_ranging()
        GPIO.cleanup()  # this ensures a clean exit
