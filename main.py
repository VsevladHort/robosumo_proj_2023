import sys
from time import sleep

from gpiozero import Button, LED, Motor
import program
import edge_detection
import numpy
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

DISTANCE_THRESHOLD = 700 # 70cm limit
TURNING_SPEED_FOR_CENTERING_TARGET = 0.6 # Scales the speed at which robot turns when trying to keep it's target in the center of it's FOV

print("Uploading firmware, please wait...")
vl53 = vl53l5cx.VL53L5CX()
print("Done!")
vl53.set_resolution(8 * 8)

this_program = program.ProgramStatus()
button_start = Button(17)  # these are BCM numbers
button_stop = Button(27)

motor_left = Motor(5, 6)
motor_right = Motor(13, 19)

edge_detector = edge_detection.EdgeDetector()

led = LED(22)


def start_program():
    this_program.start()


def stop_program():
    this_program.stop()


def set_up_buttons():
    button_start.when_pressed = (
        start_program  # callbacks are handled in the implicitly created callback thread
    )
    button_stop.when_released = stop_program


def handle_edge_detection():
    edge_detector_states = edge_detector.get_sensor_states()
    if edge_detector_states["top_left"] and edge_detector_states["top_right"]:
        motor_left.backward(1)
        motor_right.backward(1)  # moving straight backwards
        return True
    elif edge_detector_states["top_left"] and edge_detector_states["bottom_left"]:
        motor_left.forward(0.6)
        motor_right.forward(1)  # moving forwards while turning right
        return True
    elif edge_detector_states["top_right"] and edge_detector_states["bottom_right"]:
        motor_left.forward(1)
        motor_right.forward(0.6)  # moving forwards while turning left
        return True
    elif edge_detector_states["bottom_left"] and edge_detector_states["bottom_right"]:
        motor_left.forward(1)
        motor_right.forward(1)  # moving straights forwards
        return True
    elif edge_detector_states["top_left"]:
        motor_left.backward(1)
        motor_right.backward(0.6)  # moving backwards while turning left
        return True
    elif edge_detector_states["bottom_left"]:
        motor_left.forward(0.6)
        motor_right.forward(1)  # moving forwards while turning right
        return True
    elif edge_detector_states["top_right"]:
        motor_left.backward(1)
        motor_right.backward(0.6)  # moving backwards while turning left
        return True
    elif edge_detector_states["bottom_right"]:
        motor_left.forward(1)
        motor_right.forward(0.6)  # moving forwards while turning left
        return True
    else:
        return False


def handleOpponentSearch():
    if vl53.data_ready():
        data = vl53.get_data()

        distance = numpy.array(data.distance).reshape((8, 8))

        scalar = 0
        target_distance = 0
        n_distances = 0
        # Filter out unwanted distance values
        # Unlike in the pimoroni example, I think we should only care about ditance to the object and not it's reflectance
        for ox in range(8):
            for oy in range(8):
                d = distance[ox][oy]
                if d > DISTANCE_THRESHOLD:
                    distance[ox][oy] = 0
                else:
                    scalar += d

        # Get a total from all the distances within our accepted target
        for ox in range(8):
            for oy in range(8):
                d = distance[ox][oy]
                target_distance += d
                if d > 0:
                    n_distances += 1

        # Average the target distance
        if n_distances > 0:
            target_distance /= n_distances
        else:
            target_distance = 0

        distance = numpy.flip(distance, axis=0) # I am having a hard time visualising this, should we really be flipping the matrix?
        # Not so sure this will work, we'll have to tinker quite a bit here, I expect.

        # Calculate the center of mass along X and Y (Should probably just remove Y in the future)
        x = 0
        y = 0
        if scalar > 0:
            for ox in range(8):
                for oy in range(8):
                    y += distance[ox][oy] * ox
            y /= scalar
            y /= 3.5
            y -= 1.0

            for oy in range(8):
                for ox in range(8):
                    x += distance[ox][oy] * oy
            x /= scalar # x should be the only thing we care about when it comes to aligning the robot
            x /= 3.5 # 3.5 - average value of our coordinates
            x -= 1.0 # at this point x is in range from 0 to 2, we bring it to -1 to 1 for speed control

            print("Object detected at x: {:.2f}, y: {:.2f}".format(x, y))

            # Our robot will try to attack the target at full speed.
            print("Distance is {:.1f} mm.".format(target_distance))

            motor_left.forward(min(1 - (x * TURNING_SPEED_FOR_CENTERING_TARGET)), 1) # x > 0 will make the robot start turning right
            motor_right.forward(min(1 + (x * TURNING_SPEED_FOR_CENTERING_TARGET), 1)) # x < 0 will make the robot start turning left
            return 0 # Let's consider 0 to be an indicator that opponent is in front of the robot


if __name__ == "__main__":
    try:
        set_up_buttons()
        motor_left.stop()
        motor_right.stop()
        first_launch = True
        while True:
            if this_program.is_program_running():
                if first_launch:
                    sleep(5)  # sleep in the main thread
                    first_launch = False
                led.blink(0.5, 0.5)
                if handle_edge_detection():
                    sleep(1)  # give the robot some time to get away from the edge
            else:
                first_launch = True
                led.off()
                sleep(0.01)
    except Exception as e:
        sys.exit(
            1
        )  # gpiozero automatic cleanup for GPIO is only done when exceptions are handled
        #  https://gpiozero.readthedocs.io/en/stable/migrating_from_rpigpio.html#cleanup
