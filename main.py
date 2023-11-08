import sys
from time import sleep

from gpiozero import Button, LED, Motor
import program
import edge_detection

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
    button_start.when_pressed = start_program  # callbacks are handled in the implicitly created callback thread
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


if __name__ == '__main__':
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
        sys.exit(1)  # gpiozero automatic cleanup for GPIO is only done when exceptions are handled
        #  https://gpiozero.readthedocs.io/en/stable/migrating_from_rpigpio.html#cleanup
