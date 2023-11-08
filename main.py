import sys
from time import sleep

from gpiozero import Button, LED, LineSensor
import program
import edge_detection

this_program = program.ProgramStatus()
button_start = Button(17)  # these are BCM numbers
button_stop = Button(27)

edge_detector = edge_detection.EdgeDetector()

led = LED(22)


def start_program():
    this_program.start()


def stop_program():
    this_program.stop()


def set_up_buttons():
    button_start.when_pressed = start_program  # callbacks are handled in the implicitly created callback thread
    button_stop.when_released = stop_program


if __name__ == '__main__':
    try:
        set_up_buttons()
        first_launch = True
        while True:
            if this_program.is_program_running():
                if first_launch:
                    sleep(5)  # sleep in the main thread
                    first_launch = False
                led.blink(0.5, 0.5)
            else:
                first_launch = True
                led.off()
                sleep(0.01)
    except Exception as e:
        sys.exit(1)  # gpiozero automatic cleanup for GPIO is only done when exceptions are handled
        #  https://gpiozero.readthedocs.io/en/stable/migrating_from_rpigpio.html#cleanup
