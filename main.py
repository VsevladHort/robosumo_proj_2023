import sys
from time import sleep

from gpiozero import Button, LED
import program

this_program = program.ProgramStatus()
button_start = Button(17)  # these are BCM numbers
button_stop = Button(27)
led = LED(22)


def start_program():
    sleep(5)  # sleeps in the callback thread, should probably avoid it
    this_program.start()


def stop_program():
    this_program.stop()


def set_up_buttons():
    button_start.when_pressed = start_program  # callbacks are handled in the implicitly created callback thread
    button_stop.when_released = stop_program


if __name__ == '__main__':
    try:
        set_up_buttons()
        while True:
            if this_program.is_program_running():
                led.blink(0.5, 0.5)
            else:
                led.off()
                sleep(0.01)
    except Exception as e:
        sys.exit(1)  # gpiozero automatic cleanup for GPIO is only done when exceptions are handled
        #  https://gpiozero.readthedocs.io/en/stable/migrating_from_rpigpio.html#cleanup
