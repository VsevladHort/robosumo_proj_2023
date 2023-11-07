from time import sleep

from gpiozero import Button, LED
import program

this_program = program.ProgramStatus()
button_start = Button(17)
button_stop = Button(27)
led = LED(22)


def start_program():
    sleep(5)
    this_program.start()


def stop_program():
    this_program.stop()


def set_up_buttons():
    button_start.when_pressed = start_program
    button_stop.when_released = stop_program


if __name__ == '__main__':
    set_up_buttons()
    while True:
        if this_program.is_program_running():
            led.blink(0.5, 0.5)
        else:
            led.off()
            sleep(0.01)
