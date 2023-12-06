from gpiozero import Motor

motor_left = Motor(5, 6)
motor_right = Motor(19, 13)

try:
    while True:
        motor_left.forward(1)
        motor_right.backward(1)
except KeyboardInterrupt as e:
    print(e)
