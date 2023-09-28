from machine import Pin, Timer, UART, PWM
from math import sqrt, pow
from time import sleep

# motor pins array:
#
# left_forward   right_forward
# left_backward  right_backward
#
motor_pins = [[Pin(18, Pin.OUT), Pin(22, Pin.OUT)],
              [Pin(19, Pin.OUT), Pin(23, Pin.OUT)]]

# left right
motor_enabled_PWM = PWM(Pin(4, Pin.OUT), freq=5000)

# timed interrupt
timer = Timer(1)

def stop_all_PWMs():
    global motor_enabled_PWM
    global motor_pins
    
    motor_enabled_PWM.duty(0)
    for i in [0, 1]:
        for j in [0, 1]:
            motor_pins[i][j].value(0)

def drive(driven_time, direction):
    global motor_pins
    global motor_enabled_PWM
    global timer
    state_array = [[0, 0], [0, 0]]
    if direction in "forward":
        state_array = [[1, 1], [0, 0]]
    elif direction in "left":
        state_array = [[1, 0], [0, 1]]
    elif direction in "right":
        state_array = [[0, 1], [1, 0]]
    elif direction in "backward":
        state_array = [[0, 0], [1, 1]]
    timer.deinit()
    stop_all_PWMs()
    timer = Timer(1)
    for i in [0, 1]:
        for j in [0, 1]:
            motor_pins[i][j].value(state_array[i][j])
    for i in range(0, 1024, 4):
        motor_enabled_PWM.duty(i)
    timer.init(mode=Timer.ONE_SHOT, period=driven_time, callback=stop_all_PWMs)
    

while True:
    sleep(1)
    drive(2000, "forward")
    sleep(2)
    stop_all_PWMs()
    sleep(1)