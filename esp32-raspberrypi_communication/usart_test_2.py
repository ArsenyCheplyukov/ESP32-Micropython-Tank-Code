# Example # 1: Simple UART for controlling
# the state of the onboard LED
# Author: George Bantique, TechToTinker
# Date: October 7, 2020

# Load the machine module in order to access the hardware
import machine
from machine import Pin, Timer, UART, PWM
from math import sqrt, pow
from time import sleep

# motor pins array:
#
# left_forward   right_forward
# left_backward  right_backward
#
motor_pins = [[Pin(18, Pin.OUT), Pin(23, Pin.OUT)],
              [Pin(19, Pin.OUT), Pin(22, Pin.OUT)]]

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
    elif direction in "right":
        state_array = [[1, 0], [0, 1]]
    elif direction in "left":
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

# Create the led object in GPIO 2
#receive_led = machine.Pin(23, machine.Pin.OUT)
#transmit_led = machine.Pin(22, machine.Pin.OUT)
# Create the uart object in port 2
# Rx=GPIO 16, Tx=GPIO 17
Pin(16, Pin.OUT).value(0)
Pin(17, Pin.OUT).value(0)
uart = machine.UART(2, 9600)

# Create a global variable to hold the receive data in serial
strMsg = ''

# This is the main loop
while True:
    # if there is character in receive serial buffer
    if uart.any() > 0:
        # Read all the character to strMsg variable
        strMsg = uart.read()
        # Debug print to REPL
        print(strMsg)
        
        # If there is 'on' string in strMsg,
        # Turn on the LED
        if 'yes' in strMsg:
            drive(2000, "forward")
            #uart.write('Turning on LED')
            print('Turning on LED')
        # If there is 'off' string in strMsg,
        # Turn off the LED
        elif 'no' in strMsg:
            stop_all_PWMs()
            #receive_led.off()
            #uart.write('Turning off LED')
            print('Turning off LED')
        # Else, invalid command
        #else:
            #uart.write('Invalid command')
            #uart.write(strMsg.decode().strip('\r\n'))
            #print('Invalid command')
            #print(strMsg.decode().strip('\r\n'))
    sleep(2)