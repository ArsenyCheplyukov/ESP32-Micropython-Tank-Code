# Example # 1: Simple UART for controlling
# the state of the onboard LED
# Author: George Bantique, TechToTinker
# Date: October 7, 2020

# Load the machine module in order to access the hardware
import machine
from time import sleep

# Create the led object in GPIO 2
led = machine.Pin(23, machine.Pin.OUT)
# Create the uart object in port 2
# Rx=GPIO 16, Tx=GPIO 17
machine.Pin(16).value(0)
machine.Pin(17).value(0)
uart = machine.UART(2, rx=16, tx=17, baudrate=115200, bits=8, parity=None, timeout=1)
led.off()
# Create a global variable to hold the receive data in serial
strMsg = ''

# This is the main loop
while True:
    # if there is character in receive serial buffer
    if uart.any() > 0:
        # Read all the character to strMsg variable
        strMsg = uart.read().decode().strip('\r\n')
        print(strMsg)
        # If there is 'on' string in strMsg,
        # Turn on the LED
        if 'no' in strMsg:
            led.off()
        # If there is 'off' string in strMsg,
        # Turn off the LED
        elif 'yes' in strMsg:
            led.on()