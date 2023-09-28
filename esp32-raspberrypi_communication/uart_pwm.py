from machine import Pin, Timer, UART, PWM
from math import sqrt, pow
from time import sleep_ms

# motor pins array:
#
# left_forward   right_forward
# left_backward  right_backward
#
motor_pins = [[Pin(18, Pin.OUT), Pin(22, Pin.OUT)],
              [Pin(19, Pin.OUT), Pin(23, Pin.OUT)]]

# left right
motor_enabled_PWM = PWM(Pin(4, Pin.OUT), freq=5000)

# inertia time (motors need time to reach a constant speed)
INERTIA_TIME = 125 # ms

# timed interrupt
timer = Timer(1)

def stop_all_PWMs():
    global motor_enabled_PWM
    global motor_pins
    motor_enabled_PWM.duty(0)
    #for i in [0, 1]:
    #    for j in [0, 1]:
    #        motor_pins[i][j].value(0)

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
    #for i in range(0, 1024):
    motor_enabled_PWM.duty(1023)
    sleep_ms(INERTIA_TIME)
    timer.init(mode=Timer.ONE_SHOT, period=driven_time, callback=stop_all_PWMs)
    
# Create the uart object in port 2
# Rx=GPIO 16, Tx=GPIO 17
uart = UART(2, rx=16, tx=17, baudrate=1152000, bits=8, parity=None, timeout=1)

strMsg = []

while True:
    
    MINUMUM_DELTA = 448 # PIXELS
    ROTATION_SPEED = 150 # DEGREE/SECOND
    RESOLUTION = [1920, 1080] # PIXELS
    CAMERA_ANGLE = 150 # DEGREE
    MINIMUM_RESPONSE_TIME = 150 # MILLISECONDS
    MINIMUM_FIST_SIZE_PERCENTAGE = 0.65 # IF OBJECT RATIO IS BIGGER THAN PROPOSED VALUE, NEED TO RIDE BACKWARD, CAUSE WE ARE TOO CLOSE
    MINIMUM_PALM_SIZE_PERCENTAGE = 0.87 
    
    stop_all_PWMs()
    
    # if there is character in receive serial buffer
    if uart.any() > 0:
        sleep_ms(1)
        # right, left, top, bottom, class(1-fist, 0-palm)
        # right rotation - lower, left rotation - bigger
        # resolution - 640-width; 480-height
        strMsg = [int(i) for i in uart.read().decode().strip('\r\n').split(' ')]
        print(strMsg)
        if strMsg[-1] in [0, 1]:
            strMsg = [strMsg[i] for i in [-5, -4, -3, -2, -1]]
        else:
            strMsg = [strMsg[i] for i in [0, 1, 2, 3, 4]]
        # find object's measurements
        object_height = strMsg[3] - strMsg[2]
        object_width = strMsg[0] - strMsg[1]
        # find delta from center of image (half length - center of object on image)
        # position of camera gets inversed data, so multiply by -1
        central_delta = (strMsg[0] + strMsg[1] - RESOLUTION[0])/2
        # find object's bigger part 
        object_length = object_height if object_height >= object_width else object_width
        # Is the distance to the target quite large?
        if (strMsg[4] == 0 and object_length/RESOLUTION[0] >= MINIMUM_PALM_SIZE_PERCENTAGE) or (strMsg[4] == 1 and object_length/RESOLUTION[0] >= MINIMUM_FIST_SIZE_PERCENTAGE):
            drive(MINIMUM_RESPONSE_TIME, "backward")
        # check if image delta is bigger than minimal
        elif abs(central_delta) >= MINUMUM_DELTA:
            # use angle approximation (the angle is related as the delta of the image from the length of the camera image)
            delta_angle = CAMERA_ANGLE * central_delta/RESOLUTION[0]
            # find ride time, cause we know angular velocity
            # multiply by 1000, because we need time in milliseconds 
            drive_time = (delta_angle/ROTATION_SPEED)*1000
            # choose in what direction we should drive
            if delta_angle > 0:
                drive(int(drive_time), "right")
            else:
                drive(int(abs(drive_time)), "left")
        # if delta is lower than minimum angle, we don't have reason to turn to any side
        elif abs(central_delta) < MINUMUM_DELTA:
            drive(MINIMUM_RESPONSE_TIME, "forward")