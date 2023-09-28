from machine import Pin, Timer, UART, PWM, reset
from math import sqrt, pow
from time import sleep_ms
from _thread import start_new_thread, allocate_lock

# message that we get through uart
strMsg = []
# init thread lock
lock = allocate_lock()

def stop_all_motors(motor_pins):
    for i in [0, 1]:
        for j in [0, 1]:
            motor_pins[i][j].value(0)

def drive(driven_time, direction, motor_pins, INERTIA_TIME):
    state_array = [[0, 0], [0, 0]]
    if direction == "forward":
        state_array = [[1, 1], [0, 0]]
    elif direction == "left":
        state_array = [[1, 0], [0, 1]]
    elif direction == "right":
        state_array = [[0, 1], [1, 0]]
    elif direction == "backward":
        state_array = [[0, 0], [1, 1]]
    for i in [0, 1]:
        for j in [0, 1]:
            motor_pins[i][j].value(state_array[i][j])
    sleep_ms(INERTIA_TIME + driven_time)
    stop_all_motors(motor_pins)

def listenUART():
    # Create the uart object in port 2
    # Rx=GPIO 16, Tx=GPIO 17
    uart = UART(2, rx=16, tx=17, baudrate=1152000, bits=8, parity=None, timeout=1)
    while True:
        if uart.any() > 0:
            try:
                sleep_ms(2)
                # right, left, top, bottom, class(1-fist, 0-palm)
                # right rotation - lower, left rotation - bigger
                # resolution - 1920-width; 1080-height
                staff = [int(i) for i in uart.read().decode().strip('\r\n').split(' ')]
                
                print(*staff)
                if staff[-1] in [0, 1]:
                    staff = [staff[i] for i in [-5, -4, -3, -2, -1]]
                elif staff[-6] in [0, 1]:
                    staff = [staff[i] for i in [-10, -9, -8, -7, -6]]
                global strMsg
                lock.acquire()
                strMsg = staff
                lock.release()
            except:
                reset()

def ride():
    # inertia time (motors need time to reach a constant speed)
    INERTIA_TIME = 50 # ms
    
    #                    motor pins array:
    #              left_forward      right_forward
    #              left_backward     right_backward
    motor_pins = [[Pin(18, Pin.OUT), Pin(22, Pin.OUT)],
                  [Pin(19, Pin.OUT), Pin(23, Pin.OUT)]]
    # set enable high
    Pin(4, Pin.OUT).value(1)
    # stop all motors at start
    stop_all_motors(motor_pins)
    while True:
        MINUMUM_DELTA = 100 # PIXELS
        ROTATION_SPEED = 160 # DEGREE/SECOND
        RESOLUTION = [1920, 1080] # PIXELS
        CAMERA_ANGLE = 146 # DEGREE
        MINIMUM_RESPONSE_TIME = 250 # MILLISECONDS
        MINIMUM_FIST_SIZE_PERCENTAGE = 0.8 # IF OBJECT RATIO IS BIGGER THAN PROPOSED VALUE, NEED TO RIDE BACKWARD, CAUSE WE ARE TOO CLOSE
        MINIMUM_PALM_SIZE_PERCENTAGE = 0.9
        reduce_coefficient = 0.2 # coefficient of proportionality between the time of movement forward and backward
        
        # find object's measurements
        global strMsg
        lock.acquire()
        staff = strMsg
        strMsg = []
        lock.release()
        if staff:
            try:
                right, left, top, bottom, class_name = staff
                
                object_height = abs(top - bottom)
                object_width = abs(right - left)
                # find delta from center of image (half length - center of object on image)
                # position of camera gets inversed data, so multiply by -1
                central_delta = (right + left - RESOLUTION[0])/2
                # find object's bigger part 
                object_length = object_height if object_height >= object_width else object_width
                # Is the distance to the target quite large?
                if (class_name == 0 and ((object_length/RESOLUTION[0]) >= MINIMUM_PALM_SIZE_PERCENTAGE)) or (class_name == 1 and ((object_length/RESOLUTION[0]) >= MINIMUM_FIST_SIZE_PERCENTAGE)):
                    drive(int(reduce_coefficient*MINIMUM_RESPONSE_TIME), "backward", motor_pins, INERTIA_TIME)
                # check if image delta is bigger than minimal
                elif abs(central_delta) >= MINUMUM_DELTA:
                    # use angle approximation (the angle is related as the delta of the image from the length of the camera image)
                    delta_angle = CAMERA_ANGLE * central_delta/RESOLUTION[0]
                    # find ride time, cause we know angular velocity
                    # multiply by 1000, because we need time in milliseconds 
                    drive_time = (delta_angle/ROTATION_SPEED)*1000
                    # choose in what direction we should drive
                    if delta_angle > 0:
                        drive(int(drive_time), "right", motor_pins, INERTIA_TIME)
                    else:
                        drive(int(abs(drive_time)), "left", motor_pins, INERTIA_TIME)
                # if delta is lower than minimum angle, we don't have reason to turn to any side
                elif abs(central_delta) < MINUMUM_DELTA:
                    drive(MINIMUM_RESPONSE_TIME, "forward", motor_pins, INERTIA_TIME)
            except:
                reset()
            
            

if __name__ == "__main__":
    start_new_thread(listenUART, ())
    start_new_thread(ride, ())