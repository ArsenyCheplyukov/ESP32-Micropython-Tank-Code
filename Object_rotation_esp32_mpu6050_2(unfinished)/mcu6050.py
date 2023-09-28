from machine import SoftI2C, Pin
from time import sleep_ms, time_ns, time
from math import atan2, sqrt, pow

filter_coefficient = 0.25
current_time = 0
yaw, gyroAngleX, gyroAngleY = (0, 0, 0)
correctional_angles = [-0.5394342637, -0.6849819303]
correctional_gyro_data = [0.2337551, 0.6541582, -1.580318]

PI = 3.141592653589793238462643383279502884197169399375105820974945

MPU6050_ADDR = 0x68

MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_XOUT_L = 0x3C
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_YOUT_L = 0x3E
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_ACCEL_ZOUT_L = 0x40
MPU6050_TEMP_OUT_H = 0x41
MPU6050_TEMP_OUT_L = 0x42
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_GYRO_XOUT_L = 0x44
MPU6050_GYRO_YOUT_H = 0x45
MPU6050_GYRO_YOUT_L = 0x46
MPU6050_GYRO_ZOUT_H = 0x47
MPU6050_GYRO_ZOUT_L = 0x48
MPU6050_PWR_MGMT_1 = 0x6B

MPU6050_LSBC = 340.0
MPU6050_TEMP_OFFSET = 36.53
MPU6050_LSBG = 16384.0
MPU6050_LSBDS = 131.0


def mpu6050_init(i2c):
    i2c.writeto_mem(MPU6050_ADDR, MPU6050_PWR_MGMT_1, bytes([0]))


def combine_register_values(h, l):
    if not h[0] & 0x80:
        return h[0] << 8 | l[0]
    return -((h[0] ^ 255) << 8) |  (l[0] ^ 255) + 1


def mpu6050_get_temp(i2c):
    temp_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_TEMP_OUT_H, 1)
    temp_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_TEMP_OUT_L, 1)
    
    return (combine_register_values(temp_h, temp_l) / MPU6050_LSBC) + MPU6050_TEMP_OFFSET


def mpu6050_get_accel(i2c):
    accel_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1)
    accel_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_L, 1)
    accel_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, 1)
    accel_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_L, 1)
    accel_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, 1)
    accel_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_L, 1)
    
    return [combine_register_values(accel_x_h, accel_x_l) / MPU6050_LSBG,
            combine_register_values(accel_y_h, accel_y_l) / MPU6050_LSBG,
            combine_register_values(accel_z_h, accel_z_l) / MPU6050_LSBG]


def mpu6050_get_gyro(i2c):
    gyro_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1)
    gyro_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_L, 1)
    gyro_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_H, 1)
    gyro_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_L, 1)
    gyro_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, 1)
    gyro_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_L, 1)
    
    return [combine_register_values(gyro_x_h, gyro_x_l) / MPU6050_LSBDS,
            combine_register_values(gyro_y_h, gyro_y_l) / MPU6050_LSBDS,
            combine_register_values(gyro_z_h, gyro_z_l) / MPU6050_LSBDS]

def get_accurate_time():
    return time()

def get_angles(i2c):
    global filter_coefficient
    global current_time
    global yaw
    global gyroAngleX
    global gyroAngleY
    AccX, AccY, AccZ = mpu6050_get_accel(i2c)
    accAngleX = atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2)))*180/PI - correctional_angles[0]
    accAngleY = atan2(-AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2)))*180/PI - correctional_angles[1]
    GyroX, GyroY, GyroZ = mpu6050_get_gyro(i2c)
    #correcting gyroscope data
    GyroX -= correctional_gyro_data[0] # GyroErrorX ~(-0.56)
    GyroY -= correctional_gyro_data[1] # GyroErrorY ~(2)
    GyroZ -= correctional_gyro_data[2] # GyroErrorZ ~ (-0.8)
    elapsedTime = get_accurate_time() - current_time if current_time != 0 else 0
    gyroAngleX += GyroX * elapsedTime # deg/s * s = deg
    gyroAngleY += GyroY * elapsedTime
    yaw += GyroZ * elapsedTime
    current_time = get_accurate_time()
    # Complementary filter - combine acceleromter and gyro angle values
    roll = (1-filter_coefficient)*gyroAngleX + filter_coefficient*accAngleX
    pitch = (1-filter_coefficient)*gyroAngleY + filter_coefficient*accAngleY
    return [roll, pitch, yaw]
    
def calibrate_coefficients(i2c, n_samples=1000):
    # xAngle, yAngle, gyroX, gyroY, gyroZ
    commulative_sum = [0, 0, 0, 0, 0]
    for _ in range(n_samples):
        AccX, AccY, AccZ = mpu6050_get_accel(i2c)
        commulative_sum[0] += atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2)))
        commulative_sum[1] += atan2(-AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2)))
        GyroX, GyroY, GyroZ = mpu6050_get_gyro(i2c)
        commulative_sum[2] += GyroX
        commulative_sum[3] += GyroY
        commulative_sum[4] += GyroZ
    for i, element in enumerate(commulative_sum):
        commulative_sum[i] /= n_samples
    return commulative_sum
        


if __name__ == "__main__":
    i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
    mpu6050_init(i2c)
    
    while True:
        #print(calibrate_coefficients(i2c, 1000))
        #print("Temperature:\t", mpu6050_get_temp(i2c), "°C")
        #print("Accelerometer:\t", mpu6050_get_accel(i2c), "g")
        #print("Gyroscope:\t", mpu6050_get_gyro(i2c), "°/s")
        print(get_angles(i2c)[0])
        # DELAY LOWER BY 5 MS
        #print(filterUpdate(*mpu6050_get_gyro(i2c), *mpu6050_get_accel(i2c), 0.1))
        sleep_ms(1)
        for _ in range(100):
            get_angles(i2c)
            sleep_ms(1)