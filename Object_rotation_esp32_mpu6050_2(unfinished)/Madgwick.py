# Math library required for ‘sqrt’
import math
# System constants
SEq_1, SEq_2, SEq_3, SEq_4 = (1, 0, 0, 0) # estimated orientation quaternion elements with initial conditions
# Global system variables
# accelerometer measurements
# gyroscope measurements in rad/s
# sampling period in seconds
def filterUpdate(w_x, w_y, w_z, a_x, a_y, a_z, deltat):
    print("\n\n")
    gyroMeasError=3.14159265358979*(0.1/180) # gyroscope measurement error in rad/s (shown as 5 deg/s)
    beta=math.sqrt(3/4)*gyroMeasError # compute beta
    # Local system variables
    print("Beta: " + str(beta))
    global SEq_1
    global SEq_2
    global SEq_3
    global SEq_4
    #SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4 ----- quaternion derrivative from gyroscopes elements
    #f_1, f_2, f_3 ----- objective function elements
    #J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33 ------- objective function Jacobian elements
    #SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4 ------- estimated direction of the gyroscope error
    # Axulirary variables to avoid reapeated calcualtions
    print("SEq_1: " + str(SEq_1) + "; SEq_2: " + str(SEq_2) + "; SEq_3: " + str(SEq_3) + "; SEq_4: " + str(SEq_4))
    halfSEq_1, halfSEq_2, halfSEq_3, halfSEq_4 = (0.5*SEq_1, 0.5*SEq_2, 0.5*SEq_3, 0.5*SEq_4)
    twoSEq_1, twoSEq_2, twoSEq_3 = (2*SEq_1, 2*SEq_2, 2*SEq_3)
    # Normalise the accelerometer measurement
    norm = math.sqrt(a_x**2 + a_y**2 + a_z**2)
    print("Norm: ", norm)
    a_x /= norm
    a_y /= norm
    a_z /= norm
    print("Accelerations divided by norm:", a_x, a_y, a_z)
    # Compute the objective function and Jacobian
    f_1 = twoSEq_2*SEq_4 - twoSEq_1*SEq_3 - a_x
    f_2 = twoSEq_1*SEq_2 + twoSEq_3*SEq_4 - a_y
    f_3 = 1 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z
    print("Functions: f_1: " + str(f_1) + "; f2: " + str(f_2) + "; f_3: " + str(f_3))
    J_11or24 = twoSEq_3 # J_11 negated in matrix multiplication
    J_12or23 = 2*SEq_4
    J_13or22 = twoSEq_1 # J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2
    J_32 = 2*J_14or21 # negated in matrix multiplication
    J_33 = 2*J_11or24 # negated in matrix multiplication
    print("Jacobian: ")
    print(J_11or24, J_12or23, J_13or22, J_14or21)
    print(J_14or21, J_13or22, J_12or23, J_11or24)
    print(J_32, J_33)
    # Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21*f_2 - J_11or24*f_1
    SEqHatDot_2 = J_12or23*f_1 + J_13or22*f_2 - J_32*f_3
    SEqHatDot_3 = J_12or23*f_2 - J_33*f_3 - J_13or22*f_1
    SEqHatDot_4 = J_14or21*f_1 + J_11or24*f_2
    print("SEqHatDot_1: " + str(SEqHatDot_1) + "; SEqHatDot_2: " + str(SEqHatDot_2) + "; SEqHatDot_3: " + str(SEqHatDot_3) + "; SEqHatDot_4: " + str(SEqHatDot_4))
    # Normalise the gradient
    norm = math.sqrt(SEqHatDot_1**2 + SEqHatDot_2**2 + SEqHatDot_3**2 + SEqHatDot_4**2)
    print("NORM: " + str(norm))
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    print("DIVIDED BY NORM ---> SEqHatDot_1: " + str(SEqHatDot_1) + "; SEqHatDot_2: " + str(SEqHatDot_2) + "; SEqHatDot_3: " + str(SEqHatDot_3) + "; SEqHatDot_4: " + str(SEqHatDot_4))
    # Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2*w_x - halfSEq_3*w_y - halfSEq_4*w_z
    SEqDot_omega_2 = halfSEq_1*w_x + halfSEq_3*w_z - halfSEq_4*w_y
    SEqDot_omega_3 = halfSEq_1*w_y - halfSEq_2*w_z + halfSEq_4*w_x
    SEqDot_omega_4 = halfSEq_1*w_z + halfSEq_2*w_y - halfSEq_3*w_x
    print("Omega1: " + str(SEqDot_omega_1) + "; Omega2: " + str(SEqDot_omega_2) + "Omega3:" + str(SEqDot_omega_3) + "Omega4:" + str(SEqDot_omega_4))
    # Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat
    print("SEq_1: " + str(SEq_1) + "; SEq_2: " + str(SEq_2) + "; SEq_3: " + str(SEq_3) + "; SEq_4: " + str(SEq_4))
    # Normalise quaternion
    norm = math.sqrt(SEq_1**2 + SEq_2**2 + SEq_3**2 + SEq_4**2)
    SEq_1 /= norm
    SEq_2 /= norm
    SEq_3 /= norm
    SEq_4 /= norm
    print("DIVIDED BY NORM --- SEq_1:" + str(SEq_1), "; SEq_2: " + str(SEq_2) + "; SEq_3: " + str(SEq_3) + "; SEq_4: " + str(SEq_4))
    # yaw  // pitch // roll:
    return [math.atan2(2*SEq_2*SEq_3 - 2*SEq_1*SEq_4, 2*SEq_1**2 + 2*SEq_2**2 - 1)*57.3,
            -math.asin(2*SEq_2*SEq_4 + 2*SEq_1*SEq_3)*57.3,
            math.atan2(2*SEq_3*SEq_4 - 2*SEq_1*SEq_2, 2*SEq_1**2 + 2*SEq_4**2 - 1)*57.3]