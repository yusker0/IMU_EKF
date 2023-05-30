from model import IMUModel
from serialparser import SerialParser
from time import time
from filtering import imuFilter
import math

RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

parser = SerialParser()
parser.get_data()


gyro_roll = 0
gyro_pitch = 0
gyro_yaw = 0
prevTime = time()

complementary_roll = 0
complementary_pitch = 0
complementary_yaw = 0
KF = 0.6

accelModel = IMUModel("Accelerometer", [-30, 0, 0])
gyroModel = IMUModel("Gyroscope", [-10, 0, 0])
complementaryModel = IMUModel("Complementary filter", [10, 0, 0])
EKFModel = IMUModel("EKF", [30, 0, 0])


def print_angles(roll, pitch, yaw, convert=1, end="\t"):
    roll *= convert
    pitch *= convert
    yaw *= convert
    print("{:3.2f}".format(roll) if roll < 0 else " {:3.2f}".format(roll),
          "{:3.2f}".format(pitch) if pitch < 0 else " {:3.2f}".format(pitch),
          "{:3.2f}".format(yaw) if yaw < 0 else " {:3.2f}".format(yaw), end=end)


while 1:
    data = parser.get_data()

    acc_roll = math.atan2(data['acc']['y'], math.sqrt(data['acc']['z'] ** 2 + data['acc']['x'] ** 2))
    acc_pitch = -math.atan2(data['acc']['x'], math.sqrt(data['acc']['z'] ** 2 + data['acc']['y'] ** 2))
    acc_yaw = math.atan2(math.sqrt(data['acc']['x'] ** 2 + data['acc']['y'] ** 2), data['acc']['z'])

    newTime = time()
    deltaTime = newTime - prevTime
    delta_roll = data['gyro']['x'] * deltaTime * DEG_TO_RAD
    delta_pitch = data['gyro']['y'] * deltaTime * DEG_TO_RAD
    delta_yaw = data['gyro']['z'] * deltaTime * DEG_TO_RAD
    gyro_roll += delta_roll
    gyro_pitch += delta_pitch
    gyro_yaw += delta_yaw
    prevTime = newTime

    complementary_roll = KF * (complementary_roll + delta_roll) + (1 - KF) * acc_roll
    complementary_pitch = KF * (complementary_pitch + delta_pitch) + (1 - KF) * acc_pitch
    complementary_yaw = KF * (complementary_yaw + delta_yaw) + (1 - KF) * acc_yaw

    z = [data['acc']['x'], data['acc']['y'], data['acc']['z'], data['gyro']['x'], data['gyro']['y'], data['gyro']['z']]
    imuFilter.dt = deltaTime
    imuFilter.predict()
    imuFilter.update(z)
    EKF_roll, EKF_pitch, EKF_yaw, EKF_roll_vel, EKF_pitch_vel, EKF_yaw_vel = imuFilter.x

    print_angles(gyro_roll, gyro_pitch, gyro_yaw)
    print_angles(data['acc']['x'], data['acc']['y'], data['acc']['z'], end="\n")

    accelModel.setOrientation([acc_roll, acc_pitch, acc_yaw])
    gyroModel.setOrientation([gyro_roll, gyro_pitch, gyro_yaw])
    complementaryModel.setOrientation([complementary_roll, complementary_pitch, gyro_yaw])
    EKFModel.setOrientation([EKF_roll, EKF_pitch, EKF_yaw])

    print_angles(acc_roll, acc_pitch, acc_yaw)
    print_angles(gyro_roll, gyro_pitch, gyro_yaw)
    print_angles(complementary_roll, complementary_pitch, complementary_yaw)
    print_angles(EKF_roll, EKF_pitch, EKF_yaw, end="\n")

