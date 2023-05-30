import numpy as np
import matplotlib.pyplot as plt
from filtering import EKF

# Симуляция равноускоренного движения
acceleration = 5
trueStates = [[0, 0, 0]]  # position, velocity, acceleration
measurements = [[0, 0]]  # velocity, acceleration
filteredStates = [[0, 0, 0]]  # position, velocity, acceleration
time = 5  # 10 seconds
dt = 0.01  # 10 ms

velocitySigma = 2
accelerationSigma = 5


def getMeasurment(state):
    newState = [state[0] + state[1] * dt + 1 / 2 * acceleration * dt ** 2,
                state[1] + acceleration * dt,
                acceleration]
    velocityNoise = np.random.normal(0, velocitySigma)
    accelerationNoise = np.random.normal(0, accelerationSigma)
    measurement = [newState[1] + velocityNoise, newState[2] + accelerationNoise]
    return newState, measurement


def f(x, dt=0.01):
    new = [x[0] + x[1] * dt + 1 / 2 * x[2] * dt ** 2,
           x[1] + x[2] * dt,
           x[2]]
    return new


def fJ(x, dt=0.01):
    return np.matrix([[1, dt, 0.5 * dt ** 2],
                      [0, 1, dt],
                      [0, 0, 1]])


def h(x):
    return np.array([x[1], x[2]])


def hJ(x):
    return np.matrix([[0, 1, 0],
                     [0, 0, 1]])


filter = EKF([0, 0, 0], 2, f, fJ, h, hJ)
filter.Q = np.matrix([[dt**5/20, dt**4/8, dt**3/6],
                      [dt**4/8, dt**3/3, dt**2/2],
                      [dt**3/6, dt**2/2, dt]])

filter.R = np.diag([velocitySigma ** 2, accelerationSigma ** 2])

filter.P = np.array([[10.0, 0.0, 0.0],
                     [0.0, 10.0, 0.0],
                     [0.0, 0.0, 10.0]])
filter.dt = dt


for i in range(1, round(time / dt)):
    x, z = getMeasurment(trueStates[i - 1])
    filter.predict()
    filter.update(z)

    trueStates.append(x)
    measurements.append(z)
    filteredStates.append(filter.x)


trueStates = np.array(trueStates)
measurements = np.array(measurements)
filteredStates = np.array(filteredStates)

figure, axis = plt.subplots(1, 2)

# axis[0].title("Kalman Filter - position")
axis[0].plot(trueStates[1:, 0], label="Истинное значение ", color="#FF6633")
axis[0].plot(filteredStates[1:, 0], label="Оценка фильтра", color="#224411")
axis[0].set_title("position")
axis[0].legend()

# axis[1].title("Kalman Filter - velocity")
axis[1].plot(trueStates[1:, 1], label="Истинное значение ", color="#FF6633")
axis[1].plot(filteredStates[1:, 1], label="Оценка фильтра", color="#224411")
axis[1].set_title("velocity")
axis[1].legend()

plt.show()
