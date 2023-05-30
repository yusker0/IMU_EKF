import numpy as np
from math import sin, cos, pi

RAD_TO_DEG = 180 / pi
DEG_TO_RAD = pi / 180

accelMeasurementSigma = 2 * pi / 3
gyroMeasurementSigma = 250 / 3
processNoiseVariance = 1e-5


# Функция наблюдения - аналог матрицы наблюдения
# Преобразует вектор состояния x в вектор измерений z
def measurementFunction(x):
    measurement = np.zeros(6)
    measurement[0] = sin(x[1]) * cos(x[2])
    measurement[1] = sin(x[0]) * cos(x[2])
    measurement[2] = cos(x[0]) * cos(x[1])
    # measurement[3] = x[3] * RAD_TO_DEG
    # measurement[4] = x[4] * RAD_TO_DEG
    # measurement[5] = x[5] * RAD_TO_DEG
    return measurement


def JacobianMeasurementFunction(x):
    measurement = np.zeros((6, 6))
    measurement[0] = [0, cos(x[1]) * cos(x[2]), -sin(x[2]) * sin(x[1]), 0, 0, 0]
    measurement[1] = [cos(x[0] * cos(x[2])), 0, -sin(x[2]) * sin(x[0]), 0, 0, 0]
    measurement[2] = [-sin(x[0]) * cos(x[1]), -sin(x[1]) * cos(x[0]), 0, 0, 0, 0]
    # measurement[3] = [0, 0, 0, RAD_TO_DEG, 0, 0]
    # measurement[4] = [0, 0, 0, 0, RAD_TO_DEG, 0]
    # measurement[5] = [0, 0, 0, 0, 0, RAD_TO_DEG]
    return measurement


# Функция процесса - аналог матрицы процесса
def stateTransitionFunction(x, dt=0.033):
    newState = np.zeros(6)
    newState[0] = x[0] + x[3] * dt
    newState[1] = x[1] + x[4] * dt
    newState[2] = x[2] + x[5] * dt
    newState[3] = x[3]
    newState[4] = x[4]
    newState[5] = x[5]
    return newState


# Функция процесса - аналог матрицы процесса
def JacobianStateTransitionFunction(x, dt=0.033):
    stateTransitionMatrix = np.eye(6, 6)
    stateTransitionMatrix[0:3, 3:] = np.eye(3, 3) * dt
    return stateTransitionMatrix


Q = np.eye(6, 6)
Q *= 0.8
R = np.array([[1.9785e-5, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 2.1594e-5, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 4.14e-5, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 1.2968e-2, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 1.6684e-2, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.2517e-2]])
P = np.diag(
    [accelMeasurementSigma ** 2, accelMeasurementSigma ** 2, accelMeasurementSigma ** 2, gyroMeasurementSigma ** 2,
     gyroMeasurementSigma ** 2, gyroMeasurementSigma ** 2])


class EKF:
    def __init__(self, x, z_size, f, fJacobian, h, hJacobian, u=None, dt=1):
        # Вектор состояния (State vector)
        self.x = np.array(x)
        # Вектор управления (Control vector)
        self.u = np.array(u) if u is not None else None
        # Функция процесса (State transition model). Показывает связь предыдущего состояния с последующим x_k = f(x_k-1)
        self.f = f
        # Функция переводящая функцию процесса в матрицу Якоби (Jacobian Matrix) для линеаризации
        self.fJacobian = fJacobian

        # Вектор наблюдения (Observation vector)
        self.z = np.zeros(z_size)
        # Вектор инновации (Innovation). y_k = z_k - h(x_k)
        self.y = np.zeros(z_size)
        # Функция наблюдениия (Observation model). Переводит вектор состояния x в вектор наблюдения z
        self.h = h
        # Функция переводящая функцию наблюдения в матрицу Якоби (Jacobian Matrix) для линеаризации
        self.hJacobian = hJacobian

        # Ковариационная матрица априорной (независящей от измернений) оценки состояния (A posteriori estimate covariance matrix)
        self.P = np.eye(len(x))
        # Ковариационная матрица ошибки измерений (Сovariance of the observation noise)
        self.R = np.eye(z_size)
        # Ковариационная матрица ошибки модели (Сovariance of the process noise)
        self.Q = np.eye(len(x))

        # Ковариационная матрица инновации (Innovation covariance )
        self.S = np.zeros((z_size, z_size))
        self.SI = np.zeros((z_size, z_size))  # Обратная матрица ковариционной матрицы S
        # Оптимальное (почти оптимальное в нелинейном случае) усиление Калмана (Kalman gain)
        self.K = np.zeros((len(x), z_size))
        # Еденичная матрица размером с вектор состояния
        self.I = np.eye(len(x))

        self.dt = dt

    def predict(self):
        x = self.x
        u = self.u
        f = self.f
        P = self.P
        Q = self.Q

        if u is None:
            F = self.fJacobian(x, dt=self.dt)
            self.x = f(x, dt=self.dt)
        else:
            F = self.fJacobian(x, u, dt=self.dt)
            self.x = f(x, u, dt=self.dt)

        self.P = F.dot(P).dot(F.T) + Q

    def update(self, z):
        self.z = z
        x = self.x
        h = self.h
        H = self.hJacobian(x)
        P = self.P
        R = self.R
        I = self.I

        y = self.y = z - h(x)
        S = self.S = H.dot(P).dot(H.T) + R
        SI = self.SI = np.linalg.inv(S)
        K = self.K = P.dot(H.T).dot(SI)
        self.x = x + np.array(np.dot(K, y))
        self.x = np.array(self.x[0])
        self.P = (I - K.dot(H)).dot(P)


imuFilter = EKF([0, 0, 0, 0, 0, 0], 6, stateTransitionFunction, JacobianStateTransitionFunction, measurementFunction,
                JacobianMeasurementFunction)
imuFilter.P = P
imuFilter.R = R
imuFilter.Q = Q
