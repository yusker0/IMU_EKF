from vpython import *


class IMUModel:
    def __init__(self, label, position):
        self.center = box(size=vector(5, 5, 5), pos=vector(*position), color=color.red)
        self.imu = box(size=vector(9, 3, 16), pos=vector(*position))
        self.label = text(text=label, color=color.red, align='center', height=2, pos=vector(position[0], position[1]-20, position[2]))
        self.orientation = [0, 0, 0]  # Orientation in radians
        self.position = position

    def rotate_model(self, delta):
        phi = self.orientation[0]  # roll
        psi = self.orientation[1]  # pitch
        omega = self.orientation[2]  # yaw
        # Rotate by X axis
        self.imu.rotate(angle=delta[0])
        self.orientation[0] += delta[0]
        phi = self.orientation[0]

        # Rotate by Y axis (in vpython Z)
        self.imu.rotate(angle=delta[1], axis=vector(sin(omega), sin(phi), -cos(phi) * cos(omega)))
        self.orientation[1] += delta[1]
        psi = self.orientation[1]

        # Rotate by Z axis (in vpython Y)
        self.imu.rotate(angle=delta[2], axis=vector(sin(psi), cos(phi) * cos(psi), sin(phi)))
        self.orientation[2] += delta[2]
        omega = self.orientation[2]

    def setOrientation(self, orientation):
        self.imu.visible = False
        del self.imu
        self.imu = box(size=vector(9, 3, 16), pos=vector(*self.position))
        self.orientation = [0, 0, 0]
        self.rotate_model(orientation)
