"""
Class that holds information about drones. No need to run.
"""


class DroneData:

    def __init__(self):
        self.ip = ''
        self.x = 0.0  # drone x
        self.y = 0.0  # drone y
        self.z = 0.0  # drone z
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.start_x = None  # brain starting x
        self.start_y = None  # brain starting y
        self.start_z = None  # brain starting z

    def string_dict(self):
        return {
            'ip': self.ip,
            'x': str(self.x),
            'y': str(self.y),
            'z': str(self.z),
            'target_x': str(self.target_x),
            'target_y': str(self.target_y),
            'target_z': str(self.target_z),
            'start_x': str(self.start_x),
            'start_y': str(self.start_y),
            'start_z': str(self.start_z)
        }

    def real_x(self):
        return self.x

    def real_y(self):
        return self.y

    def real_z(self):
        return self.z

    def set_target(self, x, y, z):
        """
        Set drone target coordinates based on brain coordinates
        :param x:
        :param y:
        :param z:
        :return:
        """
        self.target_x = x - self.real_x() + self.x
        self.target_y = y - self.real_y() + self.y
        self.target_z = z - self.real_z() + self.z

    def __str__(self):
        return str(self.string_dict())
