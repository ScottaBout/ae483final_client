class DroneData:

    def __init__(self):
        self.ip = ''
        self.x = 0
        self.y = 0
        self.z = 0
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        self.opti_x = 0
        self.opti_y = 0

    def string_dict(self):
        return {
            'ip': self.ip,
            'x': str(self.x),
            'y': str(self.y),
            'z': str(self.z),
            'target_x': str(self.target_x),
            'target_y': str(self.target_y),
            'target_z': str(self.target_z),
            'opti_x': str(self.opti_x),
            'opti_y': str(self.opti_y)
        }
