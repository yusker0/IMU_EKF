from serial import Serial


class SerialParser:
    def __init__(self):
        self.ser = Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.accOffsets = [0, 0, 0]
        self.gyroOffsets = [0, 0, 0]
    def get_data(self):
        row_data = self.ser.readline().decode(encoding='ascii', errors='ignore').replace('\r', '').replace('\n', '').split('\t')
        while len(row_data) != 7:
            row_data = self.ser.readline().decode(encoding='ascii', errors='ignore').replace('\r', '').replace('\n', '').split('\t')
        return {
            'acc': {
                'x': int(row_data[0]) * 2 / 32768,
                'y': int(row_data[1]) * 2 / 32768,
                'z': int(row_data[2]) * 2 / 32768
            },
            'gyro': {
                'x': int(row_data[3]) * 250 / 32768,
                'y': int(row_data[4]) * 250 / 32768,
                'z': int(row_data[5]) * 250 / 32768
            },
            'btn': True if row_data[6]=='0' else False
        }
    # def calibrate(self, iterations):
    #     for i in range(iterations):
    #         row_data = self.ser.readline().decode(encoding='ascii', errors='ignore').replace('\r', '').replace('\n', '').split('\t')
    #         while len(row_data) != 7:
    #             row_data = self.ser.readline().decode(encoding='ascii', errors='ignore').replace('\r', '').replace('\n', '').split('\t')
    #         self.accOffsets = list(map(lambda x, y: x+y, self.accOffsets, [int(row_data[0]), int(row_data[1]), int(row_data[2]) - 16384]))
    #         self.gyroOffsets = list(map(lambda x, y: x+y, self.gyroOffsets, [int(row_data[i]) for i in range(3, 6)]))
    #     self.accOffsets = list(map(lambda x: round(x/iterations), self.accOffsets))
    #     self.gyroOffsets = list(map(lambda x: round(x/iterations), self.gyroOffsets))
    #     print("Acceleration offsets: ", self.accOffsets)
    #     print("Gyroscope offsets: ", self.gyroOffsets)
