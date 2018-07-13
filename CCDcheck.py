# -*- coding: utf-8 -*-
import serial
import time
import numpy as np
import matplotlib.pyplot as plt

data_length = 128
port_name = 'COM6'
baudrate = 115200

ser = serial.Serial(port_name, baudrate, timeout=0)
# timeout=0: non-blocking mode, return immediately in any case,
# returning zero or more, up to the requested number of bytes

# ser.parity = 'E'
if ser.is_open:
    print(port_name + ' is open')
else:
    print(port_name + 'is not open')
    exit

plt.ion()

datas = np.zeros((data_length, 1), dtype=np.uint8)
datas_1d = np.zeros((data_length, 1), dtype=np.uint8)
while True:
    for i in range(20):
        temp = ser.read()
        while not temp == b'\xFF':
            # 读取起始标记
            temp = ser.read()
        # 读取图像
        datas = ser.read(data_length)
        datas_1d = np.frombuffer(datas, dtype=np.uint8)
        print(datas_1d)
        #  datas_2d = np.vstack([datas_1d]*10)
        try:
            plt.clf()
            plt.axis([0, 128, 0, 128])
            plt.plot(datas_1d)
            plt.title('CCD')
            plt.show()
            plt.pause(0.01)
            #  cv2.imshow('CCD', cv2.resize(datas_2d, (512, 40)))
            #  cv2.waitKey(100)
            
        except:
            print("正在关闭串口...")
            ser.close()
            import os
            os.system('pause')
    ser.close()
    time.sleep(1)
    ser.open()

