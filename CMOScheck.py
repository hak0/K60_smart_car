# -*- coding: utf-8 -*-
import serial
from ipdb import set_trace
import numpy as np
import matplotlib.pyplot as plt
import cv2

data_length = 128
port_name = 'COM6'
baudrate = 115200

ser = serial.Serial(port_name, baudrate, timeout=1)
# timeout=0: non-blocking mode, return immediately in any case,
# returning zero or more, up to the requested number of bytes

# ser.parity = 'E'
if ser.is_open:
    print(port_name + ' is open')
else:
    print(port_name + 'is not open')
    exit


def converter(temp):
    t_int = int.from_bytes(temp, byteorder='big')  #这是给16进制用的
    #  t_int = int(temp)

    #if temp >= b'\x08\x00':
    #  t_int = (~t_int + 1) & 0x07ff #  t_int = - t_int

    return t_int


plt.ion()

ccol = 120
crow = 60
datas = np.zeros((int(ccol * crow / 4), 1), dtype=np.uint8)
datas_unpack = np.zeros((int(ccol * crow / 4), 8), dtype=np.uint8)
while True:
    return_value = b''
    # read until received signal
    while not return_value == b'\xFF':
        return_value = ser.read()
    # 读取图像
    datas = ser.read(int(ccol * crow / 4))
    datas_1d = np.frombuffer(datas, dtype=np.uint8)
    datas_unpack = np.unpackbits(
        datas_1d.reshape((len(datas_1d), 1)), axis=1)[:, 4:].reshape((crow,
                                                                      ccol))
    # 读取其它数据（光源等）
    ch = ser.read()
    see_light = converter(ch)
    ch = ser.read(2)
    light_x = converter(ch)
    ch = ser.read()
    light_y = converter(ch)

    datas_resize = cv2.resize(
        datas_unpack * 150, (640, 480), interpolation=cv2.INTER_LINEAR)

    try:
        #  plt.clf()
        #  if (see_light == 1):
        #  plt.title('CMOS x='+str(light_x)+' y='+str(light_y))
        #  else:
        #  plt.title('CMOS')
        #  plt.imshow(datas, cmap='gray')
        if (see_light == 1):
            cv2.putText(datas_resize, 'CMOS x='+str(light_x)+' y='+str(light_y), (50,50), cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),1)
        cv2.imshow('CMOS', datas_resize)
        cv2.waitKey(100)
        #  plt.axis([0, 128, 0, 1024])
        #  plt.stem(datas)
        #  plt.grid()
        #  plt.show()
        #  plt.pause(0.2)
    except:
        cv2.destroyAllWindows()
        print("正在关闭串口...")
        ser.close()
        import os
        os.system('pause')
