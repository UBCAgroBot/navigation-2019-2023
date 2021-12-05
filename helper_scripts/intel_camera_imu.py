import pyrealsense2 as rs
import numpy as np
from time import sleep

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

p = initialize_camera()
absolute_gyro = np.array([0.0,0.0,0.0])
try:
    while True:
        f = p.wait_for_frames()
        accel = accel_data(f[0].as_motion_frame().get_motion_data())
        gyro = gyro_data(f[1].as_motion_frame().get_motion_data())
        #print("accelerometer: ", accel)
        absolute_gyro += gyro/4
        print("gyro: ", absolute_gyro)
        

finally:
    p.stop()