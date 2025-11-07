import os
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
plt.style.use('dark_background')
import matplotlib.animation as animation
import numpy as np

RMAX = 32.0


fig = plt.figure()
lidar_polar = plt.subplot(polar=True)
lidar_polar.autoscale_view(True,True,True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)
ports = ydlidar.lidarPortList();
port = "/dev/ydlidar";
for key, value in ports.items():
    port = value;
    
laser = ydlidar.CYdLidar();
laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 8.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 3);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 45.);
laser.setlidaropt(ydlidar.LidarPropMinAngle, -45.);
laser.setlidaropt(ydlidar.LidarPropMaxRange, 32.0);
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01);
scan = ydlidar.LaserScan()

def animate(num):    
    tstart = time.time()
    r = laser.doProcessSimple(scan);
    if r:
        angle = []
        ran = []
        
        for point in scan.points:
            angle.append(point.angle);
            ran.append(point.range*.23);

        lidar_polar.clear()
        lidar_polar.scatter(angle, ran,color='yellow')
        lidar_polar.set_thetamin(-45)
        lidar_polar.set_thetamax(45)
        lidar_polar.set_theta_direction(-1)
        print('FPS:' , (time.time()-tstart))
ret = laser.initialize();
time.sleep(.5)
ret = laser.turnOn();
if ret:
    ani = animation.FuncAnimation(fig, animate, interval=50)
    plt.show()
