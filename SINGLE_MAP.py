class BlitManager:
    def __init__(self, canvas, animated_artists=()):
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        cv = self.canvas
        fig = cv.figure
        if self._bg is None:
            self.on_draw(None)
        else:
            cv.restore_region(self._bg)
            self._draw_animated()
            cv.blit(fig.bbox)
        cv.flush_events()
import os
import ydlidar
import time
import sys
sys.path
from matplotlib.patches import Arc
from matplotlib import gridspec
import matplotlib.pyplot as plt
plt.style.use('dark_background')
import matplotlib.animation as animation
from matplotlib.widgets import RadioButtons
import numpy as np

RMAX = 8
import mpu6050
mpu = mpu6050.mpu6050(0x68)
fig = plt.figure(figsize=(10,5),dpi=100)
gs = gridspec.GridSpec(2,2,fig,height_ratios= [10,1],width_ratios= [5,1])

#axisE = plt.subplots(2,2,gridspec_kw={'height_ratios': [5,1],'width_ratios': [5,1]})

axes=fig.add_subplot(gs[0,0],polar=True)
bar=fig.add_subplot(gs[1,0])
bframe=fig.add_subplot(gs[1,1])
yaw_axis=fig.add_subplot(gs[0,1],polar=True)
lidar_polar = axes
#lidar_polar.autoscale_view(True,True,True)
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
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 20.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 3);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 45.);
laser.setlidaropt(ydlidar.LidarPropMinAngle, -45.);
laser.setlidaropt(ydlidar.LidarPropMaxRange, 32.0);
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01);
scan = ydlidar.LaserScan()

#axisE = plt.subplots(2,2,gridspec_kw={'height_ratios': [5,1],'width_ratios': [5,1]})
angle = []
ran = []
art=axes.scatter(angle, ran,color='yellow',animated=True,s=10) 
art2=bar.plot([0,.7],[0.7,0.7],color='cyan',animated=True)[0] 
art2A=bar.plot([0,.5],[0.5,0.5],color='red',animated=True)[0]
art2B=bar.plot([0,.25],[0.25,0.25],color='green',animated=True)[0]
art3=bframe.plot([0,0.5],[.5,0.5])[0]
art5_yaw=yaw_axis.scatter(1,1,color='cyan',s=47)

plt.axis([0,1,0,1])
bar.set_ylim((0,1))
bar.set_xlim((0,8))
bframe.set_xlim((0,12))
								
plt.show(block=False)
plt.pause(.1)

axes.set_thetamin(-45)
axes.set_thetamax(45)
axes.set_xticks([])
axes.set_yticks([])
axes.set_theta_direction(-1)
yaw_axis.set_thetamin(-90)
yaw_axis.set_thetamax(90)
yaw_axis.set_yticks([])
bm = BlitManager(fig.canvas,[art,art2,art2A,art2B,art3,art5_yaw,
                            axes.set_title('LINE SCAN'),bar.set_title('DISTANCE (m)'),bframe.set_title('FPS'),yaw_axis.set_title('HEAD ANGLE'),
                            bar.legend(['FARTHEST','MIDDLE','MINIMUM'],fontsize=9)])
plt.tight_layout()
fig.canvas.draw()

ret = laser.initialize();
time.sleep(.5)
ret = laser.turnOn();

maxran = 5


def call():
	global maxran
	tstart = time.time()
	angle = []
	ran = []
	r = laser.doProcessSimple(scan);
	for point in scan.points:
		angle.append(point.angle);
		ran.append(point.range*.23);
	art.set_offsets(np.c_[angle,ran])
	axes.set_ylim((0,maxran))
	axes.relim()

	ran.sort()
	art2.set_xdata([0,ran[-1]])
	art2A.set_xdata([0,ran[-int((len(ran)*.4))]])
	art2B.set_xdata([0,ran[-int((len(ran)*.9))]])
	art5_yaw.set_offsets([mpu_axis(),1])
	x,y,z = gyro()
	g_ave = abs(x)+abs(y)+abs(z)
	if g_ave >=100:
		maxran=max(ran)
	bm.update()
	art3.set_xdata([0,1/(time.time()-tstart)])
	#print(temp())
	#print('FRAMES',1/(time.time()-tstart))
def temp():
	return mpu.get_temp()
def gyro():
    return int(mpu.get_gyro_data().get('x')),int(mpu.get_gyro_data().get('y')),int(mpu.get_gyro_data().get('z'))
def accel():
    return float(mpu.get_accel_data().get('x')), float(mpu.get_accel_data().get('y')), float(mpu.get_accel_data().get('z'))
#MPU AXIS
def mpu_axis():
   x,y,z = accel()
   rooty = np.sqrt((x*x)+(y*y))
   yaw = np.arctan(z/rooty)
   return yaw
while True:
    call()
    
end = time.time()
time.sleep(1)
plt.show()

    
'''
def animate(self):    
  
    return art
   
        
    
    


if ret:
    ani = animation.FuncAnimation(fig, animate, interval=50,blit=0)
    plt.show()

'''
