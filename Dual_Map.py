class BlitManager:
    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for sub-classes of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all of the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all of the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
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
import numpy as np

import mpu6050
mpu = mpu6050.mpu6050(0x68)
#VARIABLES
resolution = 1
mx_r_record = int(160*resolution)
mx_c_record = int(160*resolution)
matrix = np.zeros((mx_r_record,mx_c_record))
RMAX = 8

#FIGURE PYPLOT SETUP
#fig = plt.figure(figsize=(4.8,2.8),dpi=100)
fig = plt.figure(figsize=(10,5),dpi=100)
gs = gridspec.GridSpec(2,3,fig,height_ratios= [10,1],width_ratios= [5,7,2])


#SUBPLOT SETUP
axes=fig.add_subplot(gs[0,0],polar=True)
bar=fig.add_subplot(gs[1,0])
im = fig.add_subplot(gs[0,1])
bframe=fig.add_subplot(gs[1,1])
yaw_axis=fig.add_subplot(gs[0,2],polar=True)
lidar_polar = axes
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)

#LASER SETUP
ports = ydlidar.lidarPortList();
port = "/dev/ydlidar";
for key, value in ports.items():
    port = value;
    
laser = ydlidar.CYdLidar();
laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 3);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 45.);
laser.setlidaropt(ydlidar.LidarPropMinAngle, -45.);
laser.setlidaropt(ydlidar.LidarPropMaxRange, 32.0);
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01);
scan = ydlidar.LaserScan()

#INIT SUBPLOT
angle = []
ran = []
art=axes.scatter(angle, ran,color='yellow',animated=True,s=20) 
art2=bar.plot([0,.7],[0.7,0.7],color='cyan',animated=True)[0] 
art2A=bar.plot([0,.5],[0.5,0.5],color='red',animated=True)[0]
art2B=bar.plot([0,.25],[0.25,0.25],color='green',animated=True)[0]
art3=bframe.plot([0,0.5],[.5,0.5])[0]
art4_im=im.imshow(matrix,cmap='viridis')
art5_yaw=yaw_axis.scatter(1,1,color='cyan',s=47)
plt.axis([0,1,0,1])
bar.set_ylim((0,1))
bar.set_xlim((0,8))
bframe.set_xlim((0,12))

plt.show(block=False)
plt.pause(.1)

axes.set_thetamin(-45)
axes.set_thetamax(45)
axes.set_theta_direction(-1)
yaw_axis.set_thetamin(-90)
yaw_axis.set_thetamax(90)

#BLIT
bm = BlitManager(fig.canvas,[art,art2,art2A,art2B,art3,art4_im,art5_yaw,
                            axes.set_title('LINE SCAN'),bar.set_title('DISTANCE (m)'),bframe.set_title('FPS'),im.set_title('IMAGE SCAN'),
                            yaw_axis.set_title('HEAD ANGLE'),
                            bar.legend(['FARTHEST','MIDDLE','MINIMUM'],fontsize=9)])
plt.tight_layout()
fig.canvas.draw()

#INIT LASER
ret = laser.initialize();
time.sleep(.5)
ret = laser.turnOn();

count = 0

def call():
    tstart = time.time()
    angle = []
    ran = []
    r = laser.doProcessSimple(scan);
    ay = mpu_axis()
    angle_y = ((ay/1.5)*80)+3
    mx_r = 160-int(((angle_y+((mx_r_record/2)-1))))
    for point in scan.points:
        a=point.angle
        angle.append(a);
        r = point.range*.2333
        ran.append(r)
        mx_c = 160-int(((a+.8)/1.6)*(mx_c_record-1))
        if mx_c <0: 
            mx_c=0
        if mx_c >=mx_c_record:
            mx_c=mx_c_record-1
        if mx_r <0:
            mx_r=0
        if mx_r >=mx_r_record:
            mx_r=mx_r_record-1
        
        matrix[mx_r,mx_c] = r
    art.set_offsets(np.c_[angle,ran])
    ran.sort()
    art2.set_xdata([0,ran[-1]])
    axes.set_ylim((0,ran[-1]))
    axes.relim()
    art2A.set_xdata([0,ran[-int((len(ran)*.4))]])
    art2B.set_xdata([0,ran[-int((len(ran)*.9))]])
    
    art4_im.set_data(matrix)
    art4_im.set_clim(vmin = 0,vmax = ran[-1])
    
    art5_yaw.set_offsets(np.c_[ay,1])
    bm.update()
    art3.set_xdata([0,1/(time.time()-tstart)])

    global count
    reset_at = 10
    count += 1
    if count >=reset_at:
        percent_reduc = .9
        mx_reduction = 1
        for r in range(len(matrix)):
            if not mx_reduction: break 
            for c in range(len(matrix)):
                v = matrix[r,c]*percent_reduc
                matrix[r,c] = v
        count = 0
    
    #print('FRAMES',1/(time.time()-tstart))

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

