
import ydlidar
import time
import matplotlib
matplotlib.use('GTK3agg')
import matplotlib.pyplot as plt
plt.style.use('dark_background')
import matplotlib.animation as animation
import numpy as np

#IMPORT MPU6050
import mpu6050
mpu = mpu6050.mpu6050(0x68)
#laser
laser = ydlidar.CYdLidar();

#___________________________________________________________________________________________________________________#

#VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES
c_high = 'yellow'
c_low = 'black'
c_mid = 'green'
fs = 10
enable_2dscan = True
enable_polarscan = 1
enable_polarrot = 0
res_compute = 0

resolution = 1
mx_r_record = int(160*resolution)
mx_c_record = int(160*resolution)
matrix = np.zeros((mx_r_record,mx_c_record))

enable_reset = 1
reset_at = 50

START_LIDAR_ANIMATE = 1
#VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES VARIABLES

#___________________________________________________________________________________________________________________#


global_count = 0
if  enable_polarrot and enable_polarscan:
    fig, axis = plt.subplots(1,3,gridspec_kw={'width_ratios': [3,3,1]})
    axis[0] = fig.add_subplot(131)
    axis[1] = fig.add_subplot(132,polar = True)
    axis[2] = fig.add_subplot(133,polar = True)
    fig.set_facecolor(c_low)
    fig.tight_layout()
elif not enable_polarscan and enable_polarrot:
    fig, axis = plt.subplots(1,2,gridspec_kw={'width_ratios': [3,3]})
    axis[0] = fig.add_subplot(121)
    axis[1] = fig.add_subplot(122,polar = True)
    fig.set_facecolor(c_low)
    fig.tight_layout()
elif enable_polarscan and not enable_polarrot:
    fig, axis = plt.subplots(1,2,gridspec_kw={'width_ratios': [3,3]})
    axis[0] = fig.add_subplot(121)
    axis[1] = fig.add_subplot(122,polar = True)
    fig.set_facecolor(c_low)
    fig.tight_layout()
else:
    fig, axis = plt.subplots(1,2)
    axis[0] = fig.add_subplot(1,1,1)
    fig.set_facecolor(c_low)

#Setup Lidar
def lidar_SETUP():
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
        port = value;
    
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 8.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);
lidar_SETUP()
scan = ydlidar.LaserScan()
def accel():
    return float(mpu.get_accel_data().get('x')), float(mpu.get_accel_data().get('y')), float(mpu.get_accel_data().get('z'))
def gyro():
    return float(mpu.get_gyro_data().get('x')),float(mpu.get_gyro_data().get('y')),float(mpu.get_gyro_data().get('z'))
#MPU AXIS
def mpu_axis():
   x,y,z = accel()
   rootr = np.sqrt((x*x)+(z*z))
   roll = np.arctan(y/rootr)
   rootp = np.sqrt((y*y)+(z*z))
   pitch = np.arctan(x/rootp)
   rooty = np.sqrt((x*x)+(y*y))
   yaw = np.arctan(z/rooty)
   return roll,pitch,yaw
#ANIMATE PLOT
#t_size = 2000
def mx_reset():
    global global_count
    global_count=0
    for r in np.arange(0,len(matrix)):
        for c in np.arange(0,len(matrix)):
            matrix[r,c] = 0
def mx_reduction():
    percent_reduc = .05
    for r in np.arange(0,len(matrix)):
        for c in np.arange(0,len(matrix)):
            v = matrix[r,c]*percent_reduc
            matrix[r,c] -= v
def scan_points():
    laser.doProcessSimple(scan)
    a = []
    r = []
    for p in scan.points:
        if p.angle<-1: break
        a.append(p.angle)
        r.append(p.range)
    return a,r
def scan_points_withres():
    laser.doProcessSimple(scan)
    a = []
    r = []
    scan_points = scan.points
    length_p = len(scan_points)
    length_p_short = int(length_p*resolution)
    for p_i in range(0,length_p_short):
        p_v = int((p_i/length_p_short)*length_p)
        p = scan_points[p_v]
        if p.angle<-1: break
        a.append(p.angle)
        r.append(p.range)
    return a,r
def scan_choose():
    if res_compute: return scan_points_withres()
    else: return scan_points()
def plot(frame):
    laser.doProcessSimple(scan)
    a,r = scan_choose()
    for i in a:
        break
        mx_x = int(((p.angle)/3)*160)
        
        x,y,z = mpu_axis()
        angle_y = round(((z/1.5)*80),0)+3
        mx_y = int((160-(angle_y+80)))
        if mx_x <0: 
            mx_x=0
        if mx_y <0: 
            mx_y=0
        if mx_x >=160:
            mx_x=159
        if mx_y >=160:
            mx_y=159

        matrix[2,mx_x]=float(p.range)
        #print(p.angle)
        if p.angle<=-1: break
    a_a = []
    b_b = []
    x,y,z = mpu_axis()
    count_t=0
    global global_count
    global_count+=1
    for i in a:
        if not enable_2dscan: break
        mx_x = int(((i)/3)*(mx_c_record-1))
        angle_y = ((y/1.5)*80)+3
        mx_y = int(((angle_y+((mx_r_record/2)-1))))
        b_b.append(mx_y)
        
        if mx_x <0: 
            mx_x=0
        if mx_x >=mx_c_record:
            mx_x=mx_c_record-1
        if mx_y <0:
            mx_y=0
        if mx_y >=mx_r_record:
            mx_y=mx_r_record-1
        
        matrix[mx_y,mx_x] = r[count_t]
        count_t+=1
    if enable_2dscan:
        axis_i = len(axis)
        Label='COMPILED 2D SCANS'
        Xlabel='LIDAR ANGLE, 0-170 DEG'
        Ylabel='HEAD ANGLE, 0-170 DEG'
        fw = 'bold'
        Cmap='viridis'
        if axis_i >1:
            axis[0].clear()
            axis[0].set_xlabel(Xlabel,color=c_high,size=fs,fontweight=fw)
            axis[0].set_ylabel(Ylabel,color=c_high,size=fs,fontweight=fw)
            axis[0].set_facecolor(c_low)
            axis[0].set_title(label=Label,color=c_high,size=fs,fontweight=fw)
            axis[0].imshow(matrix,cmap=Cmap)
            axis[0].tick_params(colors=c_high, which='both',size= fs)
            if res_compute: mx_reduction()
            if enable_reset:
                if global_count >= reset_at: mx_reset()
        else:
            plt.clear()
            plt.set_xlabel(Xlabel,color=c_high,size=fs,fontweight=fw)
            plt.set_ylabel(Ylabel,color=c_high,size=fs,fontweight=fw)
            plt.set_facecolor(c_low)
            plt.set_title(label=Label,color=c_high,size=fs,fontweight=fw)
            plt.imshow(matrix,cmap=Cmap)
            plt.tick_params(colors=c_high, which='both',size= fs)
            if res_compute: mx_reduction()
            if enable_reset:
                if global_count >= reset_at: mx_reset()
    if enable_polarscan:
        axis_i = len(axis)
        Label='2D MAP'
        fw = 'bold'
        if axis_i<= 3:
            axis[1].clear()
            axis[1].tick_params(colors=c_high, which='both',size=fs)
            axis[1].set_facecolor(c_low)
            axis[1].set_title(label=Label,color=c_high,size=fs,fontweight=fw)
            axis[1].scatter(a,r,s=20,c=c_high)
    if enable_polarrot:
        axis_i = len(axis)
        Label='FRONT OF SCANNER'
        fw = 'bold'
        if axis_i== 3:
            axis[2].clear()
            axis[2].set_title(label=Label,color=c_high,size=fs,fontweight=fw)
            axis[2].scatter(a[int(len(a)/2)],5,s=50,c=c_high)
        elif axis_i==2:
            axis[1].clear()
            axis[1].set_title(label=Label,color=c_high,size=fs,fontweight=fw)
            axis[1].scatter(a[int(len(a)/2)],5,s=50,c=c_high)
    print("returned",time.localtime().tm_sec,':seconds')
def init_plot():
    
    return
    plt.ylim(-90,90)
    return
if START_LIDAR_ANIMATE:
    init_laser = laser.initialize()
    print(init_laser)
    time.sleep(.5)
    laser.turnOn()
    on = laser.turnOn()
    if on:
        ani = animation.FuncAnimation(fig,plot,interval=10,init_func=init_plot())
        plt.show()
