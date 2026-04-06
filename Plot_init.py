import subprocess
import os
import matplotlib.pyplot as plt
plt.style.use('dark_background')
from matplotlib.widgets import RadioButtons

fig,ax = plt.subplots()
start = 'START'
ex = 'EXIT'
rad = RadioButtons(ax,labels=('WELCOME TO SCANNER, PLEASE SELECT ONE',
								'DUAL MAP','SINGLE MAP',ex))
def select(l):
	if l=='DUAL MAP':
		print('Processing File')
		os.system('python3 /home/pi/Desktop/SOURCE_CODE_LIDAR/PyplotER_LIDAR.py')
		plt.close()
		os.system('python3 /home/pi/Desktop/SOURCE_CODE_LIDAR/Plot_init.py')
	if l =='SINGLE MAP':
		print('Processing File')
		os.system('python3 /home/pi/Desktop/SOURCE_CODE_LIDAR/SINGLE_MAP.py')
		plt.close()
		os.system('python3 /home/pi/Desktop/SOURCE_CODE_LIDAR/Plot_init.py')
	if l == ex:
		plt.close()
		
rad.on_clicked(select)
plt.show()
