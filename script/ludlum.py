#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import time

import serial
import serial.tools.list_ports
import argparse
import sys
import glob

ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))



#set the serial port settings
# set_ser = serial.Serial('/dev/input')
set_ser = serial.Serial('/dev/usb/hiddev0')
#set_ser = serial.Serial('/dev/ttyUSB1')
# set_ser.port="/dev/ttyUSB1"             #the location of the USB port 
set_ser.baudrate=38400             #baud rate of 1MHz
set_ser.parity = serial.PARITY_NONE
set_ser.stopbits=serial.STOPBITS_ONE
set_ser.bytesize = serial.EIGHTBITS
set_ser.timeout=0.5

#set_ser.open()

message="RH1\r\n"
# set_ser.write(str.encode(message))
set_ser.write(message.encode('utf-8'))
d=set_ser.read(5000)
#print(d)
#print(list(d))
#header = list(d)
header=str(d.decode('utf-8').strip())

if not header:
    print("Device not connected. Quiting")
    sys.exit(0)
else:
    print("First device header:")
    #print("   "+header)


#open the output filename
f=open("res.txt", 'w')

#data header
f.write("Time (s)\tNeutron Dose (uSv/hr)\tGamma Dose (uSv/hr)\n")

n = 100
number_of_frames = 10
gammaData = []
neutronData=[]
data=[]

#draw histogram each time this is called
def update_hist(num, data):

    global f

    #send message to survey meter. 'RR' requests dose
    message="RR\r\n"
    set_ser.write(message.encode('utf-8'))

    #read response
    d=set_ser.read(5000)
    
    if len(d) == 22:

        #parse response
        neutronDose=float(d[0:6])/100
        gammaDose = float(d[7:13])/100


        f.write(str(round(time.time(),2))+"\t"+str(neutronDose)+"\t"+str(gammaDose)+"\n")

        #don't add 0 events to plot. They will be saved to file though.
        if gammaDose != 0 and gammaDose < 100:
            gammaData.append(gammaDose)
        if neutronDose != 0 and neutronDose <100:
            neutronData.append(neutronDose)


    #get maximum dose for both neutrons and gammas, for use in the histogram range
    maxn=1;
    maxg=1;
    if len(gammaData) !=0:
        maxg = max(gammaData)

    if len(neutronData) !=0:
        maxn = max(neutronData)
        
    maxTot = max(maxg, maxn, 1)
      
            

    #plot the histograms
    plt2.cla()
    plt2.hist(neutronData,n, alpha=0.5, label='Neutron dose', range=([0, maxTot]))
    plt2.hist(gammaData  ,n, alpha=0.5, label='$\gamma$ dose', range=([0, maxTot]));
    plt.xlabel("dose ($\mu$Sv/hr)")
    plt.ylabel("Counts")
    plt.legend(loc='upper right')


#fig, ax = plt.subplots()
#plt.subplots_adjust(bottom=0.2)
fig = plt.figure()

animation = animation.FuncAnimation(fig, update_hist, number_of_frames, fargs=(data, ) )


nf=0
#reset button action
class Index(object):
    ind=0
    def reset(self,event):
        global gammaData;
        global neutronData;

        gammaData=[]
        neutronData=[]

    def save(self, event):
        global nf
        global f
        dotLoc=args.filename.find('.')
        newFile = args.filename[:dotLoc]+"_"+str(nf)+args.filename[dotLoc:]
        nf=nf+1
        print("Closed "+str(f.name))
        f.close()
        f=open(newFile, 'w')
        f.write("Time (s)\tNeutron Dose (uSv/hr)\tGamma Dose (uSv/hr)\n")

        global gammaData;
        global neutronData;

        gammaData=[]
        neutronData=[]
        
        
        
#divide plot into graph and button    
button1 = plt.subplot2grid((12, 12), (11, 10), colspan=2) #button
button2 = plt.subplot2grid((12, 12), (11, 8), colspan=2) #button
plt2 = plt.subplot2grid((7, 7), (0, 0), colspan=7, rowspan=6) #graph

#reset button
callback=Index()
breset = Button(button1, 'Reset Plot')
breset.on_clicked(callback.reset)

bsave = Button(button2, 'Save Data')
bsave.on_clicked(callback.save)

plt.show()


set_ser.close()

print("Closed "+str(f.name))
f.close()
