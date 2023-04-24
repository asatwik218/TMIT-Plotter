from vpython import *
from time import *
import numpy as np
import math
import serial
import csv
from time import time

# -----notes------------------------------------------------------
# adjust the height and width of the graphs and scenes
# update the function drawGraph()
# check the new data implementation using arduino
# check the new addCSV() implementation    
# ----------------------------------------------------------------

# -----------------------------USER INPUTS------------------------------------------------------------------------------------------------

# CHANGE FOR USING SERIAL
isTesting = True

#CHANGE COM PORT AND BAUDRATE
COM_PORT = '/dev/cu.usbmodem379B345C35341'
BAUD_RATE = 9600

#DEFINE DATA STRUCTURE
# WARNING : the 1st four values need to be the quaternions for teapot
SERIAL_DATA_STRUCT = ["q0", "q1", "q2", "q3", "alt"]

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# screen size 
display_x = 1350
display_y = 1080.0-40.0

#global variables
SERIAL_DATA_INP = {}
roll=0
pitch=0
yaw=0
t=0

# serial 
if(not isTesting):
    ad = serial.Serial(COM_PORT, BAUD_RATE)
    sleep(1)

#Graph 1: altitude 
graph1=graph(align="left",title="altitude",xtitle="time",ytitle="altitude",width=display_x/2.5,height=display_y/3,fast=False)
g1 = gcurve(color=color.blue,graph=graph1)
g1.plot(1,5)
g1.plot(2,6)

#Graph 2:velocity
graph2=graph(align="left",title="velocity",xtitle="time",ytitle="velocity",width=display_x/2.5,height=display_y/3,fast=False)
g2 = gcurve(color=color.red,graph=graph2)
g2.plot(1,5)
g2.plot(2,6)

#scene 1:sim
scene1 = canvas(align="right",title="",width=display_x/3.0,height=display_y/3.5)
scene1.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene1.forward=vector(-1,-1,-1)
scene1.select()
frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))
body = cylinder(length=6,radius=0.5,opacity=0.8,pos=vector(-3,0,0),color=color.red)
nosecone = cone(pos=vector(3,0,0),radius=0.5, color=color.red)
myObj=compound([body,nosecone])

#Graph 3:pressure
graph3=graph(align="left",scroll=True,xmin=0,xmax=100,title="velocity1",xtitle="time",ytitle="velocity1",width=display_x/2.5,height=display_y/3,fast=False)
g3 = gcurve(color=color.red,graph=graph3)
g3.plot(0,0)
g3.plot(0,0)

#Graph 4: gyro
graph4=graph(align="left",scroll=True,xmin=0,xmax=100,title="velocity2",xtitle="time",ytitle="velocity2",width=display_x/2.5,height=display_y/3,fast=False)
g4 = gcurve(color=color.blue,graph=graph4)
g4.plot(0,0)
g4.plot(0,0)

#scene 2: raw data
scene2 = canvas(align="right",title="",width=display_x/3.0,height=display_y/3.5)
scene2.select()
l = label(color=color.red,text="data",box=False,pos=vector(0,5,0),height=20,)

# Switching CSV on/off
def switchCSV():
    global isStart
    isStart = not isStart
    if(isStart):
        vb.text = "stop CSV"
    else:
        vb.text = "start csv"
vb=button(bind=switchCSV,text="start csv")

# Updating SIM/Teapot
def updateSim():

    k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
    y=vector(0,1,0)
    s=cross(k,y)
    v=cross(s,k)
    vrot=v*cos(roll)+cross(k,v)*sin(roll)
    
    frontArrow.axis=k
    sideArrow.axis=cross(k,vrot)
    upArrow.axis=vrot
    myObj.axis=k
    myObj.up=vrot
    sideArrow.length=2
    frontArrow.length=4
    upArrow.length=1

# Updating graphs
def drawGraph():  
    global t 
    # TODO: need to change this according to data
    # g1.plot(t,yaw)
    # g2.plot(t,yaw)
    g3.plot(t,roll)
    g4.plot(t,pitch)

    t+=1

# updating raw data text 
def updateText():
    global l
    text = ""
    for x, y in SERIAL_DATA_INP.items():
        text += f"{x} : {y}\n" 
    l.text = text

#updating CSV
def addCSV(): 
    # get data from dict to arr for writing into csv file
    CSVdata = []
    for i in SERIAL_DATA_INP:
        CSVdata.append(SERIAL_DATA_INP[i])

    # write data to csv file
    with open("FlightData.csv", "a") as f: #csv
        writer = csv.writer(f, delimiter=",")
        writer.writerow(CSVdata)

while (True):
    try:
        while (ad.inWaiting()==0):
            pass

        dataPacket=ad.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        
        if(len(splitPacket) == len(SERIAL_DATA_STRUCT)):
            
            # getting quaternions from serial data
            SERIAL_DATA_INP["q0"] = float(splitPacket[0])
            SERIAL_DATA_INP["q1"] = float(splitPacket[1])
            SERIAL_DATA_INP["q2"] = float(splitPacket[2])
            SERIAL_DATA_INP["q3"] = float(splitPacket[3])
            # calculation roll, pitch & yaw from quaternions
            roll=-math.atan2(2*(SERIAL_DATA_INP["q0"] * SERIAL_DATA_INP["q1"] + SERIAL_DATA_INP["q2"] * SERIAL_DATA_INP["q3"]),1 - 2*(SERIAL_DATA_INP["q1"]**2 + SERIAL_DATA_INP["q2"]**2))
            pitch=math.asin(2*(SERIAL_DATA_INP["q0"] * SERIAL_DATA_INP["q2"] - SERIAL_DATA_INP["q3"] * SERIAL_DATA_INP["q1"]))
            yaw=-math.atan2(2*(SERIAL_DATA_INP["q0"] * SERIAL_DATA_INP["q3"] + SERIAL_DATA_INP["q1"] * SERIAL_DATA_INP["q2"]),1 - 2*(SERIAL_DATA_INP["q2"]**2 + SERIAL_DATA_INP["q3"]**2)) - np.pi/2

            #getting rest of the data
            for i in range(4,len(SERIAL_DATA_STRUCT)):
                SERIAL_DATA_INP[SERIAL_DATA_STRUCT[i]] = float(splitPacket[i])            

            # updating teapot/simulation
            updateSim()

            #update graphs 
            drawGraph()
            
            #update raw data text
            updateText()

            #add into csv
            if(isStart):
                addCSV()

            rate(100) #refresh rate of vpython

    except Exception as e:
        print(e)