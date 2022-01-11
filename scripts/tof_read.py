#!/usr/bin/env python

import serial 
import time
import sys
import rospy
from std_msgs.msg import Float32MultiArray

# Frame Header + Function Mark + Data + Sum Check

# SEND PACKET

#Frame Header 	uint8 	1 	57 	0x57
#Function Mark 	uint8 	1 	00 	0x00
#reserved 	    uint8 	2 	ff 	*
#id 	        uint8 	1 	00 	0
#reserved 	    uint8 	2 	ff 	*
#Sum Check 	    uint8 	1 	3a 	0x3a 
#

TOF_length = 16
TOF_header=(0x57,0x00,0xff)

ser = serial.Serial('/dev/ttyHS5',921600)
ser.flushInput()


def calcCheckSum(data,len):
    TOF_check = 0
    for  k in range(0,len-1):
        TOF_check += data[k]
    TOF_check=TOF_check%256
    return TOF_check

def verifyCheckSum(data, len):    
    if(calcCheckSum(data, len) == data[len-1]):
        #print("TOF data is ok!")
        return 1    
    else:
        #print("TOF data is error!")
        return 0  
  
def TOFread(id_):
    payload = [0x57, 0x10, 0xff, 0xff, id_, 0xff, 0xff]
    payload.append(calcCheckSum(payload,8))
    
    ser.write(payload)                                  #TOF SEND request
    #print("sent")
    
    TOF_data=[]
    
    #time.sleep(0.001)
    
    if ser.inWaiting() >= 16:
        #print("received")
        #for i in range(0,16):
        #    TOF_data = TOF_data + (ord(ser.read(1)), ord(ser.read(1)))
        for i in range(0,16):
            TOF_data.append(ord(ser.read(1)))

       
        if( (TOF_data[0]==TOF_header[0] and TOF_data[1]==TOF_header[1] and TOF_data[2]==TOF_header[2]) and (verifyCheckSum(TOF_data, TOF_length))):
            
        
            id = TOF_data[3]
            system_time = TOF_data[4] | TOF_data[5]<<8 | TOF_data[6]<<16 | TOF_data[7]<<24
            distance = (TOF_data[8]) | (TOF_data[9]<<8) | (TOF_data[10]<<16)
            if(((TOF_data[12]) | (TOF_data[13]<<8) )==0):
                distance = -1                                   #print("Out of range!")

            status = TOF_data[11]
            signal = TOF_data[12] | TOF_data[13]<<8
        
            print("TOF id is: "+ str(id) + " " + str(distance)+'mm')
            #print("TOF system time is: "+str(system_time)+'ms')
            #print("TOF distance is: "+str(distance)+'mm')
            #print("TOF status is: "+str(status))
            #print("TOF signal is: "+str(signal))                    
            
            TOF_values = [ id, system_time, distance, status, signal]       #sensor values returned
            return TOF_values
        else :
            print ("incoherent data")
            ser.flush()
            return -1
            
          

def main(args):
    pub=rospy.Publisher('/tof_reads',Float32MultiArray,queue_size = 1)

    rospy.init_node('tof_read')

    r = rospy.Rate(50)
    
    print( str(ser.is_open) ) 

    values = Float32MultiArray
    values.data = [0]*8

    while not rospy.is_shutdown():
     
        for i in range(8):
            values.data[i] = TOFread(i)  
            time.sleep(0.005) 
      

        pub.publish(values)
        r.sleep()


       