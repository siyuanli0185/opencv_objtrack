#!/usr/bin/python
from xbee import XBee
import serial
import time
import os
from datetime import datetime

#PORT='/dev/tty.usbmodem1411' #change port to current
ser = serial.Serial('COM1', 9600, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=1, rtscts=1, dsrdtr=1)

#Creates API Object
xbee=XBee(ser,escaped=True)


#Xbee shield mac addresses
CO3='\x00\x13\xa2\x00\x40\xA8\x7A\xD8' #Coordinator
EP1='\x00\x13\xA2\x00\x40\xAB\x70\xAC' #Robot 1

#endless loop
while True:
	try:
		
		#Checks to see if file exists
		file1=os.path.exists ('data1.txt')
				
		if (file1==True):
			
			with open('data1.txt', 'r') as f1:
				data1 = f1.read()


			print ('\nSending Data')
			print (datetime.now())
			print ('Robot 1 Angle & Distance: ',data1)

			#sends data to robot 1
			xbee.tx_long_addr(dest_addr=EP1,data= data1)
			print 'waiting for response'
			response=xbee.wait_read_frame()
			print 'Response is: ', response 

			
			
			
			
			

		#deletes file after read in to prevent corruption
			
			
			#5 second time interval--increase if needed
			time.sleep(1) 
		
		if (file1==False):
			print ('\nData Files Were Not Found\nScanning for File Again in: ')
			print('3 ')
			time.sleep(1)
			print ('2')
			time.sleep(1)
			print('1')
			time.sleep(1)
			
	#to end program prest ctrl-c		
	except KeyboardInterrupt:
		break	
    
#close serial connection 
ser.close() 

			
        

        

		
			
