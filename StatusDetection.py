#!/usr/bin/python

import RPi.GPIO as GPIO
import os, time

GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.IN)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.HIGH)
print ("Raspberry Pi tell Power-Management-HAT that I am running by pin21")

while True:
	if (GPIO.input(20)):
		print ("Power-Management-HAT tell Raspberry Pi you need to poweroff by pin20")
		os.system("sudo shutdown -h now")
		break
	time.sleep(0.5)
