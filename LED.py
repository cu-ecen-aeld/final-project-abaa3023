#Header files
import RPi.GPIO as GPIO
import time

#PIR sensor placement
pir_sensor_pin = 11
#LED placememnt
led_pin = 3

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
#Set PIR sensor as input
GPIO.setup(11, GPIO.IN)
#Set LED as ouput
GPIO.setup(3, GPIO.OUT)

while True:
    #Read sensor value
    i=GPIO.input(11)
    #If nothing detected, nothing happens
    if i==0:
        GPIO.output(3, False)
        time.sleep(1)
    #If someone detected, light up LED
    elif i==1:
        GPIO.output(3, True)
        time.sleep(1)