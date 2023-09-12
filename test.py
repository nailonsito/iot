import RPi.GPIO as GPIO
import time
import math
from ADCDevice import *
adc = ADCDevice() # Define an ADCDevice class object

import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

from multiprocessing import Process

cred = credentials.Certificate('/home/pi/cred.json')

firebase_admin.initialize_app(cred,{
'databaseURL': "https://login-d1e09.firebaseio.com/"
})

ref = db.reference('ledStatus')
ref2 = db.reference('fanStatus')
ref3 = db.reference('temperature')

ledPin = 11
servoPin = 12
OFFSET_DUTY = 0.5        # define pulse offset of servo
SERVO_MIN_DUTY = 2.5 + OFFSET_DUTY     # define pulse duty cycle for minimum angle of servo
SERVO_MAX_DUTY = 12.5 + OFFSET_DUTY    # define pulse duty cycle for maximum angle of servo
SERVO_DELAY_SEC = 0.001

def setup():
    GPIO.setmode(GPIO.BOARD)       # use PHYSICAL GPIO Numbering
    GPIO.setup(ledPin, GPIO.OUT)   # set the ledPin to OUTPUT mode
    GPIO.output(ledPin, GPIO.LOW)  # make ledPin output LOW level 
    
    global p
    GPIO.setup(servoPin, GPIO.OUT)   # Set servoPin to OUTPUT mode
    GPIO.output(servoPin, GPIO.LOW)  # Make servoPin output LOW level
    p = GPIO.PWM(servoPin, 50)     # set Frequence to 50Hz
    p.start(0)                     # Set initial Duty Cycle to 0
    
    global adc
    if(adc.detectI2C(0x48)): # Detect the pcf8591.
        adc = PCF8591()
    elif(adc.detectI2C(0x4b)): # Detect the ads7830
        adc = ADS7830()
    else:
        print("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n");
        exit(-1)

def servoWrite(angle):      # make the servo rotate to specific angle, 0-180 
    if(angle < 0):
        angle = 0
    elif(angle > 180):
        angle = 180
    dc = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * angle / 180.0 # map the angle to duty cycle
    p.ChangeDutyCycle(dc)
    

def ledLoop():
    while True:
        if ref.get() == 'ledOn':
            GPIO.output(ledPin,True)
            #print('Led is on')
        elif ref.get() == 'ledOff':
            GPIO.output(ledPin,False)
            #print('Led is off')

def fanLoop():
    while True:
        if ref2.get() == 'fanOn':
            for angle in range(0, 181, 1):   # make servo rotate from 0 to 180 deg
                servoWrite(angle)
                time.sleep(SERVO_DELAY_SEC)
            time.sleep(0.5)
            for angle in range(180, -1, -1): # make servo rotate from 180 to 0 deg
                servoWrite(angle)
                time.sleep(SERVO_DELAY_SEC)
            time.sleep(0.5)
            #print('Fan is on')
        elif ref2.get() == 'fanOff':
            p.start(0)
            #print('fan is off')
            
def tempLoop():
    while True:
        value = adc.analogRead(0)        # read ADC value A0 pin
        voltage = value / 255.0 * 3.3        # calculate voltage
        Rt = 10 * voltage / (3.3 - voltage)    # calculate resistance value of thermistor
        tempK = 1/(1/(273.15 + 25) + math.log(Rt/10)/3950.0) # calculate temperature (Kelvin)
        tempC = tempK -273.15        # calculate temperature (Celsius)
        print ('ADC Value : %d, Voltage : %.2f, Temperature : %.2f'%(value,voltage,tempC))
        time.sleep(0.01)
        ref3.set(tempC)

        
def destroy():
    adc.close()
    p.stop()
    GPIO.cleanup() 

if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    setup()
    try:
        p1 = Process(target=ledLoop)
        p1.start()
        p2 = Process(target=fanLoop)
        p2.start()
        p3 = Process(target=tempLoop)
        p3.start()
        p1.join()
        p2.join()
        p3.join()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        destroy()


