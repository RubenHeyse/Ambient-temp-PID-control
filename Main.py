from time import sleep
import time
import Adafruit_DHT
import RPi.GPIO as IO
import numpy as np
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import datetime
import io


#temp and time setup
sampleTime = 1
firstStageTemp = 0
secondStageTemp = 10
elapsedTime = 0
ambientTemp = 0

#pid setup
Kp = 5
Ki = 0.1
Kd = 0.05
previousError = 0

#Initialise temp sensor data
tempSensor = Adafruit_DHT.DHT11
tempGPIO = 4
temperature = Adafruit_DHT.read_retry(tempSensor, tempGPIO)
tempArray = []

#Initialise motor data
IO.setwarnings(False)
IO.setmode (IO.BCM)
pwmPin = 12
motorOnOffPin = 16
IO.setup(pwmPin,IO.OUT)
IO.setup(motorOnOffPin, IO.OUT)
IO.output(motorOnOffPin, True) #Turn motor on
dutyCycle = 0
p = IO.PWM(pwmPin,100) #100 is the frequency
maxDutyCycle = 85
minDutyCycle = 30
previousDutyCycle = 0

#Initialise screen data
RST = None
DC = 23
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)
disp.begin()
disp.clear()
disp.display()
width = disp.width
height = disp.height
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)
draw.rectangle((0,0,width,height), outline=0, fill=0)
padding = -2
top = padding
bottom = height-padding
x = 0
font = ImageFont.load_default()

#Initialise Tachometer data
IO.setwarnings(False)
IO.setmode (IO.BCM)
tachPin = 20
IO.setup(tachPin, IO.IN)
initialDuty = 30
rpm = 0
endTime = 0
startTime = 0
processTime = datetime.timedelta(endTime-startTime)


def readRPM():
        previousGPIOValue = 0
        pulseCount = 0
        processTime = 0
        startTime = datetime.datetime.now()
        while processTime < 1:
                currentGPIOValue = IO.input(tachPin)
                if currentGPIOValue == 1 and currentGPIOValue != previousGPIOValue:
                        pulseCount += 1
                previousGPIOValue = currentGPIOValue
                endTime = datetime.datetime.now()
                delta = endTime - startTime
                processTime = int(delta.total_seconds())     
        rpm = pulseCount/2*60
        return rpm

def getAmbientTemp():
    temperature = Adafruit_DHT.read_retry(tempSensor, tempGPIO)
    tempArray = (np.asarray(temperature))
    temp = float(tempArray[1])
    return temp

def writeToScreen(temp, rpm):
    draw.rectangle((0,0,width,height), outline=0, fill=0) # reset screen
    draw.text((x, top),       "Ambient: " + str(ambientTemp) + "*C",  font=font, fill=255)
    draw.text((x, top+8),     "Duty Cycle: " + str(dutyCycle),  font=font, fill=255)
    draw.text((x, top+16),     "Target Temp: " + str(temp) + "*C",  font=font, fill=255)
    draw.text((x, top+24),     "RPM: " + str(rpm),  font=font, fill=255)
    disp.image(image)
    disp.display()

def runPID(targetTemp, dutyCycle, elapsedTime):
    integral = 0
    ambientTemp = getAmbientTemp()
    temp = map(int, previousDutyCycleFile)
    previousDutyCycle = temp[0]
    error = targetTemp - ambientTemp
    
    print(previousDutyCycle)

    if integral > maxDutyCycle:
        integral = maxDutyCycle
    elif integral < minDutyCycle:
        integral = minDutyCycle
    else:
        integral += Ki * error

    if elapsedTime == 0:
        elapsedTime = 0.001
        derivative = float((dutyCycle - previousDutyCycle)/elapsedTime)
    else:
        derivative = float((dutyCycle - previousDutyCycle)/elapsedTime)

    dutyCycle -= Kp*error + integral - Kd*derivative
    p.ChangeDutyCycle(dutyCycle)
    previousDutyCycle = dutyCycle
    previousDutyCycleFile.write(str(previousDutyCycle))

#start fan
p.start(dutyCycle)

#open and zero previousDutyCycle.txt
previousDutyCycleFile = open("previousDutyCycle.txt","r+") 
previousDutyCycleFile.write("0")

#Simulate door closed
beginloop = 0
endLoop = 0
beginLoop = datetime.datetime.now()

while elapsedTime < 20:
    runPID(firstStageTemp, dutyCycle, elapsedTime)
    rpm = readRPM()
    writeToScreen(firstStageTemp, rpm)
    sleep(sampleTime)
    endloop = datetime.datetime.now()
    delta = endLoop - beginLoop
    elapsedTime = float(delta.total_seconds()) 
    
previousDutyCycleFile.close()

#Simulate door slightly open
while True:
    runPID(secondStageTemp, dutyCycle, elapsedTime)
    rpm = readRPM()
    writeToScreen(secondStageTemp, rpm)
    sleep(sampleTime)
    endloop = datetime.datetime.now()
    delta = endLoop - beginLoop
    elapsedTime = float(delta.total_seconds()) 
